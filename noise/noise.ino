// Placa C: ESP32 CLASSICO / WROOM-32, pacote Arduino-ESP32 3.x.
// GPIO16 = UART2 observa TX antes do resistor; GPIO26 = mesma linha (start bit).
// GPIO27 = UART1 observa RX depois do resistor; GPIO25 = resistor 1k -> base NPN.
// Nunca ligue GPIO25 diretamente a linha UART. Veja LEIA_PRIMEIRO.md.
#include <Arduino.h>
#include <esp_arduino_version.h>
#if ESP_ARDUINO_VERSION_MAJOR < 3
#error "Instale esp32 by Espressif Systems 3.x no Gerenciador de Placas."
#endif
#if !CONFIG_IDF_TARGET_ESP32
#error "Este sketch usa ESP32 classico (WROOM-32), nao ESP32-C3/S2/S3."
#endif
#include <driver/gptimer.h>
#include <esp_random.h>
#include <soc/soc.h>
#include <soc/gpio_reg.h>
#include "protocolo.h"
using namespace Proto;

constexpr int DRIVE = 25, EDGE = 26;
constexpr uint32_t TIMER_HZ = 10000000; // 0,1 us por tick
// Compensacao ajustada a partir do ensaio: o pulso estava chegando
// aproximadamente um periodo de bit tarde (115200 baud ~= 87 ticks).
// 117 ticks = 11,7 us. Confirmar novamente pela coluna valido.
constexpr uint16_t PHASE_ADVANCE_TICKS = 117;
constexpr uint16_t MARGIN_TICKS = 10; // 1 us dentro de cada extremidade do pulso
constexpr uint32_t BYTE_GAP_TICKS = 5000; // 500 us entre bytes, TX usa >=2 ms
static_assert(PHASE_ADVANCE_TICKS <= 140, "Compensacao deve ficar entre 0 e 140 ticks.");

struct EdgeEvent { uint16_t offset; uint8_t level; uint8_t bits; };
struct BytePlan { uint8_t size; EdgeEvent events[4]; };
BytePlan plans[BODY_SIZE]; // no maximo dois pulsos por byte nos modos disponiveis
uint8_t targetMask[BODY_SIZE] = {};
volatile bool armed = false;
volatile uint8_t nextByte = 0, activeByte = 0, activeEvent = 0;
volatile uint64_t byteOrigin = 0, lastFalling = 0;
volatile uint64_t pulseStart[8][8] = {}, pulseEnd[8][8] = {};
volatile uint32_t timerFaults = 0, maxLateness = 0;
gptimer_handle_t pulseTimer = nullptr;
uint8_t selectedMode = 0, frameMode = 0;
Block cleanBlock, noisyBlock;
uint8_t sentFrame[FRAME_SIZE] = {}, observed[64] = {};
size_t observedSize = 0;
bool haveClean = false, haveNoisy = false, observedOverflow = false;
char command[32]; size_t commandSize = 0;

void IRAM_ATTR driveLow(bool active) {
  REG_WRITE(active ? GPIO_OUT_W1TS_REG : GPIO_OUT_W1TC_REG, 1UL << DRIVE);
}

void IRAM_ATTR scheduleAt(uint64_t time) {
  gptimer_alarm_config_t alarm = {};
  alarm.alarm_count = time;
  if (gptimer_set_alarm_action(pulseTimer, &alarm) != ESP_OK) {
    ++timerFaults; driveLow(false);
  }
}

bool IRAM_ATTR onAlarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *data, void *) {
  uint8_t b = activeByte, e = activeEvent;
  if (e >= plans[b].size) { driveLow(false); return false; }
  EdgeEvent event = plans[b].events[e];
  uint64_t now = 0; gptimer_get_raw_count(timer, &now);
  uint64_t due = byteOrigin + event.offset;
  uint32_t late = now > due ? uint32_t(now - due) : 0;
  if (late > maxLateness) maxLateness = late;
  // Se o inicio ja perdeu a janela, nao deixe um pulso atrasado atingir stop/debug.
  if (event.level && e + 1 < plans[b].size &&
      now + 20 >= byteOrigin + plans[b].events[e+1].offset) {
    ++timerFaults; driveLow(false); activeEvent += 2;
  } else {
    driveLow(event.level != 0);
    for (uint8_t k = 0; k < 8; ++k) if (event.bits & (1u << k)) {
      if (event.level) pulseStart[b][k] = now;
      else pulseEnd[b][k] = now;
    }
    ++activeEvent;
  }
  if (activeEvent < plans[b].size)
    scheduleAt(byteOrigin + plans[b].events[activeEvent].offset);
  return false;
}

void IRAM_ATTR onFalling() {
  uint64_t now = 0; gptimer_get_raw_count(pulseTimer, &now);
  uint64_t gap = now - lastFalling; lastFalling = now;
  if (gap < BYTE_GAP_TICKS || !armed) return; // transicao interna, nao start bit
  uint8_t b = nextByte++;
  if (b >= BODY_SIZE) { armed = false; return; }
  if (nextByte >= BODY_SIZE) armed = false; // termina ANTES da copia de depuracao
  if (plans[b].size == 0) return;
  byteOrigin = now; activeByte = b; activeEvent = 0;
  scheduleAt(now + plans[b].events[0].offset);
}

uint16_t boundary(unsigned bitPeriods) {
  return uint16_t((uint64_t(bitPeriods) * TIMER_HZ + BAUD/2) / BAUD);
}

void preparePlan() {
  armed = false; driveLow(false);
  memset(targetMask, 0, sizeof(targetMask));
  memset(plans, 0, sizeof(plans));
  for (uint8_t b = 0; b < 8; ++b) for (uint8_t k = 0; k < 8; ++k)
    pulseStart[b][k] = pulseEnd[b][k] = 0;
  timerFaults = 0; maxLateness = 0; frameMode = selectedMode;
  if (frameMode == 1 || frameMode == 2) {
    uint8_t a = uint8_t(random(64)); targetMask[a/8] |= uint8_t(1u << (a%8));
    if (frameMode == 2) {
      uint8_t b; do { b = uint8_t(random(64)); } while (b == a);
      targetMask[b/8] |= uint8_t(1u << (b%8));
    }
  } else if (frameMode == 3) {
    uint8_t b = uint8_t(random(8)), length = uint8_t(random(3,9));
    uint8_t first = uint8_t(random(9-length));
    targetMask[b] = uint8_t(((1u << length) - 1u) << first);
  } else if (frameMode == 4) { // extra: colisao de SUM8, nao modo 0..3
    targetMask[3] = 0x80; targetMask[4] = 0x80;
  } else if (frameMode == 5) { // extra: erro multiplo do polinomio CRC8
    targetMask[3] = 0x01; targetMask[4] = 0x07;
  }
  for (uint8_t b = 0; b < 8; ++b) {
    uint8_t k = 0;
    while (k < 8) {
      if (!(targetMask[b] & (1u << k))) { ++k; continue; }
      uint8_t first = k, bits = 0;
      while (k < 8 && (targetMask[b] & (1u << k))) bits |= uint8_t(1u << k++);
      // Bit 0 de dados comeca depois de 1 periodo (start). UART envia LSB antes.
      // Usa contas assinadas e limita o inicio a 1 tick. Sem isso, um
      // avanco maior que o inicio do bit 0 viraria um uint16 enorme.
      int32_t startTick = int32_t(boundary(1 + first)) + MARGIN_TICKS - PHASE_ADVANCE_TICKS;
      int32_t endTick = int32_t(boundary(1 + k)) - MARGIN_TICKS - PHASE_ADVANCE_TICKS;
      if (startTick < 1) startTick = 1;
      if (endTick <= startTick) endTick = startTick + 1;
      EdgeEvent on = {uint16_t(startTick), 1, bits};
      EdgeEvent off = {uint16_t(endTick), 0, bits};
      BytePlan &p = plans[b]; p.events[p.size++] = on; p.events[p.size++] = off;
    }
  }
  nextByte = 0;
  armed = true; // SOF ja reconhecido; o proximo start bit e o campo LEN.
}

void printInjection() {
  if (!haveClean || !haveNoisy) return;
  uint8_t changed[8] = {};
  bool valid = observedSize == FRAME_SIZE && !observedOverflow && timerFaults == 0;
  unsigned attempted = 0, changedCount = 0;
  if (observedSize == FRAME_SIZE) {
    if (observed[0] != SOF0 || observed[1] != SOF1) valid = false;
    for (uint8_t b = 0; b < 8; ++b) {
      changed[b] = sentFrame[b+2] ^ observed[b+2];
      // Exige exatamente as mudancas fisicamente possiveis nos bits selecionados.
      // Se o alvo ja era zero, nao deve mudar. Se era um, deve aparecer zero.
      if (changed[b] != uint8_t(sentFrame[b+2] & targetMask[b])) valid = false;
    }
  }
  for (uint8_t b = 0; b < 8; ++b) for (uint8_t k = 0; k < 8; ++k) {
    if (targetMask[b] & (1u << k)) {
      ++attempted;
      if (!pulseStart[b][k] || pulseEnd[b][k] <= pulseStart[b][k]) valid = false;
    }
    if (changed[b] & (1u << k)) ++changedCount;
  }
  char targets[17], changes[17], txHex[21], rxHex[129];
  hex(targetMask, 8, targets); hex(changed, 8, changes);
  hex(sentFrame, FRAME_SIZE, txHex); hex(observed, observedSize, rxHex);
  Serial.printf("%u;%u;%s;%s;%u;%u;%u;", seq(sentFrame), frameMode,
    targets, observedSize == FRAME_SIZE ? changes : "indeterminado",
    attempted, changedCount, valid ? 1 : 0);
  bool separator = false;
  for (uint8_t b = 0; b < 8; ++b) {
    for (uint8_t e = 0; e < plans[b].size; e += 2) {
      uint8_t bits = plans[b].events[e].bits, k = 0;
      while (k < 8 && !(bits & (1u << k))) ++k;
      if (separator) Serial.print('|');
      separator = true;
      Serial.printf("b%u.mask%02X@%llu-%llu", b, bits,
        (unsigned long long)(pulseStart[b][k]/10),
        (unsigned long long)(pulseEnd[b][k]/10));
    }
  }
  Serial.printf(";%lu;%.1f;%s;%s\n", (unsigned long)timerFaults,
    double(maxLateness)/10.0, txHex, rxHex);
  haveClean = false; haveNoisy = false;
}

void finishClean() {
  if (shape(cleanBlock.data, cleanBlock.size) && !cleanBlock.overflow) {
    memcpy(sentFrame, cleanBlock.data, FRAME_SIZE); haveClean = true;
  } else if (!debugShape(cleanBlock.data, cleanBlock.size))
    Serial.println("# ERRO_OBSERVACAO_TX: verifique GPIO16 e tempos; repetir coleta.");
  cleanBlock.clear();
}
void finishNoisy() {
  if (!debugShape(noisyBlock.data, noisyBlock.size)) {
    memcpy(observed, noisyBlock.data, noisyBlock.size); observedSize = noisyBlock.size;
    observedOverflow = noisyBlock.overflow; haveNoisy = true;
  }
  noisyBlock.clear();
}
void header() {
  Serial.println("sequencia;modo;alvos_hex;mudou_hex;bits_tentados;bits_mudados;valido;pulsos_byte_mascara_inicio_fim_us;falhas_timer;atraso_max_us;quadro_tx_hex;quadro_linha_hex");
}
void handleCommand(const char *s) {
  if (strlen(s) == 1 && s[0] >= '0' && s[0] <= '3') selectedMode = uint8_t(s[0]-'0');
  else if (strcmp(s, "demo") == 0) selectedMode = 4;
  else if (strcmp(s, "democrc") == 0) selectedMode = 5;
  else if (strcmp(s, "header") == 0) { header(); return; }
  else if (strcmp(s, "reset") == 0) {
    armed = false; driveLow(false); cleanBlock.clear(); noisyBlock.clear();
    haveClean = false; haveNoisy = false;
  } else { Serial.println("# Comandos: 0 | 1 | 2 | 3 | demo | democrc | reset | header"); return; }
  Serial.printf("# modo=%u (vale a partir do proximo quadro)\n", selectedMode);
}

void setup() {
  Serial.begin(115200);
  pinMode(DRIVE, OUTPUT); digitalWrite(DRIVE, LOW);
  pinMode(EDGE, INPUT);
  Serial2.setRxBufferSize(2048); Serial1.setRxBufferSize(2048);
  // TX17 e TX33 desta placa ficam SEM fio; sao apenas pinos de saida nao usados.
  Serial2.begin(BAUD, SERIAL_8N1, 16, 17, false, 20000UL, 1);
  Serial1.begin(BAUD, SERIAL_8N1, 27, 33, false, 20000UL, 1);
  randomSeed(esp_random());
  gptimer_config_t config = {};
  config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  config.direction = GPTIMER_COUNT_UP; config.resolution_hz = TIMER_HZ;
  ESP_ERROR_CHECK(gptimer_new_timer(&config, &pulseTimer));
  gptimer_event_callbacks_t callbacks = {}; callbacks.on_alarm = onAlarm;
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(pulseTimer, &callbacks, nullptr));
  ESP_ERROR_CHECK(gptimer_enable(pulseTimer));
  ESP_ERROR_CHECK(gptimer_start(pulseTimer));
  attachInterrupt(digitalPinToInterrupt(EDGE), onFalling, FALLING);
  delay(300);
  Serial.println("# RUIDO pronto em modo 0. Monitor: 115200, Nova linha.");
  Serial.println("# validez=0 indica ajuste de bancada; nao usar lote como resultado final.");
  header();
}

void loop() {
  if (cleanBlock.expired(micros())) finishClean();
  if (noisyBlock.expired(micros())) finishNoisy();
  while (Serial2.available()) {
    uint32_t now = micros();
    if (cleanBlock.expired(now)) finishClean();
    cleanBlock.push(uint8_t(Serial2.read()), now);
    if (cleanBlock.size == 2 && cleanBlock.data[0] == SOF0 && cleanBlock.data[1] == SOF1) {
      if (haveClean || haveNoisy) {
        Serial.println("# PAR_INCOMPLETO: faltou observacao TX/RX; repetir coleta.");
        haveClean = false; haveNoisy = false;
      }
      preparePlan();
    }
  }
  while (Serial1.available()) {
    uint32_t now = micros();
    if (noisyBlock.expired(now)) finishNoisy();
    noisyBlock.push(uint8_t(Serial1.read()), now);
  }
  printInjection();
  while (Serial.available()) {
    char c = char(Serial.read()); if (c == '\r') continue;
    if (c == '\n') { command[commandSize] = 0; handleCommand(command); commandSize = 0; }
    else if (commandSize < sizeof(command)-1) command[commandSize++] = c;
  }
  delay(1);
}
