// Placa B: RX. A decisao de aceitar/descartar NUNCA consulta a referencia.
// UART2: RX=16, TX=17 (retorno sem transmissao). USB=115200.
#include <Arduino.h>
#include <esp_timer.h>
#include "protocolo.h"
using namespace Proto;

Algorithm algorithm = SUM8;
Block input;
Sequence tracking;
char command[32]; size_t commandSize = 0;
uint32_t lastFrameAt = 0, recoveryFrom = 0;
bool linkTimedOut = false, recovering = false;

// Contagem automatica de cada lote. Os detalhes continuam no CSV acima;
// este resumo aparece quando o TX para e o timeout de enlace ocorre.
struct Statistics {
  uint32_t ok = 0, corrupted = 0, lost = 0, undetected = 0;

  void reset() { ok = corrupted = lost = undetected = 0; }
  uint32_t total() const { return ok + corrupted + lost + undetected; }
} statistics;

// Medicoes acumuladas para a tabela de tempos do relatorio. O tempo do TX
// vem na copia de depuracao; a copia nao interfere na decisao do RX.
struct TimingStatistics {
  uint64_t txCalcTotalUs = 0, rxCheckTotalUs = 0, recoveryTotalUs = 0;
  uint32_t txCalcSamples = 0, rxCheckSamples = 0, recoverySamples = 0;

  void reset() {
    txCalcTotalUs = rxCheckTotalUs = recoveryTotalUs = 0;
    txCalcSamples = rxCheckSamples = recoverySamples = 0;
  }
  void addTxCalculation(uint32_t us) { txCalcTotalUs += us; ++txCalcSamples; }
  void addRxVerification(uint32_t us) { rxCheckTotalUs += us; ++rxCheckSamples; }
  void addRecovery(uint32_t us) { recoveryTotalUs += us; ++recoverySamples; }
} timings;

struct Decision {
  bool present = false, checked = false, accepted = false;
  uint8_t bytes[64] = {}; size_t size = 0;
  uint8_t remainder = 0;
  uint32_t checkUs = 0, startUs = 0, recoveryUs = 0;
  int32_t gap = 0;
  const char *reason = "sem_quadro";
} pending;

void csvHeader() {
  Serial.println("sequencia_ref;algoritmo;sequencia_rx;residuo;decisao;payload_confere;status;verificacao_us;calculo_tx_us;inicio_rx_us;recuperacao_us;lacuna_operacional;motivo;quadro_tx_hex;quadro_rx_hex");
}

void printSummary() {
  const uint32_t detectedErrors = statistics.corrupted + statistics.undetected;
  const float undetectedRate = detectedErrors
    ? (100.0f * statistics.undetected / detectedErrors)
    : 0.0f;

  Serial.println("# RESUMO_DO_LOTE");
  Serial.printf("# tecnica=%s;total=%lu;OK=%lu;CORROMPIDO=%lu;PERDIDO=%lu;NAO_DETECTADO=%lu;taxa_nao_deteccao=%.2f%%\n",
    name(algorithm), (unsigned long)statistics.total(),
    (unsigned long)statistics.ok, (unsigned long)statistics.corrupted,
    (unsigned long)statistics.lost, (unsigned long)statistics.undetected,
    undetectedRate);

  Serial.println("# TEMPOS_DO_LOTE");
  if (timings.txCalcSamples) {
    Serial.printf("# calculo_tx_medio_us=%.3f;amostras=%lu\n",
      float(timings.txCalcTotalUs) / timings.txCalcSamples,
      (unsigned long)timings.txCalcSamples);
  }
  if (timings.rxCheckSamples) {
    Serial.printf("# verificacao_rx_media_us=%.3f;amostras=%lu\n",
      float(timings.rxCheckTotalUs) / timings.rxCheckSamples,
      (unsigned long)timings.rxCheckSamples);
  }
  if (timings.recoverySamples) {
    Serial.printf("# recuperacao_media_ms=%.3f;eventos=%lu\n",
      float(timings.recoveryTotalUs) / (1000.0f * timings.recoverySamples),
      (unsigned long)timings.recoverySamples);
  } else {
    Serial.println("# recuperacao_media_ms=NA;eventos=0");
  }
}

void decideFrame(const Block &b) {
  // Fase 1: protocolo. Nenhum argumento contem a referencia verdadeira.
  if (pending.present) Serial.println("# REFERENCIA_AUSENTE: salvar log e repetir a coleta.");
  pending = Decision(); pending.present = true;
  pending.size = b.size; memcpy(pending.bytes, b.data, b.size);
  pending.startUs = b.first;
  lastFrameAt = b.first; linkTimedOut = false;

  // Busca o SOF no bloco. Em caso de falha, o timeout de 8 ms encerra o bloco.
  size_t offset = 0;
  while (offset + 1 < b.size && !(b.data[offset] == SOF0 && b.data[offset+1] == SOF1)) ++offset;
  if (b.overflow || offset + FRAME_SIZE != b.size ||
      !shape(b.data + offset, b.size - offset)) {
    pending.reason = "timeout_enquadramento_8ms";
    if (!recovering) { recoveryFrom = micros(); recovering = true; }
    return;
  }
  pending.size = FRAME_SIZE; memcpy(pending.bytes, b.data + offset, FRAME_SIZE);
  int64_t startCheck = esp_timer_get_time();
  pending.remainder = residue(algorithm, pending.bytes + 2, BODY_SIZE);
  pending.checkUs = uint32_t(esp_timer_get_time() - startCheck);
  pending.checked = true; pending.accepted = pending.remainder == 0;
  pending.reason = pending.accepted ? "residuo_zero" : "checagem_falhou";
  if (pending.accepted) pending.gap = tracking.accept(seq(pending.bytes));
  else tracking.discard(); // payload descartado, nao entregue a aplicacao
  if (recovering) {
    pending.recoveryUs = uint32_t(micros() - recoveryFrom);
    recovering = false;
  }
}

void logReference(const Block &b) {
  // Fase 2: somente instrumentacao. Nao altera tracking nem a decisao anterior.
  // A referencia traz o quadro inteiro para prova byte a byte, alem do payload.
  const uint8_t *original = b.data + 4;
  const uint32_t txCalculationUs = get32(b.data + 14);
  bool payloadMatches = pending.checked && memcmp(pending.bytes + 5, original + 5, PAYLOAD_SIZE) == 0;
  const char *status = !pending.checked ? "PERDIDO" :
    (!pending.accepted ? "CORROMPIDO" : (payloadMatches ? "OK" : "NAO_DETECTADO"));
  const char *decision = !pending.checked ? "nao_verificado" :
    (pending.accepted ? "aceito" : "descartado");

  if (strcmp(status, "OK") == 0) ++statistics.ok;
  else if (strcmp(status, "CORROMPIDO") == 0) ++statistics.corrupted;
  else if (strcmp(status, "PERDIDO") == 0) ++statistics.lost;
  else ++statistics.undetected;

  // Toda referencia inclui a medicao feita no TX. A verificacao e a
  // recuperacao so entram na media quando de fato aconteceram no RX.
  timings.addTxCalculation(txCalculationUs);
  if (pending.checked) timings.addRxVerification(pending.checkUs);
  if (pending.recoveryUs) timings.addRecovery(pending.recoveryUs);

  char sent[FRAME_SIZE*2+1], received[129];
  hex(original, FRAME_SIZE, sent); hex(pending.bytes, pending.size, received);
  Serial.printf("%u;%s;", seq(original), name(algorithm));
  if (pending.checked) Serial.printf("%u;%02X;", seq(pending.bytes), pending.remainder);
  else Serial.print(";;");
  Serial.printf("%s;%s;%s;%lu;%lu;%lu;%lu;%ld;%s;%s;%s\n", decision,
    pending.checked ? (payloadMatches ? "sim" : "nao") : "na",
    status, (unsigned long)pending.checkUs, (unsigned long)txCalculationUs,
    (unsigned long)pending.startUs, (unsigned long)pending.recoveryUs,
    (long)pending.gap, pending.reason, sent, received);
  // Se nem um bloco chegou, PERDIDO acima e uma classificacao experimental.
  // A falta operacional e percebida por lacuna/timeout, jamais pela referencia.
  pending = Decision();
}

void finishBlock() {
  if (debugShape(input.data, input.size) && !input.overflow) logReference(input);
  else decideFrame(input);
  input.clear();
}

void handleCommand(const char *s) {
  if (strcmp(s, "sum") == 0) algorithm = SUM8;
  else if (strcmp(s, "crc") == 0) algorithm = CRC8;
  else if (strcmp(s, "reset") != 0 && strcmp(s, "header") != 0) {
    Serial.println("# Comandos: sum | crc | reset | header. Usar com TX parado."); return;
  }
  if (strcmp(s, "header") != 0) {
    tracking.reset(); pending = Decision(); input.clear();
    lastFrameAt = 0; linkTimedOut = false; recovering = false;
    statistics.reset();
    timings.reset();
  }
  Serial.printf("# RX configurado: %s. Esperando sequencia 0.\n", name(algorithm));
  csvHeader();
}

void setup() {
  Serial.begin(115200);
  Serial2.setRxBufferSize(2048);
  Serial2.begin(BAUD, SERIAL_8N1, 16, 17, false, 20000UL, 1);
  delay(300);
  Serial.println("# RX pronto. sum/crc deve coincidir com TX. Monitor em Nova linha.");
  csvHeader();
}

void loop() {
  if (input.expired(micros())) finishBlock();
  while (Serial2.available()) {
    uint32_t now = micros();
    if (input.expired(now)) finishBlock();
    input.push(uint8_t(Serial2.read()), now);
  }
  if (lastFrameAt && !linkTimedOut && uint32_t(micros()-lastFrameAt) > LINK_TIMEOUT_US) {
    linkTimedOut = true;
    Serial.println("# TIMEOUT_ENLACE_250ms: sem novo quadro; quantidade final por sequencia/referencia, sem contar descartados duas vezes. Normal apos FIM do TX.");
    printSummary();
  }
  while (Serial.available()) {
    char c = char(Serial.read()); if (c == '\r') continue;
    if (c == '\n') { command[commandSize] = 0; handleCommand(command); commandSize = 0; }
    else if (commandSize < sizeof(command)-1) command[commandSize++] = c;
  }
  delay(1);
}
