// Placa A: ESP32 DevKit / WROOM-32. Arduino-ESP32 3.x.
// Serial USB: comandos e log. UART2: RX=16, TX=17, 115200 8N1.
#include <Arduino.h>
#include <esp_timer.h>
#include "protocolo.h"
using namespace Proto;

Algorithm algorithm = SUM8;
uint16_t sequenceNumber = 0;
uint32_t remaining = 0, nextFrame = 0, lastFrame = 0;
uint8_t demoPayload = 0; // 0=normal, 1=colisao SUM8, 2=colisao CRC8
char command[48]; size_t commandSize = 0;

void handleCommand(const char *s) {
  if (remaining && strcmp(s, "stop") != 0) {
    Serial.println("# Aguarde terminar ou envie stop."); return;
  }
  if (strcmp(s, "sum") == 0) algorithm = SUM8;
  else if (strcmp(s, "crc") == 0) algorithm = CRC8;
  else if (strcmp(s, "normal") == 0) demoPayload = 0;
  else if (strcmp(s, "demo") == 0) demoPayload = 1;
  else if (strcmp(s, "democrc") == 0) demoPayload = 2;
  else if (strcmp(s, "header") == 0) {
    Serial.println("sequencia;algoritmo;inicio_us;intervalo_us;calculo_tx_us;quadro_tx_hex"); return;
  }
  else if (strcmp(s, "stop") == 0) remaining = 0;
  else if (strncmp(s, "run ", 4) == 0) {
    long n = strtol(s + 4, nullptr, 10);
    if (n < 1 || n > 1000000) { Serial.println("# Use run 10 ou run 1000."); return; }
    sequenceNumber = 0; remaining = uint32_t(n);
    nextFrame = micros() + 500000; lastFrame = 0;
  } else { Serial.println("# Comandos: sum | crc | run 10 | run 1000 | stop | normal | demo | democrc | header"); return; }
  Serial.printf("# algoritmo=%s;restantes=%lu;payload=%s\n", name(algorithm),
                (unsigned long)remaining, demoPayload ? "demo" : "normal");
}

void transmitFrame() {
  uint8_t frame[FRAME_SIZE] = {SOF0, SOF1, LEN};
  frame[3] = uint8_t(sequenceNumber); frame[4] = uint8_t(sequenceNumber >> 8);
  // Telemetria sintetica de dois canais de 16 bits. Nenhum sensor e necessario.
  uint16_t channelA = uint16_t(1000u + sequenceNumber * 37u);
  uint16_t channelB = uint16_t(2000u + sequenceNumber * 53u);
  frame[5] = uint8_t(channelA); frame[6] = uint8_t(channelA >> 8);
  frame[7] = uint8_t(channelB); frame[8] = uint8_t(channelB >> 8);
  // Apenas ensaio extra para demonstrar a colisao da soma; nao usar nas tabelas.
  if (demoPayload) {
    frame[5] = frame[6] = demoPayload == 1 ? 0x80 : 0xFF;
    frame[7] = 0; frame[8] = 0;
  }
  int64_t t0 = esp_timer_get_time();
  frame[9] = code(algorithm, frame + 2, BODY_SIZE - 1);
  uint32_t calcUs = uint32_t(esp_timer_get_time() - t0);
  uint32_t start = micros();
  uint32_t interval = lastFrame ? uint32_t(start - lastFrame) : 0;
  lastFrame = start;

  // Espacamento explicito entre bytes: o injetor identifica cada start bit.
  // delay(2) organiza bytes; a duracao dos pulsos de ruido e feita por GPTimer.
  for (size_t i = 0; i < FRAME_SIZE; ++i) {
    Serial2.write(frame[i]); Serial2.flush(true);
    if (i + 1 < FRAME_SIZE) delay(2);
  }
  // O RX encerra a decisao por comprimento/timeout ANTES da referencia.
  delay(25);
  uint8_t debug[DEBUG_SIZE];
  memcpy(debug, DBG, 4); memcpy(debug + 4, frame, FRAME_SIZE);
  put32(debug + 14, calcUs);
  Serial2.write(debug, sizeof(debug)); Serial2.flush(true);

  char textFrame[FRAME_SIZE*2+1]; hex(frame, sizeof(frame), textFrame);
  Serial.printf("%u;%s;%lu;%lu;%lu;%s\n", sequenceNumber, name(algorithm),
    (unsigned long)start, (unsigned long)interval, (unsigned long)calcUs, textFrame);
  ++sequenceNumber; --remaining;
  nextFrame = start + PERIOD_US;
  if (!remaining) Serial.println("# FIM: lote concluido.");
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(BAUD, SERIAL_8N1, 16, 17);
  delay(300);
  Serial.println("# TX pronto. Monitor: 115200; final de linha: Nova linha.");
  Serial.println("sequencia;algoritmo;inicio_us;intervalo_us;calculo_tx_us;quadro_tx_hex");
  Serial.println("# Envie sum ou crc, depois run 10. TX inicia parado.");
}

void loop() {
  while (Serial.available()) {
    char c = char(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') { command[commandSize] = 0; handleCommand(command); commandSize = 0; }
    else if (commandSize < sizeof(command)-1) command[commandSize++] = c;
  }
  if (remaining && int32_t(micros() - nextFrame) >= 0) transmitFrame();
  delay(1);
}
