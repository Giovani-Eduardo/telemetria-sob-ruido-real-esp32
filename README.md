# Telemetria sob Ruído Real — ESP32

Este repositório contém os códigos utilizados na realização do trabalho de transmissão de dados com três ESP32.

- <code>tx/tx.ino</code> — código da placa transmissora.
- <code>rx/rx.ino</code> — código da placa receptora.
- <code>noise/noise.ino</code> — código da placa que injeta ruído na comunicação.

Cada pasta também possui o arquivo <code>protocolo.h</code>, com as definições compartilhadas do protocolo. As cópias são mantidas em cada pasta para que os três sketches possam ser abertos e compilados separadamente no Arduino IDE.

Para utilizar, abra <code>tx.ino</code>, <code>rx.ino</code> e <code>noise.ino</code> nas respectivas pastas no Arduino IDE, selecione <code>ESP32 Dev Module</code> e envie cada sketch à placa correspondente.
