# Line Follower e PCBs – PUCPR

Repositório com códigos de seguidor de linha (Arduino) e arquivos de hardware (PCBs).

## Estrutura

- firmware/
  - line-follower-rcx-2022/ — Seguidor de linha usando QTRSensors.
  - line-follower-2023/ — Seguidor de linha com PID e rotina de calibração.
  - examples/
    - simple-two-sensors/ — Exemplo simples com 2 sensores digitais.
    - three-sensors/ — Exemplo com 3 sensores digitais.
  - experiments/
    - atmega328p-usart-asm/ — Exemplo AVR Assembly (USART no ATmega328P).
- hardware/
  - pcb-follow-2023/ — Projeto PCB 2023.
  - pcb-follow-2023-calouros/ — Projeto PCB 2023 (calouros).
  - sumo-sensor-pcb/ — PCB de sensor para sumô.
- docs/
  - images/ — Imagens e assets.

## Projetos principais (código)

- 2022 (QTR + PID)
  - Arquivo principal: [Cod RCX 2022/Follow_line_cod_2/Follow_line_cod_2.ino](Cod RCX 2022/Follow_line_cod_2/Follow_line_cod_2.ino)
  - Exemplo de calibração: [`sensor_calibrate`](Cod RCX 2022/Follow_line_cod_2/Follow_line_cod_2.ino)

- 2023 (PID + calibração dinâmica)
  - Arquivo principal: [Projeto 2023/Codigo 2023/LF2_Line_follower/LF2_Line_follower.ino](Projeto 2023/Codigo 2023/LF2_Line_follower/LF2_Line_follower.ino)
  - Funções: [`linefollow`](Projeto 2023/Codigo 2023/LF2_Line_follower/LF2_Line_follower.ino), [`calibrate`](Projeto 2023/Codigo 2023/LF2_Line_follower/LF2_Line_follower.ino)

- AVR Assembly (USART/Serial no ATmega328P)
  - Fonte: [Projeto Follow/teste_serial/teste_serial/main.asm](Projeto Follow/teste_serial/teste_serial/main.asm)

## Como compilar e gravar

### Arduino (UNO – ATmega328P)
- Com Arduino IDE: abra o .ino e selecione Board “Arduino Uno”, porta correta, Compile/Upload.
- Com Arduino CLI:
  - Compile: `arduino-cli compile -b arduino:avr:uno path/para/sketch`
  - Upload: `arduino-cli upload -p /dev/ttyACM0 -b arduino:avr:uno path/para/sketch`

Exemplos:
- 2022: `arduino-cli compile -b arduino:avr:uno "Cod RCX 2022/Follow_line_cod_2"`
- 2023: `arduino-cli compile -b arduino:avr:uno "Projeto 2023/Codigo 2023/LF2_Line_follower"`

### AVR Assembly (Atmel Studio 7 – Windows)
- Abra a solução: firmware/experiments/atmega328p-usart-asm/teste_serial.atsln
- Dispositivo: ATmega328P, Tool: AVRISP mkII/USBasp/etc., Programar Flash.

Artefatos úteis:
- Listagem: [Projeto Follow/teste_serial/teste_serial/Debug/teste_serial.lss](Projeto Follow/teste_serial/teste_serial/Debug/teste_serial.lss)

## Hardware (PCBs)

- hardware/pcb-follow-2023/ — arquivos .json, imagens e esquemáticos.
- hardware/pcb-follow-2023-calouros/ — variantes para calouros.
- hardware/sumo-sensor-pcb/ — PCB de sensor para robô de sumô.

## Requisitos de biblioteca (Arduino)

- QTRSensors (Pololu) para os projetos com array QTR:
  - Usada em: [Cod RCX 2022/Follow_line_cod_2/Follow_line_cod_2.ino](Cod RCX 2022/Follow_line_cod_2/Follow_line_cod_2.ino)
  - Instalar via Library Manager ou: `arduino-cli lib install "Pololu QTR Sensors"`

## Licença

Defina a licença do projeto (ex.: MIT, Apache-2.0) conforme política da equipe.