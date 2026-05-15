# Prothesis controller firmware

[English](#english) | [Русский](#русский)

---

## English

### Overview

This project is an ESP32 firmware for a prosthesis controller. It reads flex and EMG inputs, drives servo outputs, estimates contact force from an FSR sensor, and exchanges configuration and telemetry with a mobile app over BLE.

The firmware is built around 5 logical finger pairs. Each pair can be configured independently and use either a flex sensor or a single-channel EMG input as its motion source.

---

### Documentation

- Landing page: [prothesis.ru](https://prothesis.ru)
- Controller guide: [Open controller and assembly guide](https://drive.google.com/file/d/1TqZTdqmVpPDhjSL2WnfT9wohUkeKcAlp/view?usp=sharing)
- Android app guide: [Open application guide](https://drive.google.com/file/d/1A2usxykovqEe099k2ItJ9acGboUhyZ13/view?usp=sharing)
- Backend guide: [Open backend guide](https://drive.google.com/file/d/1RrIl7wzfEjwOqdwEEhMq3LTtUpla6rIw/view?usp=sharing)
- 3D model package: [Open STL and STEP package](https://drive.google.com/drive/folders/1bwpGPP83HMJFmotW1bT_C9SNT2XIXNeM?usp=sharing)
- Android app repository: [graevsky/6thFinger-App](https://github.com/graevsky/6thFinger-App)
- Backend repository: [graevsky/6thFinger-Backend](https://github.com/graevsky/6thFinger-Backend)

---

### Features

- per-pair input selection: `Flex` or `EMG`
- BLE configuration exchange with chunked JSON transport
- persistent settings storage in ESP32 NVS
- optional BLE session authentication with a 4-digit PIN
- live servo control commands from the mobile app
- realtime telemetry streaming for sensors, servo state, EMG state, and vibro output
- force-feedback
- realtime 1-channel EMG classifier integrated directly into firmware

---

### Technologies

- `ESP32` with the Arduino framework
- `PlatformIO` (`env:esp32dev`)
- `NimBLE-Arduino` for BLE GATT services and characteristics
- `ESP32Servo` for servo control
- `ArduinoJson` for settings, ACK, and telemetry payloads
- `Preferences` for persisted configuration
- `FreeRTOS` mutexes for serialized BLE transmission
- `LEDC` PWM for vibro output
- EMG model asset in `include/`

---

### Architecture

The firmware is split into several layers:

- `src/main.cpp` - entry point, application wiring, main loop
- `src/ble/` - BLE GATT setup, config protocol, ACK responses, telemetry transport, settings persistence
- `src/control/` - sensor acquisition, filtering, resistance conversion, servo updates, vibro feedback, runtime scheduling
- `src/emg/` - EMG sampling, realtime classifier wrapper, snapshot detection, bend/unfold state machine
- `src/config/` - settings schema and shared telemetry model
- `include/emg_features.h` - shared realtime DSP/feature extraction helpers for model
- `include/realtime_1ch_binary_model.h` - 1-channel binary EMG model and inference helpers
- `platformio.ini` - target board, build flags, and library dependencies

Short runtime flow:

1. `main.cpp` boots serial, BLE, and the control subsystem.
2. `BleApp` restores settings from NVS and exposes the BLE service.
3. `Control` schedules flex, FSR, and EMG sampling and computes actuator targets.
4. `EmgEngine` converts realtime EMG windows into snapshot events and bend/unfold actions.
5. Servo and vibro outputs are updated, and telemetry is sent back over BLE when enabled.

---

### Components

Main firmware components:

- BLE application layer with config, ACK, telemetry, and live-servo channels
- versioned settings model for FSR, vibro, flex, servo, pair input, EMG, and auth PIN
- control subsystem with smoothing, outlier rejection, ADC stabilization, and actuator scheduling
- EMG runtime with a binary classifier and gesture "state machine"
- telemetry snapshot model shared between control and BLE transport

Main hardware-side components expected by the firmware:

- `ESP32` development board
- up to `5` servo outputs
- up to `5` flex sensor inputs
- up to `5` single-channel EMG inputs
- `1` FSR input for force estimation
- `1` vibro motor output for haptic feedback

---

### Running the project

You can build and flash the firmware with PlatformIO.

1. Install PlatformIO CLI or use the PlatformIO VS Code extension.
2. Connect an ESP32 board.
3. Build the project: `pio run`
4. Upload the firmware: `pio run -t upload`
5. Open the serial monitor: `pio device monitor -b 115200`

Notes:

- the default BLE device name is `ESP32-Flex6`
- runtime settings can be changed from the mobile app and are stored in NVS

---

### Current limitations

- BLE payloads use a chunked transport with `[BEGIN] ... [END]` framing
- only the generated single-channel binary realtime EMG model is integrated right now
- hardware behavior depends on the configured pins and calibrated sensor values

---

## Русский

### Обзор

Это прошивка для `ESP32`, предназначенная для контроллера протеза пальца. Она считывает flex- и EMG-входы, управляет сервоприводами, оценивает силу контакта по FSR-датчику и обменивается настройками и телеметрией с мобильным приложением по BLE.

Прошивка поддерживает до 5 пар датчик-сервопривод (т. е. до 5 пальцев). Каждая пара настраивается независимо и может использовать либо flex-сенсор, либо одноканальный EMG-вход как источник движения.

---

### Документация

- Лендинг: [prothesis.ru](https://prothesis.ru)
- Гайд по протезу: [Открыть гайд](https://drive.google.com/file/d/1TqZTdqmVpPDhjSL2WnfT9wohUkeKcAlp/view?usp=sharing)
- Гайд по приложению: [Открыть гайд](https://drive.google.com/file/d/1A2usxykovqEe099k2ItJ9acGboUhyZ13/view?usp=sharing)
- Гайд по backend: [Открыть гайд](https://drive.google.com/file/d/1RrIl7wzfEjwOqdwEEhMq3LTtUpla6rIw/view?usp=sharing)
- 3D-модели: [Открыть STL и STEP](https://drive.google.com/drive/folders/1bwpGPP83HMJFmotW1bT_C9SNT2XIXNeM?usp=sharing)
- Репозиторий приложения: [graevsky/6thFinger-App](https://github.com/graevsky/6thFinger-App)
- Репозиторий backend: [graevsky/6thFinger-Backend](https://github.com/graevsky/6thFinger-Backend)

---

### Фичи

- выбор источника входа для каждой пары: `Flex` или `EMG`
- обмен конфигурацией по BLE через chunked JSON transport
- постоянное хранение настроек в `NVS` на `ESP32`
- опциональная BLE-аутентификация по 4-значному PIN-коду
- live-управление сервоприводами из мобильного приложения
- потоковая телеметрия по датчикам, сервоприводам, EMG-состоянию и vibro-выходу
- обратная связь
- EMG-модель на 1 канал

---

### Технологии

- `ESP32` и `Arduino framework`
- `PlatformIO` (`env:esp32dev`)
- `NimBLE-Arduino` для BLE GATT-сервисов и характеристик
- `ESP32Servo` для управления сервоприводами
- `ArduinoJson` для payload настроек, ACK и телеметрии
- `Preferences` (`NVS`) для хранения конфигурации
- `FreeRTOS` mutex для сериализации BLE-отправки
- `LEDC` PWM для vibro-канала
- EMG-модель и хелперы в `include/`

---

### Архитектура

Прошивка разделена на несколько слоев:

- `src/main.cpp` - точка входа
- `src/ble/` - настройка BLE GATT, конфигурационный протокол, ACK-ответы, транспорт телеметрии, сохранение настроек
- `src/control/` - опрос датчиков, фильтрация, перевод ADC в сопротивление, управление сервоприводами, vibro feedback, runtime-планирование
- `src/emg/` - сэмплирование EMG, обертка над realtime-классификатором, детекция snapshot-событий, state machine для bend/unfold
- `src/config/` - схема настроек и общая модель телеметрии
- `include/emg_features.h` - общие realtime DSP/feature extraction helper для моделей
- `include/realtime_1ch_binary_model.h` - 1-канальная бинарная EMG-модель и функции инференса
- `platformio.ini` - зависимости и настройки

Краткий runtime flow:

1. `main.cpp` поднимает serial, BLE и control-подсистему.
2. `BleApp` восстанавливает настройки из `NVS` и поднимает BLE-сервис.
3. `Control` планирует опрос flex, FSR и EMG и вычисляет целевые значения для актуаторов.
4. `EmgEngine` превращает realtime EMG-окна в snapshot-события и действия bend/unfold.
5. Обновляются выходы сервоприводов и vibro, а при включенной телеметрии данные отправляются обратно по BLE.

---

### Компоненты

Основные firmware-компоненты:

- BLE application layer с каналами конфигурации, ACK, телеметрии и live servo control
- versioned settings model для FSR, vibro, flex, servo, источника входа пары, EMG и PIN-аутентификации
- control subsystem со сглаживанием, защитой от выбросов, стабилизацией ADC и планированием актуаторов
- EMG runtime с бинарным классификатором и state machine
- общая telemetry snapshot model между control-логикой и BLE-транспортом

Основные аппаратные компоненты, на которые рассчитана прошивка:

- плата `ESP32`
- до `5` выходов на сервоприводы
- до `5` входов flex-сенсоров
- до `5` одноканальных EMG-входов
- `1` вход FSR для оценки силы
- `1` выход на vibro-мотор для тактильной обратной связи

---

### Запуск проекта

Собрать и прошить проект можно через PlatformIO.

1. Установите PlatformIO CLI или используйте расширение PlatformIO для VS Code.
2. Подключите плату `ESP32`, совместимую с `env:esp32dev`.
3. Соберите проект: `pio run`
4. Загрузите прошивку: `pio run -t upload`
5. Откройте serial monitor: `pio device monitor -b 115200`

Примечания:

- BLE-имя устройства по умолчанию: `ESP32-Flex6`
- runtime-настройки можно менять из мобильного приложения, они сохраняются в `NVS`

---

### Текущие ограничения

- BLE-payload используют chunked transport с framing вида `[BEGIN] ... [END]`
- сейчас интегрирована только сгенерированная одноканальная бинарная realtime EMG-модель
- поведение в реальной среде зависит от корректной настройки пинов и датчиков
