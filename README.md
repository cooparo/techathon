# Channel Sounding Demo — Initiator / Reflector (nRF54L15)

Informal technical README for the Wi‑Fi‑free distance demo we built during the techathon.

Overview
This repo contains firmware for two Nordic nRF54L15 boards demonstrating Bluetooth 6.0 Channel Sounding (CS) based distance estimation. One board is the Initiator (actively scans, connects and triggers CS procedures). The other board is the Reflector (advertises and responds / reflects CS tones). The code is written in C using the Zephyr / nRF Connect SDK stack and is built/flashed with west and the Zephyr SDK.

What we built (behaviour summary)
- Initiator
  - Continuously scans for reflectors advertising the chosen name.
  - When a reflector is found the Initiator turns LED0 blinking ON to signal discovery.
  - The Initiator waits for a local button press to approve the connection.
  - On button press it connects, turns LED0 OFF, negotiates Channel Sounding and starts CS procedures.
  - While CS runs, the firmware computes distance using two methods (phase-slope and RTT). If estimated distance < 0.5 m an event is triggered and logged.

- Reflector
  - Starts advertising immediately on boot (name "Beacon" in our code).
  - Accepts connections and participates in Channel Sounding.
  - When connected LED0 is solid ON; when disconnected it returns to advertising and LED0 OFF.
  - Reflector writes its step data to the Initiator via GATT during each CS procedure.

Hardware
- 2 × Nordic Semiconductor nRF54L15 development boards (one flashed with initiator firmware, one with reflector firmware).

Software stack / tools used
- Language: C (Zephyr applications)
- nRF Connect SDK (NCS, version `3.1.1`) + Zephyr (version `4.1.99`)
- west (Zephyr meta-tool, version `1.4.0`) to fetch, build and flash
- Zephyr SDK (toolchain, version `0.17`)
- python (version 3.12)

Important files (high level)
- `initiator/connected_cs_initiator.c`
  - Main Initiator logic: scanning, found-device flow, button defer flow, connection establish, CS config creation and procedure handling.
  - Key runtime semaphores: sem_button_pressed, sem_connected, sem_procedure_done, sem_data_received.
  - Uses `estimate_distance(...)` defined in `distance_estimation.c` to compute phase-based and RTT-based distances.
- `initiator/distance_estimation.c`
  - Implements two distance estimation algorithms:
    - phase-slope method (estimates range from phase vs frequency slope)
    - RTT (round-trip timing) method derived from CS timing fields
  - Parsing and processing of CS "step" data and helper math (linear regression, unwrapping).
- `reflector/` (mirror for the server side)
  - Advertising on boot, accepts connection, toggles LED0 on connect/disconnect, performs CS role as Reflector.
  - Discovers step data characteristic and writes step data to the Initiator.

Build & flash — typical workflow
(Assumes west is initialized and the nRF Connect SDK + Zephyr SDK are installed on your machine.)

1. From repo root, build and flash the Initiator:

```bash
cd initiator
west build -b nrf54l15dk/nrf54l15/cpuapp -d build .
west flash -d build
```
2. build and flash the Reflector:

```bash
cd reflector
west build -b nrf54l15dk/nrf54l15/cpuapp -d build .
west flash -d build
```