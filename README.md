## Custom Firmware Changes In This Repo

This repository contains custom changes on top of upstream MeshCore firmware. The most important customizations currently included are:

### T1000-E repeater: GNSS bring-up and first-fix mesh message

This fork adds a **T1000-E–specific GNSS path** for the `simple_repeater` example, built with PlatformIO environment **`t1000e_repeater`** (Seeed T1000-E, LR1110, onboard Airoha AG3335 on `Serial1`). Upstream-style NMEA parsing still runs through `MicroNMEALocationProvider`; the custom pieces are power/init, UX, and a one-shot mesh announcement on first fix.

**Build and flash (from the repo root, where `platformio.ini` lives)**

- The **16-byte group PSK** for the first-fix mesh message is **never in source**. Pass **exactly 32 hex digits** at build time:
  - Shell: `export SECRET_GNSS_CHANNEL_KEY_HEX=<32_lowercase_hex_digits>`, or
  - Gitignored one-line file in the project root: `.secret_gnss_channel_key`
- The build fails if the key is missing or not 32 hex characters (see `variants/t1000-e/t1000e_repeater_gnss_channel_key.py`).

Example:

```bash
SECRET_GNSS_CHANNEL_KEY_HEX=<your_32_hex_digits> pio run -e t1000e_repeater -t upload
```

There is **no separate channel-name build flag**. The hashtag name is **derived from the PSK**: `#` plus the first **14 bytes** of `SHA256(PSK)` rendered as **lowercase hex** (28 characters). Any listener with the same PSK can compute the same channel name and decrypt.

**Initialization and bring-up**

- `variants/t1000-e/t1000e_gps_bringup.cpp` runs an **AG3335 bring-up sequence** (command order and delays aligned with the terminaattori reference): reset, baud, wakeup, and periodic health checks. Optional step logs can be enabled from the bring-up API.
- The UART is **not** used to stream full NMEA to USB. When GNSS is started from the **user button** path, bring-up can print `[GPS]` / `[GNSS]` step labels and a one-time **`first NMEA received`** after the first plausible NMEA/PAIR/PMTK line; the automatic `start_gps()` path runs the same sequence **without** that progress logging.

**Runtime behavior (repeater firmware)**

- **User button** (short press): toggles GNSS power/init. **Buzzer**: short rising glissando when GNSS is turned on, falling when turned off (see `examples/simple_repeater/main.cpp`).
- **Boot**: short low–high buzzer chirp in `MyMesh::begin` (T1000-E with buzzer pins defined).
- **GNSS mesh messages** (derived secret `#…` channel, see below): a buzzer pattern plays on **first** valid fix; the same payload is sent as encrypted **group text** (flood) when a fix is valid and **either** about **5 minutes** have passed since the last successful send **or** the receiver has moved **more than 500 m** (great-circle) from the position last sent. Failed sends retry at most once per minute until the next normal trigger.
- **Session auto-off:** **two hours** after GNSS is enabled (boot `start_gps()` or button turn-on), firmware **sleeps the GNSS module**, **clears** pending mesh GNSS sends, sets **`gps_enabled` saved prefs to 0**, and prints `[GNSS] Auto power-off after 2h session` on USB serial. Turn GNSS on again with a **short button press** (or CLI `set gps 1` / `save` as applicable).

**Secret GNSS group channel (companion / another radio)**

- **PSK:** Same **32 hex characters** you passed as `SECRET_GNSS_CHANNEL_KEY_HEX` when building the repeater. The receiver must use that **16-byte** secret; decryption fails otherwise.
- **Name:** `#` + lowercase hex of `SHA256(PSK)[0..13]` (14 bytes → 28 hex chars). Join or create that hashtag in the client together with the PSK.
- **GNSS** must get a **first valid fix** (sky view, antenna) before mesh GNSS messages are sent. **New installs** default to GPS on; **saved prefs** decide whether GNSS starts after reboot (after a 2h auto-off, prefs stay off until you enable again).

**Source layout (GNSS-related)**

| Area | Location |
|------|----------|
| AG3335 bring-up | `variants/t1000-e/t1000e_gps_bringup.{h,cpp}` |
| Sensors, fix formatting, first-fix chirp / mesh pending flag | `variants/t1000-e/target.{h,cpp}` |
| Secret channel init, first-fix send, boot buzzer | `examples/simple_repeater/MyMesh.{h,cpp}` |
| Button / GNSS toggle tones | `examples/simple_repeater/main.cpp` |
| Build-time GNSS group PSK | `variants/t1000-e/t1000e_repeater_gnss_channel_key.py`, `variants/t1000-e/t1000e_secret_gnss_channel.h` |

`T1000-E` builds that use `examples/simple_repeater/main.cpp` are expected to use **`env:t1000e_repeater`** (`T1000E_REPEATER_BUILD`); otherwise compilation stops with an explicit error.

### How `#telemetry` works in this fork

- Repeater firmware auto-creates (or uses) a region named `#telemetry`.
- Telemetry text is sent as encrypted group datagrams on that region using a shared channel secret configured in firmware.
- The repeater periodically gathers sensor data + board data and publishes a human-readable message to `#telemetry`.
- Message format is variant-dependent:
  - `T1000-E`: `T=...`, `V=...V`, `LU=...`
  - Other repeaters: standard repeater telemetry text fields.

- `T1000-E` repeater behavior:
  - Device status LED heartbeat: very short blink every 5 seconds.
  - Short speaker "tick" on each received radio packet.
  - Variant-specific `#telemetry` text payload format including:
    - temperature (`T=...`)
    - battery voltage (`V=...V`)
    - luminosity (`LU=...`)
- Repeater telemetry text cleanup:
  - Removed the `A=...` airtime field from `simple_repeater` telemetry message format.
- BME280 detection robustness:
  - Sensor init now probes both common I2C addresses (`0x76` and `0x77`) instead of only one.
- XIAO nRF52 I2C mapping adjustment:
  - Swapped `PIN_WIRE_SCL` / `PIN_WIRE_SDA` mapping in the `xiao_nrf52` PlatformIO target config.

These changes are intended for the custom hardware/firmware workflow in this fork and may differ from the official upstream behavior.

## About MeshCore

MeshCore is a lightweight, portable C++ library that enables multi-hop packet routing for embedded projects using LoRa and other packet radios. It is designed for developers who want to create resilient, decentralized communication networks that work without the internet.

## 🔍 What is MeshCore?

MeshCore now supports a range of LoRa devices, allowing for easy flashing without the need to compile firmware manually. Users can flash a pre-built binary using tools like Adafruit ESPTool and interact with the network through a serial console.
MeshCore provides the ability to create wireless mesh networks, similar to Meshtastic and Reticulum but with a focus on lightweight multi-hop packet routing for embedded projects. Unlike Meshtastic, which is tailored for casual LoRa communication, or Reticulum, which offers advanced networking, MeshCore balances simplicity with scalability, making it ideal for custom embedded solutions., where devices (nodes) can communicate over long distances by relaying messages through intermediate nodes. This is especially useful in off-grid, emergency, or tactical situations where traditional communication infrastructure is unavailable.

## ⚡ Key Features

* Multi-Hop Packet Routing
  * Devices can forward messages across multiple nodes, extending range beyond a single radio's reach.
  * Supports up to a configurable number of hops to balance network efficiency and prevent excessive traffic.
  * Nodes use fixed roles where "Companion" nodes are not repeating messages at all to prevent adverse routing paths from being used.
* Supports LoRa Radios – Works with Heltec, RAK Wireless, and other LoRa-based hardware.
* Decentralized & Resilient – No central server or internet required; the network is self-healing.
* Low Power Consumption – Ideal for battery-powered or solar-powered devices.
* Simple to Deploy – Pre-built example applications make it easy to get started.

## 🎯 What Can You Use MeshCore For?

* Off-Grid Communication: Stay connected even in remote areas.
* Emergency Response & Disaster Recovery: Set up instant networks where infrastructure is down.
* Outdoor Activities: Hiking, camping, and adventure racing communication.
* Tactical & Security Applications: Military, law enforcement, and private security use cases.
* IoT & Sensor Networks: Collect data from remote sensors and relay it back to a central location.

## 🚀 How to Get Started

- Watch the [MeshCore Intro Video](https://www.youtube.com/watch?v=t1qne8uJBAc) by Andy Kirby.
- Watch the [MeshCore Technical Presentation](https://www.youtube.com/watch?v=OwmkVkZQTf4) by Liam Cottle.
- Read through our [Frequently Asked Questions](./docs/faq.md) and [Documentation](https://docs.meshcore.io).
- Flash the MeshCore firmware on a supported device.
- Connect with a supported client.

For developers;

- Install [PlatformIO](https://docs.platformio.org) in [Visual Studio Code](https://code.visualstudio.com).
- Clone and open the MeshCore repository in Visual Studio Code.
- See the example applications you can modify and run:
  - [Companion Radio](./examples/companion_radio) - For use with an external chat app, over BLE, USB or WiFi.
  - [KISS Modem](./examples/kiss_modem) - Serial KISS protocol bridge for host applications. ([protocol docs](./docs/kiss_modem_protocol.md))
  - [Simple Repeater](./examples/simple_repeater) - Extends network coverage by relaying messages.
  - [Simple Room Server](./examples/simple_room_server) - A simple BBS server for shared Posts.
  - [Simple Secure Chat](./examples/simple_secure_chat) - Secure terminal based text communication between devices.
  - [Simple Sensor](./examples/simple_sensor) - Remote sensor node with telemetry and alerting.

The Simple Secure Chat example can be interacted with through the Serial Monitor in Visual Studio Code, or with a Serial USB Terminal on Android.

## ⚡️ MeshCore Flasher

We have prebuilt firmware ready to flash on supported devices.

- Launch https://meshcore.io/flasher
- Select a supported device
- Flash one of the firmware types:
  - Companion, Repeater or Room Server
- Once flashing is complete, you can connect with one of the MeshCore clients below.

## 📱 MeshCore Clients

**Companion Firmware**

The companion firmware can be connected to via BLE, USB or WiFi depending on the firmware type you flashed.

- Web: https://app.meshcore.nz
- Android: https://play.google.com/store/apps/details?id=com.liamcottle.meshcore.android
- iOS: https://apps.apple.com/us/app/meshcore/id6742354151?platform=iphone
- NodeJS: https://github.com/liamcottle/meshcore.js
- Python: https://github.com/fdlamotte/meshcore-cli

**Repeater and Room Server Firmware**

The repeater and room server firmwares can be setup via USB in the web config tool.

- https://config.meshcore.io

They can also be managed via LoRa in the mobile app by using the Remote Management feature.

## 🛠 Hardware Compatibility

MeshCore is designed for devices listed in the [MeshCore Flasher](https://meshcore.io/flasher)

## 📜 License

MeshCore is open-source software released under the MIT License. You are free to use, modify, and distribute it for personal and commercial projects.

## Contributing

Please submit PR's using 'dev' as the base branch!
For minor changes just submit your PR and we'll try to review it, but for anything more 'impactful' please open an Issue first and start a discussion. Is better to sound out what it is you want to achieve first, and try to come to a consensus on what the best approach is, especially when it impacts the structure or architecture of this codebase.

Here are some general principals you should try to adhere to:
* Keep it simple. Please, don't think like a high-level lang programmer. Think embedded, and keep code concise, without any unnecessary layers.
* No dynamic memory allocation, except during setup/begin functions.
* Use the same brace and indenting style that's in the core source modules. (A .clang-format is prob going to be added soon, but please do NOT retroactively re-format existing code. This just creates unnecessary diffs that make finding problems harder)

Help us prioritize! Please react with thumbs-up to issues/PRs you care about most. We look at reaction counts when planning work.

## Road-Map / To-Do

There are a number of fairly major features in the pipeline, with no particular time-frames attached yet. In very rough chronological order:
- [X] Companion radio: UI redesign
- [X] Repeater + Room Server: add ACL's (like Sensor Node has)
- [X] Standardise Bridge mode for repeaters
- [ ] Repeater/Bridge: Standardise the Transport Codes for zoning/filtering
- [X] Core + Repeater: enhanced zero-hop neighbour discovery
- [ ] Core: round-trip manual path support
- [ ] Companion + Apps: support for multiple sub-meshes (and 'off-grid' client repeat mode)
- [ ] Core + Apps: support for LZW message compression
- [ ] Core: dynamic CR (Coding Rate) for weak vs strong hops
- [ ] Core: new framework for hosting multiple virtual nodes on one physical device
- [ ] V2 protocol spec: discussion and consensus around V2 packet protocol, including path hashes, new encryption specs, etc

## 📞 Get Support

- Report bugs and request features on the [GitHub Issues](https://github.com/ripplebiz/MeshCore/issues) page.
- Find additional guides and components on [my site](https://buymeacoffee.com/ripplebiz).
- Join [MeshCore Discord](https://meshcore.gg) to chat with the developers and get help from the community.
