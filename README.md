# quadro3
![CI](https://github.com/NikLeberg/quadro3/workflows/CI/badge.svg)

A thing that flies. Or should when this project is done. A drone build around an ESP32 chip from Espressif. Based on my previous (unfinished) projects quadro2 and quadro. The previous attempts were based on arduino or platformio. But as they progressed I've wanted to have more control of the underlying hardware. For now I haven't gone bare-metal and use the [esp-idf](https://github.com/espressif/esp-idf) provided by Espressif.

Generally I'm here for learning and I'm in no rush to get this thing to fly.
Most of the comments are in german. (Sorry)

## Requirements

- Hardware
    - ESP32 (Heart of the quadro)
    - BNO080 (IMU)
    - BME280 (Environment)
    - GPS
    - Mateksys 3901-L0X (Optical Flow & Lidar)
    - 4x ESC's & Motors
    - Frame
    - custom board to connect it all
- Software
    - esp-idf (v4.3)

## Installation and usage

### General
1. Install [esp-idf](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/)
2. (optional) Install [VSCode](https://code.visualstudio.com/download) - with it you can use the predefined tasks in `.vscode/tasks.json` folder
3. `idf.py flash` - build & flash quadro3 app
4. connect to WLAN-Hotspot - the quadro creates its own hotspot meant for remote control
5. surf to [quadro3](quadro3.local) - the webpage acts as remote control
6. fly!

### Tests
- `cd test && idf.py flash` - build & flash tests

## Related Projects

- [Ardupilot](https://ardupilot.org/) - They probably make the better software for drones.
- [iNav](https://github.com/iNavFlight/inav) - Based on other chips (like the STM32 family).

## Credits

Thanks to the community, espressif for making their api public and igrr who unknowingly showed me how to integrate qemu into a CI system to test my software in github actions.

## License

[MIT](LICENSE) © [NikLeberg](https://github.com/NikLeberg).