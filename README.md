> NOTE: This is a community branch, and is not supported by OpenPPG!!!

> NOTE: It may not be stable and is not recommended for flying.

> See official OpenPPG stable releases [here](https://github.com/openppg/eppg-controller/releases)

# OpenPPG Controller

![Build](https://github.com/thandal/eppg-controller/actions/workflows/config.yml/badge.svg)

Arduino-based logic for OpenPPG SP140 RP2040 Throttle Controller.

Download releases [here](https://github.com/thandal/eppg-controller/releases)

## Build and flash firmware using PlatformIO

Suitable for *Linux*

### Setup

1. Follow the instructions here for using with VSCode https://platformio.org/install/ide?install=vscode
2. Extract the downloaded code from the repo [here](https://github.com/thandal/eppg-controller/archive/master.zip) (or `git clone` it)
3. Open the folder using the PlatformIO "open project" option inside of VSCode.

### Flash the OpenPPG Code

Option 1: Click the "PlatformIO Build" button inside of VSCode or enter `platformio run --target upload` in the command line. PlatformIO will automatically download libraries the first time it runs.

Option 2: Build the PlaformIO project. Then plug in the controller via a USB cable and put it into update mode, then copy the .pio/build/OpenPPG-CRP2040-SP140/firmware.uf2 to the usb drive that appears. The controller will reset and run the new firmware.

## Config tool

> NOTE: Web-based config is not currently supported for this branch!

The open source web based config tool for updating certain settings over USB (without needing to flash firmware) can be found at https://config.openppg.com.

## Help improve these docs

Pull requests are welcome for these instructions and code changes.
