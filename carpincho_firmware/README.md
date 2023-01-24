# carpincho_firmware

This folder contains the firmware project for the low-level code that runs on the Arduino Uno (ATmega328) microcontroller.

## Getting Started

### Prerequisites

The firmware project is based on [PlatformIO IDE for VSCode](https://docs.platformio.org/en/latest/integration/ide/vscode.html). See [here](https://docs.platformio.org/en/latest/integration/ide/vscode.html#installation) for installation steps.

In case you are using Ubuntu/Debian, you will need to install the PlatformIO udev rules, find the required steps to do so [here](https://docs.platformio.org/en/stable/core/installation/udev-rules.html).

### Project Structure

```
    .
    ├── platformio.ini          # Project configuration file
    ├── include                 # Application header files
    ├── lib                     # Libraries
    ├── src                     # Application source code
    └── test                    # Unit testing directory
        ├── test_desktop        # Desktop (Linux) unit tests (GoogleTest used as framework)
        └── test_embedded       # Embedded (Arduino Uno) unit tests (Unity used as framework)
```

### Building

To build the application execute the following steps on the [PlatformIO Toolbar](https://docs.platformio.org/en/latest/integration/ide/vscode.html#platformio-toolbar):
 * Select `uno` as the project environment.
 * Click on the `Build` button.

### Installing & Running

To install and run the application execute the following steps on the [PlatformIO Toolbar](https://docs.platformio.org/en/latest/integration/ide/vscode.html#platformio-toolbar):
 * Select `uno` as the project environment.
 * Connect the Arduino Uno device.
 * Click on the `Upload` button.

You should see that the built-in led starts to blink.

### Running tests

This project supports unit testing, both on desktop and embedded devices. The following unit testing frameworks are used:
 * Desktop tests: [GoogleTest](https://google.github.io/googletest/).
 * Embedded tests: [Unity](http://www.throwtheswitch.org/unity/).

To build and run the desktop tests execute the following steps on the [PlatformIO Toolbar](https://docs.platformio.org/en/latest/integration/ide/vscode.html#platformio-toolbar):
 * Select `native` as the project environment.
 * Click on the `Test` button.

To build and run the embedded tests execute the following steps on the [PlatformIO Toolbar](https://docs.platformio.org/en/latest/integration/ide/vscode.html#platformio-toolbar):
 * Select `uno` as the project environment.
 * Connect the Arduino Uno device.
 * Click on the `Test` button.

## Wiring Diagram

TODO(jballoffet): Complete this section.
