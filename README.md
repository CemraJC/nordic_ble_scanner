# Nordic BLE Scanner
> Windows only, for now

This is a simple utility for scanning local Bluetooth-Low-Energy (BLE) devices with wireless chips from Nordic Semiconductor.

The app was built using the `pc-ble-driver` package provided by Nordic to serialise commands for their proprietary bus protocol over a standard virtual COM port. A stripped-down version of the driver is provided with the project (in directory `nrf-ble-driver-4.1.1-win_x86_32`).

The `pc-ble-driver` library was built for Windows machines, but theoretically can be built on other platforms as well. This is left as an exercise to the reader.

### Getting Started

Three steps:
* Download the [latest release](https://github.com/CemraJC/nordic_ble_scanner/releases/latest)
* Program your Nordic BLE-enabled device with the Nordic Connectivity 4.1.1 (using V3 API)
	* A `.hex` file is provided with the release - you can program the device with that
	* Easiest to use [nRF Connect for Desktop](https://www.nordicsemi.com/Software-and-tools/Development-Tools/nRF-Connect-for-desktop) to program the chip.
* Run `ble_scanner.exe` in your favourite command line

### Using the Tool

Here is the output from the tool's help option:

```
Usage:
  BLE Scanner [OPTION...]

  -h, --help        Show this help message
  -v, --version     Output the version number
  -d, --debug       Enable verbose debug output
  -p, --port arg    Serial port to use (e.g. COM1). (default: COM1)
  -s, --server arg  Start the scanner as a local server on the given port.
                    E.g. -s 3000 will listen on 127.0.0.1:3000. (default: 0)

Server API commands:
  START                 - Start sending discovered BLE advertisements in real-time
  STOP                  - Stop sending
  FILTER_NAME <str>     - Filter device names. Names filtered by if they contain <str>.
                          To stop filtering, set <str> to empty (send 'FILTER_NAME ')
  FILTER_MAC <str>      - Filter device MAC. Filtered by if they start with <str>.
                          To stop filtering, set <str> to empty (send 'FILTER_MAC ')
  QUIT                  - Force the server to exit

Device names are provided in the format <MAC>,<name> where <MAC> is the device address
and <name> is either '<unknown>' or the device complete local name.
```

### Supported Chips

This project was developed on the nRF52840 Dongle.

List of devices supported by the `pc-ble-driver` library (all should work with this tool):

| PCA      | Official name                | Article number | Notes    |
-----------|------------------------------|----------------|----------|
| PCA10028 | nRF51 DEVELOPMENT KIT        | nRF6824        |          |
| PCA10031 | nRF51 DONGLE                 | nRF6825        |          |
| PCA10040 | nRF52 DEVELOPMENT KIT        | nRF6827        |          |
| PCA10056 | nRF52840 ( Development Kit ) | nRF6828        |          |
| PCA10059 | nRF52840 ( Dongle )          | nRF6829        | Only with Nordic USB CDC support |

## Sample Outputs

Note that `*action*` lines are not output on the console, just notes of actions I made while running the program.

**Local Mode**
```
> ble_scanner.exe -p COM4
B8:74:DF:FB:06:15,<unknown>
6F:01:DB:7D:51:45,<unknown>
5F:6A:43:91:F2:64,<unknown>
...
E5:3F:0A:44:45:4E,<unknown>
6F:01:DB:7D:51:45,<unknown>
*Pressed Ctrl + C*
> _
```

**Local Mode with Debug Output**
```
> ble_scanner.exe -d -p COM4
Log: Verbose mode: 1
Log: Server mode: 0
Log: Serial port used: COM4
Log: Baud rate used: 1000000
Info: Successfully opened COM4. Baud rate: 1000000. Flow control: none. Parity: none.
Status: 6, message: Target Reset performed
Status: 7, message: Connection active
B8:74:DF:FB:06:15,<unknown>
6F:01:DB:7D:51:45,<unknown>
97:CB:37:9C:D2:14,<unknown>
...
C8:D6:D5:A5:11:28,<unknown>
73:C3:36:FE:54:1C,<unknown>
38:8B:DC:1E:27:C1,fm50:8C:B1:61:E5:0B
B8:74:DF:FB:06:15,<unknown>
*Pressed Ctrl + C*
Log: Got input: ' '
> _
```

**Server Mode with Debug Output**
```
> ble_scanner.exe -d -s 4000 -p COM4
Log: Verbose mode: 1
Log: Server mode: 1
Log: Serial port used: COM4
Log: Baud rate used: 1000000
Info: Successfully opened COM4. Baud rate: 1000000. Flow control: none. Parity: none.
Status: 6, message: Target Reset performed
Status: 7, message: Connection active
Listening for connections on port 4000.

*Opened netcat, made connection*

Connection request from 127.0.0.1:18038

*Sent 'FILTER_NAME fm' from netcat*

Log: CLIENT: FILTER_NAME fm
Log: SERVER: Servicing FILTER_NAME command
Log: SERVER: Set name filter to 'fm'
Log: CLIENT: START

*netcat is outputting lines like '38:8B:DC:1E:27:C1,fm50:8C:B1:61:E5:0B'*

Log: SERVER: Servicing START command
Log: CLIENT: STOP
Log: SERVER: Servicing STOP command
Log: CLIENT: QUIT
Log: SERVER: Servicing QUIT command
Log: Client disconnected.
Error: A blocking operation was interrupted by a call to WSACancelBlockingCall.
> _
```

In this server mode, the final "Error" log is expected because the server is in debug mode - the client issued a QUIT command, which forces the server to exit.

## Building from Scratch

This project was built with Visual Studio 2017. 

To build your own binary, open the `ble_scanner.vcxproj` file in Visual Studio, and set the output configuration to "Release x86". Then hit build, and everything should (hopefully) work!