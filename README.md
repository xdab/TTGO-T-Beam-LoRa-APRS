# LoRa tracker with KISS TNC capability

Tracker can be used on its own. 
You can also connect it via blueetooth with APRSdroid.
After connection with APRX based DIGI it can be used as KISS-TNC

![diagram](https://github.com/SQ9MDD/TTGO-T-Beam-LoRa-APRS/blob/master/img/digi-schemat.png)

## Contributors
* Initial work: OE1ACM, OE3CJB
* Redesigned: SQ9MDD
* KISS TNC Over Seriall or Bluetooth: SQ5RWU
* Lora32 board support DJ1AN

## Supported boards
* TTGO T-beam v.0.7
* TTGO T-beam v.1.0 
* Lora32 board

## User key functions:
### short press (OLED Enabled):
* if OLED is OFF a short press will wake it up and show current data
* if OLED is ON with GPS-FIX immediate sending of a frame with the position from the GPS
* if OLED is ON without GPS-FIX, immediate sending of the frame with the position saved in the configuration

### short press (OLED Disabled):
* with a valid GPS-FIX immediate sending of a frame with the position from the GPS
* without valid GPS-FIX, immediate sending of the frame with the position saved in the configuration

### long press while booting 
* reset to factory default

### long press: toggle on or off GPS power supply 
* if the "FIXED_BEACON_EN" option is enabled in the configuration, a beacon will be sent every set time interval

## Oled screens information
* ((TX)) - position frame sent automatically
* ((RX)) - informs about the received frame 
* ((GPSOFF)) - GPS power-off information
* ((GPS ON)) - GPS power-on information
* ((MAN TX)) - information about sending a manually initialized GPS position frame
* ((FIX TX)) - information about the forced manual sending of a frame with the position saved in the configuration when GPS is off or no fix
* ((AUT TX)) - information about sending automatic positioning frame when GPS is turned off
* ((KISSTX)) - information about sending the frame sent by KISS
* ((WEB TX)) - sending frame as requested via HTTP

## How to binary first flash readme... (thanx SP6VWX)
* Download the appropriate binary file for your board from: https://github.com/SQ9MDD/TTGO-T-Beam-LoRa-APRS/releases
* Download current version of the esphome-flasher tool: https://github.com/esphome/esphome-flasher/releases
* Connect your board to your USB port and open ESPHome Flasher.
* If your board is not showing under Serial Port then you likely need to install the drivers for the CP210X serial chip. In Windows you can check by searching “Device Manager” and ensuring the device is shown under “Ports”.
* In ESPHome Flasher, refresh the serial ports and select your board's serial port.
* Browse to the downloaded firmware and select the correct firmware based on the board type.
* Select/Press Flash ESP.
* Once complete, “Done! Flashing is complete!” will be shown.
* any subsequent updates can be done via WWW

## Installing from source (only for advanced users)
* Clone Git repo to any folder of your PC
* Download and install "Visual Studio Code"
* Open "Visual Studio Code" and add "PlatformIO" plugin
* Open "PlatformIO" and click on "Import Arduino Project"
* In the top part of the window choose youy board then browse to cloned repo and click "import"
* In the left column click on the ANT-shaped icon, choose your board and click on "Upload". COM port should be detected automatically Wait for procedure to finish and keep reading

## Configuring parameters
Wait for the board to reboot, connect to "N0CALL AP" WiFi network, password is: xxxxxxxxxx (10 times "x") and point your browser to "192.168.4.1"

### WiFi Settings
you can scan for local SSID or manually type in name and password

###  APRS Settings
These are main APRS settings such as callsign, SSID and symbol (refer to: http://www.aprs.org/symbols.html). Please remember to turn ON GPS in order to use it as a tracker

### Device Settings
These are main device settings, hover the mouse on the checkboxes and explainations will appear Note: as of 09/2021 "Auto Power OFF" is enabled only with TBEAM 1.0, it will turn board off after unplugging USB

### Received
Here is the list of recently received stations with some details

### Actions
Some shortcuts to useful functions such as manually send beacon

### Update
This box allows firmware to be uploaded via OTA

## Default WiFi Password:
* Default WiFi Password is: xxxxxxxxxx
* that is, 10 times x
