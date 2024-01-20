# ESPelomon

### overview
Recreate Imran Haque's lovely pelomon project for a cheaper, more accessible microcontroller that handles more of the complicated stuff that I don't know how to do

### contributing
everything is being done in vscode with esp-idf because that seems like the most straightforward way to do this, using the framework developed for these microcontrollers, rather than replicating Imran Haque's Arduino-based framework that's intended to interface with an nRF52 on an Adafruit 32u4 BLE feather

### rough plan
 - translate serial from peloton into a BLE device for consumption by a fenix 6
    - consume serial
        - get the calibration data of the bike from bootup
            -  store it somehow with the nonvolatile storage module
                https://github.com/espressif/esp-idf/tree/master/examples/storage/nvs_rw_value
        - capture data from bike
            - create two interrupt-driven uart receive tasks
                https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/uart.html
                https://github.com/espressif/esp-idf/tree/master/examples/peripherals/uart/uart_echo
                https://github.com/espressif/esp-idf/tree/master/examples/peripherals/uart/uart_async_rxtxtasks
            - store that in a way the ble server can get at it (?????)
    - output bluetooth
        https://www.espressif.com/sites/default/files/documentation/esp32_bluetooth_architecture_en.pdf
        - configure the radio
            - set up advertising of the GATT
                https://github.com/espressif/esp-idf/blob/master/examples/bluetooth/bluedroid/ble/gatt_server_service_table/tutorial/Gatt_Server_Service_Table_Example_Walkthrough.md
            - create the matching GATT profile that is known to work based on the DFC's open source information and Imran's GATT config files
                https://github.com/intelligenate/dfc/blob/main/src/arduino/IrieTron_3030.ino
        - pack it up and send it over
            - parse the data
            - wait for a request I think?
