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
        - capture data from bike
            - create two interrupt-driven uart receive tasks
            - store that in a ringbuffer (like Imran) or some other way for consumption by the bluetooth broadcaster
    - output bluetooth
        - configure the radio
            - implement a task to, at regular intervals, broadcast the current data
        - advertise the bluetooth feed's existence
            - create the matching GATT profile that is known to work based on the DFC's open source information and Imran's GATT config files
        - pack it up and send it over
            - parse the data 