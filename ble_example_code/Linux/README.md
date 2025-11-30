### Depends on installation
    bluez
    libbluetooth-dev
    gatt
The corresponding instructions are as follows. 

The instructions may be different in different systems.

    sudo apt-get install bluez \ libbluetooth-dev
    pip install gatt
### How to use
Enable Bluetooth: Make sure Bluetooth is enabled

In RDK X3, you need to manually activate Bluetooth every time you turn on the computer.

    sudo /usr/bin/startbt6212.sh

Connect imu:

    python3 imu_ble.py 70:53:B2:02:20:02