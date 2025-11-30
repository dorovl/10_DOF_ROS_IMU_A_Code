### Depends on installation
    pip install bleak numpy
### How to use
Line 14 in the code par_device_addr="70:53:B2:02:20:02" 

The MAC address of the sensor needs to be replaced with the mac address of your sensor

Connect imu:

    python3 imu_ble.py 

Note that after the example connects to imu,directly closing the running window may cause the windows system to not automatically release the connection resources to the device
 (observe the imu indicator light and you will find that it flashes twice every now and then to indicate that it is connected but not disconnected). 

If this happens, run it again. In the example, imu cannot be connected.

You can turn off the Bluetooth switch in the Windows system Bluetooth management interface and then turn it back on to disconnect the device.