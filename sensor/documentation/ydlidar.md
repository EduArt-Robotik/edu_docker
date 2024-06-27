# YD Lidar

## Create UDEV Rules

Due to the USB chip (serial converter) used, there may be ambiguous assignments. The chip is mounted as a ttyUSB device under Linux. If several devices are connected, the assignment (ttyUSB0, ttyUSB1, ...) depends more or less on randomness. To prevent this, a UDEV rule should be created so that a unique simulink is generated for each lidar. This makes it easy to keep several lidar sensors separate.

Unfortunately, the serial number of the chip used is always the same. This means that the lidar cannot be assigned correctly. Unfortunately, we cannot say whether this is the case with all YD lidars or only with the ones we have. However, it seems to be the case that they are all supplied with the same serial number, namely 0001.

In order to still be able to differentiate between the lidars, we use the absolute USB bus address here. A way to determine all the necessary properties is described below. Finally, you should be able to create a UDEV rule for each YDLidar you want to use.

### Getting Absolute USB Bus Address

Make sure that the lidar not connected. Connect the Lidar via USB and execute following command to get the correct device name:

```bash
# shows last kernel messages entries
sudo dmesg | tail
```

Following example shows a possible print out of the last command:

```bash
[17384.923911] usb 3-3.2: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[17384.923913] usb 3-3.2: Product: CP2102 USB to UART Bridge Controller
[17384.923915] usb 3-3.2: Manufacturer: Silicon Labs
[17384.923916] usb 3-3.2: SerialNumber: 0001
[17384.949005] usbcore: registered new interface driver usbserial_generic
[17384.949014] usbserial: USB Serial support registered for generic
[17384.950198] usbcore: registered new interface driver cp210x
[17384.950207] usbserial: USB Serial support registered for cp210x
[17384.950252] cp210x 3-3.2:1.0: cp210x converter detected
[17384.956990] usb 3-3.2: cp210x converter now attached to ttyUSB0
```

You can see that a new USB device has been connected. It is a CP2102 USB to UART converter and has been integrated as **ttyUSB0**. Now we want to determine the ID_PATH of that device. Please execute the following command to do so (adapt device name):

```bash
# shows ID_PATH to the connected ttyUSB device
udevadm info /dev/ttyUSB0 | grep ID_PATH=
```

The printed line shows the absolute device address. The line could look like:

```bash
# print out of grepped udevadm info command
E: ID_PATH=pci-0000:05:00.3-usb-0:3.2:1.0
```

Now we will use it to create a udev rule for it. Create a new file and open it by:

```bash
# creates and opens new udev rules file
sudo nano /etc/udev/rules.d/ydlidar.rules
```

Put following line above into it and adapt the **ENV{ID_PATH}** with it. Please be aware of that your line will look different.


file /etc/udev/rules.d/ydlidar.rules
```bash
# udev rule for YDLidar sensors
SUBSYSTEM=="tty", ENV{ID_PATH}=="pci-0000:05:00.3-usb-0:3.2:1.0", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="ydlidar"
```

After the UDEV rules was created it needs to be loaded. Please execute following command for reloading it:

```bash
# reloads all udev rules --> new onces will be active after
sudo udevadm control --reload
```

Now please disconnect and connect the Lidar device again. The created UDEV rule should create a symlink for the connected Lidar. Please proof it by checking the folder **/dev** by:

```bash
ls /dev -l
```

The symlink **rplidar** should be listed and pointing to a **ttyUSB** device:

```bash
lrwxrwxrwx   1 root    root             7 Jun 27 14:20 ydlidar -> ttyUSB0
```
