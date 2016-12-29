# nrf51_mqtt_pir
test program that sends an BLE mqtt message when a pir sensor is triggered (pin 18)
the topic is
kitchen/sensor1
the message is either 1 or 0. You can change it to a string but I am going to graph the 
sensor messages in OpenHAB, so 1 and 0 are easier to handle.
Compiles in eclipse for a nordic semiconductor nrf51822
it is based on the sdk nrf5_iot_sdk_3288530 and uses the make file. I have moved directories 
and other files around to make it easier to program in eclipse.

It is a test program so has a lot of things to fix but it essentially works
It will startup and wait for a 6lowpan connection. I am using a RPI as a 6lowpan router and 
6tunnel to send the message to an ipv4 mosquitto broker.
The PI can then connect to the nrf51 using a command like

echo "connect AA:BB:CC:DD:EE:FF 1" > /sys/kernel/debug/bluetooth/6lowpan_control
where  AA:BB:CC:DD:EE:FF is the mac address of the nrf51
you can find the mac address by doing an hcitool lescan.

root@raspberrypi:~# hcitool lescan
LE Scan ...
AA:BB:CC:DD:EE:FF MQTTSensor1
once you have connected, you can confirm by typing

root@raspberrypi:~# hcitool con
Connections:
	< Unknown AA:BB:CC:DD:EE:FF handle 72 state 1 lm MASTER 

In turn this message is also going to an OpenHAB service on the same computer using the mqtt addon.

The uart is turned on so you get debug messages on uart 1 or on your usb if you are using a development board.

I an using stlinkv2 to flash the device. The commands are
In terminal 1

sudo openocd -f /usr/share/openocd/scripts/interface/stlink-v2.cfg -f /usr/share/openocd/scripts/target/nrf51.cfg

in terminal 2


telnet localhost 4444
reset_config  srst_nogate
set WORKAREASIZE 0
adapter_nsrst_delay 100
adapter_nsrst_assert_width 100
init 
reset  halt
nrf51 mass_erase 
sleep 500
flash write_image /pathto/nrf51/components/softdevice/s1xx_iot/s1xx-iot-prototype2_softdevice.hex 0
flash write_image /pathto/nrf51_mqtt_pir/_build/nrf51422_xxac_s1xx_iot.hex 0  
reset run
shutdown







