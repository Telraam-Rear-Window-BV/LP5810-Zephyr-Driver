# LP5810 Zephyr Driver

In this repo we will develop the TI LP5810 driver for Zephyr

## Load your FW
You should not clone this repo as a normal repo but instead load it with west:

Use it with Zephyr's `west` metatool, e.g.:
```
mkdir lp5810
cd lp5810
west init -m git@github.com:Telraam-Rear-Window-BV/LP5810-Zephyr-Driver.git
```
your repo data will be copied to the application directory:
```
cd application
west update
```

for building run following in `application` directory:
```
west build -b nrf52840dk_nrf52840
```

