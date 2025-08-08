# recipes-bsp/bootfiles/rpi-config_%.bbappend
FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

RPI_EXTRA_CONFIG:append = "\n# DHT11 sensor overlay\n\
dtoverlay=dht11"

RPI_EXTRA_CONFIG:append = "\n# LED overlay \n\
dtoverlay=led\n"

RPI_EXTRA_CONFIG:append = "\n# DHT11 mmio sensor overlay\n\
dtoverlay=dht11mmio"
