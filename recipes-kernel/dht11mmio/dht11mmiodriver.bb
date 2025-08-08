SUMMARY = "DHT11 Character Device Driver (MMIO)"
DESCRIPTION = "A simple DHT11 character device driver using ioremap on Raspberry Pi 4"
LICENSE = "CLOSED"

SRC_URI = " \
    file://dht11mmiodriver/dht11mmiodriver.c \
    file://dht11mmiodriver/Makefile \
"

S = "${WORKDIR}/dht11mmiodriver"

inherit module

EXTRA_OEMAKE += "KERNELDIR=${STAGING_KERNEL_DIR}"

do_compile() {
    make -C ${STAGING_KERNEL_DIR} M=${S} modules
}

do_install() {
    # Install kernel module
    install -d ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra
    install -m 0644 ${S}/dht11mmiodriver.ko ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra/

    # Auto-load at boot
    install -d ${D}${sysconfdir}/modules-load.d
    echo "dht11mmiodriver" > ${D}${sysconfdir}/modules-load.d/dht11mmiodriver.conf

    # Set device permissions
    install -d ${D}${sysconfdir}/udev/rules.d
    echo 'KERNEL=="dht11", MODE="0666"' > ${D}${sysconfdir}/udev/rules.d/99-dht11mmio.rules
}

FILES:${PN} += " \
    ${sysconfdir}/modules-load.d/dht11mmiodriver.conf \
    ${sysconfdir}/udev/rules.d/99-dht11mmio.rules \
"

COMPATIBLE_MACHINE = "raspberrypi4-64"
