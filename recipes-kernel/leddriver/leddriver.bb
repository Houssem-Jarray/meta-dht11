SUMMARY = "LED Character Device Driver"
DESCRIPTION = "Simple LED driver using platform registration (no device tree)"
LICENSE = "CLOSED"

SRC_URI = "file://leddriver/leddriver.c \
           file://leddriver/Makefile"

S = "${WORKDIR}/leddriver"

inherit module

EXTRA_OEMAKE += "KERNELDIR=${STAGING_KERNEL_DIR}"

do_compile() {
    make -C ${STAGING_KERNEL_DIR} M=${S} modules
}

do_install() {
    # Install kernel module
    install -d ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra
    install -m 0644 ${S}/leddriver.ko ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra/

    # Autoload at boot
    install -d ${D}${sysconfdir}/modules-load.d
    echo "leddriver" > ${D}${sysconfdir}/modules-load.d/leddriver.conf

    # Udev rule for /dev/leddriver
    install -d ${D}${sysconfdir}/udev/rules.d
    echo 'KERNEL=="leddriver", MODE="0666"' > ${D}${sysconfdir}/udev/rules.d/99-leddriver.rules
}

FILES:${PN} += "${sysconfdir}/modules-load.d/leddriver.conf"
FILES:${PN} += "${sysconfdir}/udev/rules.d/99-leddriver.rules"

COMPATIBLE_MACHINE = "raspberrypi4-64"
