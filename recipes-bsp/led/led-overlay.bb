SUMMARY = "LED Overlay Recipe"
DESCRIPTION = "Builds device tree overlay for led"
LICENSE = "CLOSED"

inherit devicetree deploy

SRC_URI = "file://led.dts"
S = "${WORKDIR}"

PROVIDES = ""

LED_NAME = "led.dtbo"

do_compile() {
    echo "Compiling LED overlay..."

    dtc -@ -I dts -O dtb \
        -i ${STAGING_KERNEL_DIR}/include \
        -i ${S} \
        -o ${S}/${LED_NAME} \
        ${S}/led.dts
}

do_install() {
    install -d ${D}${nonarch_base_libdir}/firmware/overlays
    install -m 0644 ${S}/${LED_NAME} ${D}${nonarch_base_libdir}/firmware/overlays/
}

do_deploy() {
    install -d ${DEPLOYDIR}/overlays
    install -m 0644 ${S}/${LED_NAME} ${DEPLOYDIR}/overlays/
}

FILES:${PN} = "${nonarch_base_libdir}/firmware/overlays/${LED_NAME}"

COMPATIBLE_MACHINE = "raspberrypi4-64"
