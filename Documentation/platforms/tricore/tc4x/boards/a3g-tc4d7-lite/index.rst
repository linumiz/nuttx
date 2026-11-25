==================================================
Infineon AURIX TC4D7 Lite Kit (KIT_A3G_TC4D7_LITE)
==================================================

.. tags:: arch:tricore, chip:tc4d7, vendor:infineon

The `KIT_A3G_TC4D7_LITE <https://www.infineon.com/evaluation-board/KIT-A3G-TC4D7-LITE>`__ is a low cost evaluation board of the
Infineon AURIX TC4D7 with an on-board debugger.  The debugger also
provides a USB serial port, which is connected to ASCLIN0 (TX on P14.0, RX
on P14.1) and used as the NuttX console at 115200 baud.

Toolchains
==========

The AURIX GCC and the TASKING compiler are supported, with make and CMake.

Configurations
==============

Each configuration is maintained in a sub-directory and can be selected as
follows::

    tools/configure.sh a3g-tc4d7-lite:<subdir>

nsh
---

NuttShell with the ostest and hello examples::

    tools/configure.sh a3g-tc4d7-lite:nsh

netnsh
------

NuttShell with the Ethernet, the ping and iperf commands, and the address
192.168.178.230/24::

    tools/configure.sh a3g-tc4d7-lite:netnsh

Ethernet
========

The DP83825I PHY is on port 0 of the GETH in RMII mode, with the TX pins on
P16.6, P16.8 and P16.13, the RX pins on P16.0, P16.1, P16.2 and P16.4, and
MDC and MDIO on P21.2 and P21.3.  The board polls the PHY once a second and
reports the link to the dwxgmac driver.  The DMA descriptors and the I/O
buffers are in the non-cached alias of LMU0, the ``.dma_data`` section of
the linker script.  10 and 100 Mbit/s are supported::

    nsh> ping -c 4 192.168.178.27
    nsh> iperf -s -i 1

User LEDs
=========

LED1 (P03.9) and LED2 (P03.10) are on when their pin is low.  With
``CONFIG_DEV_GPIO`` the board registers them as ``/dev/gpio0`` and
``/dev/gpio1``, off at boot, for example for the ``gpio`` example::

    nsh> gpio -o 0 /dev/gpio0

Flashing
========

The image can be programmed with the Infineon AURIX OpenOCD and the board
configuration ``board/infineon/kit_a3g_tc4d7_lite.cfg``::

    openocd -f board/infineon/kit_a3g_tc4d7_lite.cfg -c init -c "reset halt" \
            -c "program nuttx.hex verify" -c "reset run" -c shutdown
