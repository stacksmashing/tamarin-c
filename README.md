# Tamarin-C

A tool to explore USB-C on Apple devices using the [Tamarin C Hardware](https://github.com/stacksmashing/tamarin-c-hw).

With the firmware flashed you should get two serial ports: One for control, and one for UART. On the control port you should be able to see the following menu:

```
1: JTAG Mode (with Tamarin Probe support)
!: JTAG Mode (For external debugger)
2: DCSD Mode (with UART in Tamarin)
@: DCSD Mode (For external probe)
3: Reboot device
A: Map internal bus 1 (ACE SPMI on iPhone 15, I2C on macs)
B: Map internal bus 2 (Unknown on iPhone 15, I2C ACE communication on macs)
D: Enter DFU (Debug USB)
F: Fetch supported VDM actions
M: Set pin mapping
```

**This is experimental software and hardware. No warranty is given, and it is not guaranteed that this will not break your device.**

## Building/Installing

Built it yourself using the Pico SDK, or download one of the releases. Connect the Tamarin-C board while holding down the button on the Pico and it should appear as an external drive. Copy the .uf2 file of the release to the board and you should be ready to go!

## Hardware notes

The current Tamarin-C release only supports UART & SWD with Thunderbolt 3 or USB3 cables that have the SBU pins connected. A future release will also allow UART on the data lines.

Please bare with me while I add documentation :)

## Thanks

Tamarin-C is heavily based on the [Central Scrutinizer by Marc Zyngier](https://git.kernel.org/pub/scm/linux/kernel/git/maz/cs-sw.git/about/), which in turn is heavily based on [vdmtool by the AsahiLinux project](https://github.com/AsahiLinux/vdmtool/tree/master/vdmtool).

In addition, a lot of the functionality is based on information discovered by the [T8012 team](https://t8012.dev) and the [Asahi Linux USB-PD documentation](https://github.com/AsahiLinux/docs/wiki/HW%3AUSB-PD).

## License & copyright.

This release is licensed under the GPLv3 license. See LICENSE for details.

It was written by Thomas 'stacksmashing' Roth. See the single files for additioanl attribution of included dependencies.
