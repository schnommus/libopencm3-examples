# BOOTLOADER README

These examples are built for the vanilla (non-DFU, AN0042) bootloader present on board revision 0.3.

For different bootloaders, modify the linker script and SCB\_VTOR assignment as required.

Use:
    arm-none-eabi-objcopy -O binary <infile>.elf <outfile>.bin

To create a raw binary that is transferable to the Tomu board using XMODEM.
