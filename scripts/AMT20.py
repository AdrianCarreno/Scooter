# You must 'import spidev' in your script when calling this function
# and create a SPI object with
# enc = spidev.SpiDev()

nop_a5 = 0x00
rd_pos = 0x10
set_zero_point = 0x70

# This command causes a read of the current position.
def read_position(enc):
    resp = enc.xfer([rd_pos], 0, 20)[0]
    if resp == 0:
        raise IOError('Remote I/O error')

    while resp != rd_pos:
        resp = enc.xfer([nop_a5], 0, 20)[0]

    MSB = enc.xfer([nop_a5], 0, 20)[0]  # Most Significant Byte
    LSB = enc.xfer([nop_a5], 0, 20)[0]  # Least Significant Byte

    return (MSB << 8) | LSB

# This command sets the current position to zero and saves this setting in the EEPROM.
def set_zero(enc):
    resp = enc.xfer([set_zero_point], 0, 20)[0]
    if resp == 0:
        raise IOError('Remote I/O error')

    while resp != 0x80:
        resp = enc.xfer([nop_a5], 0, 20)[0]

    # The encoder must be power cycled. If the encoder is not power cycled, the position
    # values will not be calculated off the latest zero position. When the encoder is
    # powered on next the new offset will be used for the position calculation.

    return True
