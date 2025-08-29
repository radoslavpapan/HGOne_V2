import struct
import utime

_CONV_REG      = 0x00
_CONFIG_REG    = 0x01

_VOLTAGE_RANGE = 0x0000  # ±6.144V

_SINGLE_SHOT_MODE = 0x0100
_COMP_0_GND = 0x4000
_COMP_1_GND = 0x5000
_COMP_2_GND = 0x6000
_COMP_3_GND = 0x7000

_SPS_128 = 0x0080

_REG_FACTOR = 0x7FFF

class ADS1115:
    def __init__(self, i2c, addr=0x48):
        self._i2c = i2c
        self._address = addr

    def begin(self):
        try:
            _ = self._read_reg(_CONFIG_REG)
            return True
        except Exception:
            return False

    def read_voltage(self, channel):
        raw = self.read_adc(channel)
        if raw > 0x7FFF:
            raw -= 0x10000

        voltage = (raw * 6.144) / _REG_FACTOR
        return round(voltage, 4)
    
    def read_adc(self, channel):
        if channel not in (0, 1, 2, 3):
            raise ValueError("Invalid channel")

        mux_bits = {
            0: _COMP_0_GND,
            1: _COMP_1_GND,
            2: _COMP_2_GND,
            3: _COMP_3_GND
        }[channel]

        config = (0x8000
                  | mux_bits
                  | _VOLTAGE_RANGE
                  | _SPS_128
                  | _SINGLE_SHOT_MODE)

        self._write_reg(_CONFIG_REG, config)
        
        utime.sleep(0.01)

        raw = self._read_reg(_CONV_REG)  # už unsigned
        return raw

    def _write_reg(self, reg, val):
        data = struct.pack(">H", val)
        self._i2c.writeto_mem(self._address, reg, data)

    def _read_reg(self, reg):
        data = self._i2c.readfrom_mem(self._address, reg, 2)
        return struct.unpack(">H", data)[0]
