class LM75:
    def __init__(self, i2c, addr=0x48):
        self.i2c = i2c
        self.addr = addr

    def begin(self):
        devices = self.i2c.scan()
        if self.addr in devices:
            return True
        else:
            return False

    def read_temp(self):
        data = self.i2c.readfrom_mem(self.addr, 0x00, 2)
        temp_raw = (data[0] << 8 | data[1]) >> 5
        if temp_raw & (1 << 10):
            temp_raw -= 1 << 11
        return round(temp_raw * 0.125, 1)