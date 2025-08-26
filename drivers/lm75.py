
class LM75:
    """
    Synchronous driver for the LM75 temperature sensor.
    Uses I2C communication.
    """
    
    def __init__(self, i2c, addr=0x48):
        """
        Initialize the LM75 sensor.

        Args:
            i2c: An initialized I2C object.
            addr (int): The I2C address of the sensor (default is 0x48).
        """
        self.i2c = i2c
        self.addr = addr

    def begin(self):
        """
        Check if the LM75 sensor is present on the I2C bus.

        Returns:
            bool: True if the sensor is found, False otherwise.
        """
        devices = self.i2c.scan()
        return self.addr in devices

    def read_temp(self):
        """
        Read the temperature from the LM75 sensor.

        Returns:
            float: Temperature in Celsius, rounded to 1 decimal place.
        """
        data = self.i2c.readfrom_mem(self.addr, 0x00, 2)

        # Convert 11-bit two's complement value to Celsius
        temp_raw = (data[0] << 8 | data[1]) >> 5
        if temp_raw & (1 << 10):  # If sign bit is set
            temp_raw -= 1 << 11    # Apply two's complement

        return round(temp_raw * 0.125, 1)