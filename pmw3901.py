import utime
from machine import Pin, SPI

# Expected chip identifiers from the sensor datasheet
CHIP_ID = 0x49
CHIP_ID_INVERSE = 0xB6

class PMW3901:
    def __init__(self, spi: SPI, cs_pin: int):
        """
        Initialize the PMW3901 driver.
        
        :param spi: A pre-configured machine.SPI object.
        :param cs_pin: The GPIO number used for chip select.
        """
        self.spi = spi
        self.cs = Pin(cs_pin, Pin.OUT)
        self.cs.value(1)  # Ensure CS is high (inactive)

    def begin(self) -> bool:
        """
        Initializes the sensor by resetting the SPI bus,
        performing a power-on reset, and verifying chip IDs.
        
        :return: True if initialization succeeds, else False.
        """
        # Reset SPI bus
        self.cs.value(1)
        utime.sleep_ms(1)
        self.cs.value(0)
        utime.sleep_ms(1)
        self.cs.value(1)
        utime.sleep_ms(1)

        # Power on reset
        self.register_write(0x3A, 0x5A)
        utime.sleep_ms(5)

        # Verify chip ID and its inverse
        chip_id = self.register_read(0x00)
        chip_id_inverse = self.register_read(0x5F)
        if chip_id != CHIP_ID or chip_id_inverse != CHIP_ID_INVERSE:
            return False

        # Read motion registers to clear any residual data
        self.register_read(0x02)
        self.register_read(0x03)
        self.register_read(0x04)
        self.register_read(0x05)
        self.register_read(0x06)
        utime.sleep_ms(1)

        self.init_registers()
        return True

    def read_motion_count(self) -> (int, int):
        """
        Reads the motion count (delta X and Y) from the sensor.
        
        :return: A tuple (delta_x, delta_y).
        """
        # Latch the motion registers
        self.register_read(0x02)
        delta_x = (self.register_read(0x04) << 8) | self.register_read(0x03)
        delta_y = (self.register_read(0x06) << 8) | self.register_read(0x05)
        
        # Convert to signed 16-bit integers if necessary
        if delta_x & 0x8000:
            delta_x -= 0x10000
        if delta_y & 0x8000:
            delta_y -= 0x10000
            
        return delta_x, delta_y

    def enable_frame_buffer(self):
        """
        Configures the sensor to enable the frame buffer mode
        (i.e. to read a complete 35x35 pixel frame).
        """
        self.register_write(0x7F, 0x07)
        self.register_write(0x41, 0x1D)
        self.register_write(0x4C, 0x00)
        self.register_write(0x7F, 0x08)
        self.register_write(0x6A, 0x38)
        self.register_write(0x7F, 0x00)
        self.register_write(0x55, 0x04)
        self.register_write(0x40, 0x80)
        self.register_write(0x4D, 0x11)
        self.register_write(0x70, 0x00)
        self.register_write(0x58, 0xFF)

        # Wait until sensor indicates frame data is ready
        while True:
            temp = self.register_read(0x58)
            check = temp >> 6
            if check != 0x03:
                break
        utime.sleep_us(50)

    def read_frame_buffer(self) -> bytearray:
        """
        Reads one frame of pixel data from the sensor.
        The PMW3901 outputs a 35x35 pixel image (1225 pixels).
        
        :return: A bytearray of 1225 pixel values (8-bit each).
        """
        fb = bytearray(1225)
        count = 0
        mask = 0x0C  # Mask for extracting lower 2 bits
        
        for _ in range(1225):
            # Wait for valid pixel data
            while True:
                a = self.register_read(0x58)
                hold = a >> 6  # Get the top two status bits
                if hold not in (0x00, 0x03):
                    break
            if hold == 0x01:
                b = self.register_read(0x58)
                # Combine the upper 6 bits (from a) and lower 2 bits (from b)
                pixel = (a << 2) | (b & mask)
                fb[count] = pixel
                count += 1

        # Reset frame buffer state
        self.register_write(0x70, 0x00)
        self.register_write(0x58, 0xFF)

        while True:
            temp = self.register_read(0x58)
            check = temp >> 6
            if check != 0x03:
                break

        return fb

    def register_write(self, reg: int, value: int):
        """
        Writes a value to a sensor register via SPI.
        The register address is OR’ed with 0x80 to denote a write.
        
        :param reg: Register address.
        :param value: Value to write.
        """
        reg |= 0x80  # Set write flag
        self.cs.value(0)
        utime.sleep_us(50)
        self.spi.write(bytearray([reg, value]))
        utime.sleep_us(50)
        self.cs.value(1)
        utime.sleep_us(200)

    def register_read(self, reg: int) -> int:
        """
        Reads and returns the value from a sensor register via SPI.
        The register address has its MSB cleared to denote a read.
        
        :param reg: Register address.
        :return: The value read from the register.
        """
        reg &= 0x7F  # Clear the write flag
        self.cs.value(0)
        utime.sleep_us(50)
        self.spi.write(bytearray([reg]))
        utime.sleep_us(50)
        result = self.spi.read(1)
        utime.sleep_us(100)
        self.cs.value(1)
        return result[0]

    def init_registers(self):
        """
        Writes a series of configuration registers to optimize sensor performance.
        These register values are derived from the sensor documentation or calibration.
        """
        self.register_write(0x7F, 0x00)
        self.register_write(0x61, 0xAD)
        self.register_write(0x7F, 0x03)
        self.register_write(0x40, 0x00)
        self.register_write(0x7F, 0x05)
        self.register_write(0x41, 0xB3)
        self.register_write(0x43, 0xF1)
        self.register_write(0x45, 0x14)
        self.register_write(0x5B, 0x32)
        self.register_write(0x5F, 0x34)
        self.register_write(0x7B, 0x08)
        self.register_write(0x7F, 0x06)
        self.register_write(0x44, 0x1B)
        self.register_write(0x40, 0xBF)
        self.register_write(0x4E, 0x3F)
        self.register_write(0x7F, 0x08)
        self.register_write(0x65, 0x20)
        self.register_write(0x6A, 0x18)
        self.register_write(0x7F, 0x09)
        self.register_write(0x4F, 0xAF)
        self.register_write(0x5F, 0x40)
        self.register_write(0x48, 0x80)
        self.register_write(0x49, 0x80)
        self.register_write(0x57, 0x77)
        self.register_write(0x60, 0x78)
        self.register_write(0x61, 0x78)
        self.register_write(0x62, 0x08)
        self.register_write(0x63, 0x50)
        self.register_write(0x7F, 0x0A)
        self.register_write(0x45, 0x60)
        self.register_write(0x7F, 0x00)
        self.register_write(0x4D, 0x11)
        self.register_write(0x55, 0x80)
        self.register_write(0x74, 0x1F)
        self.register_write(0x75, 0x1F)
        self.register_write(0x4A, 0x78)
        self.register_write(0x4B, 0x78)
        self.register_write(0x44, 0x08)
        self.register_write(0x45, 0x50)
        self.register_write(0x64, 0xFF)
        self.register_write(0x65, 0x1F)
        self.register_write(0x7F, 0x14)
        self.register_write(0x65, 0x60)
        self.register_write(0x66, 0x08)
        self.register_write(0x63, 0x78)
        self.register_write(0x7F, 0x15)
        self.register_write(0x48, 0x58)
        self.register_write(0x7F, 0x07)
        self.register_write(0x41, 0x0D)
        self.register_write(0x43, 0x14)
        self.register_write(0x4B, 0x0E)
        self.register_write(0x45, 0x0F)
        self.register_write(0x44, 0x42)
        self.register_write(0x4C, 0x80)
        self.register_write(0x7F, 0x10)
        self.register_write(0x5B, 0x02)
        self.register_write(0x7F, 0x07)
        self.register_write(0x40, 0x41)
        self.register_write(0x70, 0x00)
        utime.sleep_ms(100)
        self.register_write(0x32, 0x44)
        self.register_write(0x7F, 0x07)
        self.register_write(0x40, 0x40)
        self.register_write(0x7F, 0x06)
        self.register_write(0x62, 0xF0)
        self.register_write(0x63, 0x00)
        self.register_write(0x7F, 0x0D)
        self.register_write(0x48, 0xC0)
        self.register_write(0x6F, 0xD5)
        self.register_write(0x7F, 0x00)
        self.register_write(0x5B, 0xA0)
        self.register_write(0x4E, 0xA8)
        self.register_write(0x5A, 0x50)
        self.register_write(0x40, 0x80)

    def set_led(self, led_on: bool):
        """
        Controls the onboard LED (if available) on the sensor.
        
        :param led_on: True to turn on the LED, False to turn it off.
        """
        utime.sleep_ms(200)
        self.register_write(0x7F, 0x14)
        self.register_write(0x6F, 0x1C if led_on else 0x00)
        self.register_write(0x7F, 0x00)
