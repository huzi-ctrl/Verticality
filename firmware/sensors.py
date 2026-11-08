"""
CanSat Team Verticality - Onboard Sensor Drivers (MicroPython / RP2040)
Target Hardware: Custom RP2040 CanSat Avionics & Bioluminescence Payload PCB
Sensors:
  - BMP280 Barometric Pressure & Temperature Sensor (SPI1, CS=GP11)
  - ADXL375 +-200g High-G MEMS Accelerometer (SPI1, CS=GP13, INT1=GP22)
  - 2x TSL2591 Ultra-High-Sensitivity Digital Light Sensors (I2C0 GP4/GP5 & I2C1 GP6/GP7)
  - 2x 10k NTC Thermistors (ADC1 GP27 & ADC2 GP28) + Battery Divider (ADC0 GP26)
"""

import math
import struct
import time


class BMP280_SPI:
    """SPI driver for Bosch BMP280 barometric pressure/altitude sensor."""

    REG_ID = 0xD0
    REG_RESET = 0xE0
    REG_STATUS = 0xF3
    REG_CTRL_MEAS = 0xF4
    REG_CONFIG = 0xF5
    REG_PRESS_MSB = 0xF7
    REG_CALIB = 0x88

    def __init__(self, spi, cs_pin, sea_level_hpa=1013.25):
        self.spi = spi
        self.cs = cs_pin
        self.cs.value(1)
        self.sea_level_hpa = sea_level_hpa
        self.t_fine = 0
        self._load_calibration()
        # Normal mode, Temp oversampling x2, Pressure oversampling x16
        self._write_reg(self.REG_CTRL_MEAS, 0x57)
        # Standby 0.5 ms, IIR filter coeff 16
        self._write_reg(self.REG_CONFIG, 0x10)

    def _read_regs(self, reg, length):
        buf = bytearray(length)
        self.cs.value(0)
        self.spi.write(bytes([reg | 0x80]))
        self.spi.readinto(buf)
        self.cs.value(1)
        return buf

    def _write_reg(self, reg, val):
        self.cs.value(0)
        self.spi.write(bytes([reg & 0x7F, val & 0xFF]))
        self.cs.value(1)

    def _load_calibration(self):
        raw = self._read_regs(self.REG_CALIB, 24)
        (
            self.dig_T1,
            self.dig_T2,
            self.dig_T3,
            self.dig_P1,
            self.dig_P2,
            self.dig_P3,
            self.dig_P4,
            self.dig_P5,
            self.dig_P6,
            self.dig_P7,
            self.dig_P8,
            self.dig_P9,
        ) = struct.unpack("<HhhHhhhhhhhh", raw)

    def read(self):
        """Return (temperature_c, pressure_hpa, altitude_m)."""
        data = self._read_regs(self.REG_PRESS_MSB, 6)
        adc_p = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        adc_t = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)

        var1 = (adc_t / 16384.0 - self.dig_T1 / 1024.0) * self.dig_T2
        var2 = ((adc_t / 131072.0 - self.dig_T1 / 8192.0) ** 2) * self.dig_T3
        self.t_fine = int(var1 + var2)
        temp_c = (var1 + var2) / 5120.0

        var1 = (self.t_fine / 2.0) - 64000.0
        var2 = var1 * var1 * self.dig_P6 / 32768.0
        var2 = var2 + var1 * self.dig_P5 * 2.0
        var2 = (var2 / 4.0) + (self.dig_P4 * 65536.0)
        var1 = (self.dig_P3 * var1 * var1 / 524288.0 + self.dig_P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self.dig_P1
        if var1 == 0:
            return temp_c, 0.0, 0.0
        p = 1048576.0 - adc_p
        p = ((p - (var2 / 4096.0)) * 6250.0) / var1
        var1 = self.dig_P9 * p * p / 2147483648.0
        var2 = p * self.dig_P8 / 32768.0
        p = p + (var1 + var2 + self.dig_P7) / 16.0
        press_hpa = p / 100.0
        altitude_m = 44330.0 * (1.0 - (press_hpa / self.sea_level_hpa) ** 0.1903)
        return temp_c, press_hpa, altitude_m


class ADXL375_SPI:
    """SPI driver for Analog Devices ADXL375 +-200g 3-axis accelerometer."""

    REG_DEVID = 0x00
    REG_THRESH_SHOCK = 0x1D
    REG_DUR = 0x21
    REG_BW_RATE = 0x2C
    REG_POWER_CTL = 0x2D
    REG_INT_ENABLE = 0x2E
    REG_INT_MAP = 0x2F
    REG_INT_SOURCE = 0x30
    REG_DATA_FORMAT = 0x31
    REG_DATAX0 = 0x32

    SCALE_G_PER_LSB = 0.049  # 49 mg/LSB typical for ADXL375

    def __init__(self, spi, cs_pin, shock_threshold_g=15.0):
        self.spi = spi
        self.cs = cs_pin
        self.cs.value(1)
        # 4-wire SPI, active-high interrupt, right-justified
        self._write_reg(self.REG_DATA_FORMAT, 0x0B)
        # 400 Hz output data rate
        self._write_reg(self.REG_BW_RATE, 0x0C)
        # Configure single-shock interrupt on INT1 (780 mg/LSB scale)
        thresh_code = max(1, min(255, int(shock_threshold_g / 0.78)))
        self._write_reg(self.REG_THRESH_SHOCK, thresh_code)
        self._write_reg(self.REG_DUR, 0x10)  # 10 ms shock duration
        self._write_reg(self.REG_INT_MAP, 0x00)  # Route shock to INT1 (GP22)
        self._write_reg(self.REG_INT_ENABLE, 0x40)  # Enable SINGLE_SHOCK
        # Enter measurement mode
        self._write_reg(self.REG_POWER_CTL, 0x08)

    def _write_reg(self, reg, val):
        self.cs.value(0)
        self.spi.write(bytes([reg & 0x3F, val & 0xFF]))
        self.cs.value(1)

    def _read_regs(self, reg, length):
        buf = bytearray(length)
        cmd = (reg & 0x3F) | 0x80 | (0x40 if length > 1 else 0x00)
        self.cs.value(0)
        self.spi.write(bytes([cmd]))
        self.spi.readinto(buf)
        self.cs.value(1)
        return buf

    def read_accel_g(self):
        """Return (ax_g, ay_g, az_g, mag_g)."""
        raw = self._read_regs(self.REG_DATAX0, 6)
        x, y, z = struct.unpack("<hhh", raw)
        ax = x * self.SCALE_G_PER_LSB
        ay = y * self.SCALE_G_PER_LSB
        az = z * self.SCALE_G_PER_LSB
        mag = math.sqrt(ax * ax + ay * ay + az * az)
        return ax, ay, az, mag


class TSL2591_I2C:
    """I2C driver for AMS TSL2591 high-dynamic-range digital light sensor (bioluminescence)."""

    I2C_ADDR = 0x29
    CMD_BIT = 0xA0
    REG_ENABLE = 0x00
    REG_CONTROL = 0x01
    REG_C0DATAL = 0x14

    GAIN_LOW = 0x00   # 1x
    GAIN_MED = 0x10   # 25x
    GAIN_HIGH = 0x20  # 428x
    GAIN_MAX = 0x30   # 9876x (used for dark-chamber bioluminescence detection)

    ATIME_100MS = 0x00
    ATIME_200MS = 0x01
    ATIME_400MS = 0x03
    ATIME_600MS = 0x05

    def __init__(self, i2c, gain=GAIN_MAX, atime=ATIME_200MS):
        self.i2c = i2c
        self.gain = gain
        self.atime = atime
        self._write_reg(self.REG_ENABLE, 0x03)  # PON | AEN
        self._write_reg(self.REG_CONTROL, self.gain | self.atime)

    def _write_reg(self, reg, val):
        self.i2c.writeto_mem(self.I2C_ADDR, self.CMD_BIT | reg, bytes([val]))

    def read_raw_and_lux(self):
        """Return (ch0_full, ch1_ir, visible_counts, millilux)."""
        buf = self.i2c.readfrom_mem(self.I2C_ADDR, self.CMD_BIT | self.REG_C0DATAL, 4)
        ch0, ch1 = struct.unpack("<HH", buf)
        visible = max(0, ch0 - ch1)

        atime_ms = (self.atime + 1) * 100.0
        gain_map = {0x00: 1.0, 0x10: 25.0, 0x20: 428.0, 0x30: 9876.0}
        again = gain_map.get(self.gain, 9876.0)
        cpl = (atime_ms * again) / 408.0
        if ch0 == 0xFFFF or ch1 == 0xFFFF or cpl <= 0:
            return ch0, ch1, visible, -1.0
        lux = (((ch0 - ch1)) * (1.0 - (ch1 / ch0 if ch0 > 0 else 0.0))) / cpl
        return ch0, ch1, visible, max(0.0, lux * 1000.0)


def read_ntc_temp_c(adc_obj, r_pullup=10000.0, r0=10000.0, beta=3950.0, t0_c=25.0):
    """
    Read 10k NTC thermistor connected between ADC pin and GND with 10k pull-up to +3V3.
    Schematic nets:
      - /NTC_A (GP27 / ADC1): R13 (10k pull-up), C23 (100nF filter), TH1 (10k NTC)
      - /NTC_B (GP28 / ADC2): R14 (10k pull-up), C24 (100nF filter), TH2 (10k NTC)
    """
    raw = adc_obj.read_u16()
    raw = max(16, min(65519, raw))
    r_ntc = r_pullup * (raw / (65535.0 - raw))
    inv_t = (1.0 / (t0_c + 273.15)) + (1.0 / beta) * math.log(r_ntc / r0)
    return (1.0 / inv_t) - 273.15


def read_battery_voltage(adc_obj, vref=3.3, divider_ratio=2.0):
    """
    Read Li-Ion battery voltage via R15 (100k) / R16 (100k) 2:1 divider on GP26 (ADC0).
    """
    raw = adc_obj.read_u16()
    return (raw / 65535.0) * vref * divider_ratio
