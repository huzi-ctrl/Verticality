"""
CanSat Team Verticality - Ai-Thinker Ra-02 (Semtech SX1278) 433 MHz LoRa Driver
SPI0 Pin Mapping on CanSat PCB:
  - GP16 (U1 Pin 27): /RA_MISO -> U5 Pin 13 (MISO)
  - GP17 (U1 Pin 28): /RA_CS   -> U5 Pin 15 (NSS)
  - GP18 (U1 Pin 29): /RA_SCLK -> U5 Pin 12 (SCK)
  - GP19 (U1 Pin 30): /RA_MOSI -> U5 Pin 14 (MOSI)
  - GP20 (U1 Pin 31): /RA_RESET-> U5 Pin 4  (RESET)
  - GP21 (U1 Pin 32): /RA_DIO0 -> U5 Pin 5  (DIO0)
"""

import time


class SX1278:
    REG_FIFO = 0x00
    REG_OP_MODE = 0x01
    REG_FRF_MSB = 0x06
    REG_FRF_MID = 0x07
    REG_FRF_LSB = 0x08
    REG_PA_CONFIG = 0x09
    REG_OCP = 0x0B
    REG_LNA = 0x0C
    REG_FIFO_ADDR_PTR = 0x0D
    REG_FIFO_TX_BASE_ADDR = 0x0E
    REG_FIFO_RX_BASE_ADDR = 0x0F
    REG_FIFO_RX_CURRENT_ADDR = 0x10
    REG_IRQ_FLAGS = 0x12
    REG_RX_NB_BYTES = 0x13
    REG_PKT_SNR_VALUE = 0x19
    REG_PKT_RSSI_VALUE = 0x1A
    REG_MODEM_CONFIG_1 = 0x1D
    REG_MODEM_CONFIG_2 = 0x1E
    REG_PAYLOAD_LENGTH = 0x22
    REG_MODEM_CONFIG_3 = 0x26
    REG_SYNC_WORD = 0x39
    REG_DIO_MAPPING_1 = 0x40
    REG_VERSION = 0x42
    REG_PA_DAC = 0x4D

    MODE_LONG_RANGE_MODE = 0x80
    MODE_SLEEP = 0x00
    MODE_STDBY = 0x01
    MODE_TX = 0x03
    MODE_RX_CONTINUOUS = 0x05

    IRQ_TX_DONE_MASK = 0x08
    IRQ_RX_DONE_MASK = 0x40
    IRQ_PAYLOAD_CRC_ERROR_MASK = 0x20

    def __init__(self, spi, cs, rst, dio0=None, freq_mhz=433.0, tx_power_dbm=17):
        self.spi = spi
        self.cs = cs
        self.rst = rst
        self.dio0 = dio0
        self.cs.value(1)
        self.reset()

        version = self._read_reg(self.REG_VERSION)
        if version != 0x12:
            raise RuntimeError(f"SX1278 not detected (REG_VERSION=0x{version:02X})")

        self.sleep()
        self.set_frequency(freq_mhz)

        self._write_reg(self.REG_FIFO_TX_BASE_ADDR, 0x00)
        self._write_reg(self.REG_FIFO_RX_BASE_ADDR, 0x00)
        self._write_reg(self.REG_LNA, self._read_reg(self.REG_LNA) | 0x03)
        self._write_reg(self.REG_MODEM_CONFIG_3, 0x04)  # LowDataRateOptimize=0, AGC auto on

        # BW=125 kHz (0x70), CR=4/5 (0x02), Explicit Header (0x00)
        self._write_reg(self.REG_MODEM_CONFIG_1, 0x72)
        # SF=9 (0x90), CRC On (0x04)
        self._write_reg(self.REG_MODEM_CONFIG_2, 0x94)
        # Private CanSat SyncWord
        self._write_reg(self.REG_SYNC_WORD, 0x12)

        self.set_tx_power(tx_power_dbm)
        self.standby()

    def reset(self):
        self.rst.value(0)
        time.sleep_ms(10)
        self.rst.value(1)
        time.sleep_ms(10)

    def _read_reg(self, addr):
        buf = bytearray(1)
        self.cs.value(0)
        self.spi.write(bytes([addr & 0x7F]))
        self.spi.readinto(buf)
        self.cs.value(1)
        return buf[0]

    def _write_reg(self, addr, val):
        self.cs.value(0)
        self.spi.write(bytes([(addr & 0x7F) | 0x80, val & 0xFF]))
        self.cs.value(1)

    def sleep(self):
        self._write_reg(self.REG_OP_MODE, self.MODE_LONG_RANGE_MODE | self.MODE_SLEEP)

    def standby(self):
        self._write_reg(self.REG_OP_MODE, self.MODE_LONG_RANGE_MODE | self.MODE_STDBY)

    def set_frequency(self, freq_mhz):
        frf = int((freq_mhz * 1_000_000.0) / (32_000_000.0 / 524288.0))
        self._write_reg(self.REG_FRF_MSB, (frf >> 16) & 0xFF)
        self._write_reg(self.REG_FRF_MID, (frf >> 8) & 0xFF)
        self._write_reg(self.REG_FRF_LSB, frf & 0xFF)

    def set_tx_power(self, level_dbm):
        level_dbm = max(2, min(17, int(level_dbm)))
        self._write_reg(self.REG_PA_DAC, 0x84)
        self._write_reg(self.REG_OCP, 0x2B)
        self._write_reg(self.REG_PA_CONFIG, 0x80 | (level_dbm - 2))

    def send_packet(self, payload_bytes, timeout_ms=500):
        if isinstance(payload_bytes, str):
            payload_bytes = payload_bytes.encode("utf-8")
        self.standby()
        self._write_reg(self.REG_IRQ_FLAGS, 0xFF)
        self._write_reg(self.REG_FIFO_ADDR_PTR, 0x00)

        self.cs.value(0)
        self.spi.write(bytes([self.REG_FIFO | 0x80]))
        self.spi.write(payload_bytes)
        self.cs.value(1)

        self._write_reg(self.REG_PAYLOAD_LENGTH, len(payload_bytes))
        self._write_reg(self.REG_OP_MODE, self.MODE_LONG_RANGE_MODE | self.MODE_TX)

        t0 = time.ticks_ms()
        while (self._read_reg(self.REG_IRQ_FLAGS) & self.IRQ_TX_DONE_MASK) == 0:
            if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
                self.standby()
                return False
            time.sleep_ms(2)

        self._write_reg(self.REG_IRQ_FLAGS, self.IRQ_TX_DONE_MASK)
        self.standby()
        return True

    def receive_packet(self):
        """Non-blocking packet check in continuous RX mode. Returns (bytes, rssi_dbm, snr_db) or None."""
        irq = self._read_reg(self.REG_IRQ_FLAGS)
        if (irq & self.IRQ_RX_DONE_MASK) == 0:
            return None
        self._write_reg(self.REG_IRQ_FLAGS, 0xFF)
        if (irq & self.IRQ_PAYLOAD_CRC_ERROR_MASK) != 0:
            return None

        current_addr = self._read_reg(self.REG_FIFO_RX_CURRENT_ADDR)
        nb_bytes = self._read_reg(self.REG_RX_NB_BYTES)
        self._write_reg(self.REG_FIFO_ADDR_PTR, current_addr)

        buf = bytearray(nb_bytes)
        self.cs.value(0)
        self.spi.write(bytes([self.REG_FIFO & 0x7F]))
        self.spi.readinto(buf)
        self.cs.value(1)

        pkt_snr = self._read_reg(self.REG_PKT_SNR_VALUE)
        if pkt_snr > 127:
            pkt_snr -= 256
        snr_db = pkt_snr * 0.25
        rssi_dbm = -164 + self._read_reg(self.REG_PKT_RSSI_VALUE)
        return bytes(buf), rssi_dbm, snr_db

    def start_rx_continuous(self):
        self._write_reg(self.REG_FIFO_ADDR_PTR, 0x00)
        self._write_reg(self.REG_OP_MODE, self.MODE_LONG_RANGE_MODE | self.MODE_RX_CONTINUOUS)
