"""
CanSat Team Verticality - 433 MHz LoRa Ground Station Receiver (MicroPython / RP2040)
Receives CSV telemetry packets from the CanSat over Ai-Thinker Ra-02 (SX1278),
appends RSSI (dBm) and SNR (dB), and streams live CSV lines over USB serial.
"""

import time
from machine import Pin, SPI
from sx1278 import SX1278

PIN_RA_MISO = 16
PIN_RA_CS = 17
PIN_RA_SCLK = 18
PIN_RA_MOSI = 19
PIN_RA_RESET = 20
PIN_RA_DIO0 = 21
PIN_LED = 25


def main():
    led = Pin(PIN_LED, Pin.OUT, value=0)
    spi0 = SPI(
        0,
        baudrate=4_000_000,
        polarity=0,
        phase=0,
        sck=Pin(PIN_RA_SCLK),
        mosi=Pin(PIN_RA_MOSI),
        miso=Pin(PIN_RA_MISO),
    )
    lora = SX1278(
        spi0,
        cs=Pin(PIN_RA_CS, Pin.OUT, value=1),
        rst=Pin(PIN_RA_RESET, Pin.OUT, value=1),
        dio0=Pin(PIN_RA_DIO0, Pin.IN),
        freq_mhz=433.0,
    )
    lora.start_rx_continuous()
    print("CanSat Ground Station Listening on 433.0 MHz (SF9, BW125kHz, CR4/5)...")

    while True:
        res = lora.receive_packet()
        if res is not None:
            raw_bytes, rssi_dbm, snr_db = res
            led.value(1)
            try:
                text = raw_bytes.decode("utf-8").strip()
            except UnicodeError:
                text = raw_bytes.hex()
            print(f"{text},RSSI={rssi_dbm},SNR={snr_db:.1f}")
            led.value(0)
        time.sleep_ms(10)


if __name__ == "__main__":
    main()
