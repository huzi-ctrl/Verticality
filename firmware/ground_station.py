"""
CanSat Team Verticality - 433.175 MHz LoRa Ground Station Receiver
Decodes the 34-byte little-endian binary telemetry frame (PDR Appendix D),
verifies the 16-bit CRC-CCITT checksum (skipping corrupted frames without crashing
per PDR Test T7), logs two redundant CSV copies, and streams decoded telemetry over USB serial.
"""

import struct
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

TELEM_STRUCT_FULL = "<HBIIhhhIIHHHBH"
FRAME_LEN_BYTES = 34

STATE_NAMES = {
    0: "SELF_TEST",
    1: "PAD",
    2: "BOOST",
    3: "DESCENT",
    4: "LANDED",
}


def crc16_ccitt(data_bytes, poly=0x1021, init_val=0xFFFF):
    """Compute 16-bit CRC-CCITT over bytes 0..31 of the 34-byte frame."""
    crc = init_val
    for b in data_bytes:
        crc ^= (b & 0xFF) << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ poly) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def decode_telemetry_frame(raw_bytes):
    """
    Unpack and validate a 34-byte binary telemetry frame.
    Returns (dict_of_fields, True) if CRC-16 is valid, or (None, False) if corrupted.
    """
    if len(raw_bytes) != FRAME_LEN_BYTES:
        return None, False
    calc_crc = crc16_ccitt(raw_bytes[:32])
    (
        pkt_num,
        state,
        mission_time_ms,
        pressure_pa,
        air_temp_c_x100,
        chamber_a_temp_x100,
        chamber_b_temp_x100,
        light_a,
        light_b,
        peak_accel_g_x10,
        rms_jerk_gps,
        battery_mv,
        heater_duty_pct,
        rx_crc16,
    ) = struct.unpack(TELEM_STRUCT_FULL, raw_bytes)
    if calc_crc != rx_crc16:
        return None, False
    return {
        "pkt_num": pkt_num,
        "state": state,
        "state_name": STATE_NAMES.get(state, f"UNK_{state}"),
        "mission_time_ms": mission_time_ms,
        "pressure_pa": pressure_pa,
        "air_temp_c": air_temp_c_x100 / 100.0,
        "chamber_a_c": chamber_a_temp_x100 / 100.0,
        "chamber_b_c": chamber_b_temp_x100 / 100.0,
        "light_a": light_a,
        "light_b": light_b,
        "peak_accel_g": peak_accel_g_x10 / 10.0,
        "rms_jerk_gps": rms_jerk_gps,
        "battery_mv": battery_mv,
        "heater_duty_pct": heater_duty_pct,
        "crc16": rx_crc16,
    }, True


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
        freq_mhz=433.175,
    )
    lora.start_rx_continuous()
    print("CanSat Ground Station Listening on 433.175 MHz (SF8, BW125kHz, CR4/5, 34B binary+CRC16)...")

    header = (
        "pkt_num,state,mission_time_ms,pressure_pa,air_temp_c,chamber_a_c,chamber_b_c,"
        "light_a,light_b,peak_accel_g,rms_jerk_gps,battery_mv,heater_duty_pct,rssi_dbm,snr_db\n"
    )
    log_primary = open("ground_log_primary.csv", "a")
    log_backup = open("ground_log_backup.csv", "a")
    log_primary.write(header)
    log_backup.write(header)

    crc_errors = 0
    while True:
        res = lora.receive_packet()
        if res is not None:
            raw_bytes, rssi_dbm, snr_db = res
            led.value(1)
            decoded, ok = decode_telemetry_frame(raw_bytes)
            if not ok:
                crc_errors += 1
                print(f"# CRC/LEN ERROR (count={crc_errors}, len={len(raw_bytes)}, RSSI={rssi_dbm})")
            else:
                line = (
                    f"{decoded['pkt_num']},{decoded['state_name']},{decoded['mission_time_ms']},"
                    f"{decoded['pressure_pa']},{decoded['air_temp_c']:.2f},"
                    f"{decoded['chamber_a_c']:.2f},{decoded['chamber_b_c']:.2f},"
                    f"{decoded['light_a']},{decoded['light_b']},"
                    f"{decoded['peak_accel_g']:.1f},{decoded['rms_jerk_gps']},"
                    f"{decoded['battery_mv']},{decoded['heater_duty_pct']},"
                    f"{rssi_dbm},{snr_db:.1f}"
                )
                print(line)
                log_primary.write(line + "\n")
                log_backup.write(line + "\n")
                log_primary.flush()
                log_backup.flush()
            led.value(0)
        time.sleep_ms(10)


if __name__ == "__main__":
    main()
