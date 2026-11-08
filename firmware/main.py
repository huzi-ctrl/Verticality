"""
CanSat Team Verticality - Flight Controller & Bioluminescence Payload Firmware
Target MCU: Raspberry Pi RP2040 (U1) + W25Q128JVS 16MB QSPI Flash (U2)
Reference Documents:
  - LUSoM Team Verticality PDR (Updated v2)
  - CanSat PCB Design Guide (Team Verticality Rev 2)
Mission Summary:
  - Maintain Chamber A (warmed experimental Pyrocystis fusiformis dinoflagellate culture;
    UK backup species Pyrocystis lunula) at 22.0 +/- 1.0 C using closed-loop PID control
    on an off-board 15 Ohm 2W axial resistor (R8 via Q1 AO3400A on GP8), while Chamber B
    remains an unheated ambient flight control.
  - Measure bioluminescence emission intensity in both dark chambers using dual TSL2591
    sensors (U7 on I2C0 GP4/GP5, U8 on I2C1 GP6/GP7) at 9876x gain.
  - Track ascent, apogee, descent rate, and high-g shock/jerk via BMP280 (SPI1 CS=GP11)
    and ADXL375 +-200g accelerometer (SPI1 CS=GP13, INT1=GP22).
  - Execute the 5-state flight machine (0=SELF_TEST, 1=PAD, 2=BOOST, 3=DESCENT, 4=LANDED):
      * 5 s RAM ring buffer in PAD, flushed to flash upon BOOST trigger (>25 m or >4 g).
      * Pre-touchdown landing leg release in DESCENT via MG90S servo (Q2 on GP9, PWM on GP10)
        when altitude drops below 150.0 m above pad OR 15.0 s after apogee, whichever comes first
        (locked out in SELF_TEST, PAD, and BOOST).
  - Log full-rate CSV telemetry to onboard 16 MB QSPI flash and transmit 1 Hz 34-byte
    little-endian binary LoRa frames with CRC-16-CCITT at 433.175 MHz (SF8, BW125, CR4/5).
"""

import math
import struct
import time
from machine import ADC, I2C, PWM, SPI, Pin, UART
from sensors import ADXL375_SPI, BMP280_SPI, TSL2591_I2C, read_battery_voltage, read_ntc_temp_c
from sx1278 import SX1278

# ==============================================================================
# 1. Exact Schematic Pin Definitions (CanSat.kicad_sch)
# ==============================================================================
PIN_UART_TX = 0       # GP0 -> /RX (TP4 UART debug, MCU TX)
PIN_UART_RX = 1       # GP1 -> /TX (TP5 UART debug, MCU RX)

PIN_INT_A = 2         # GP2 -> /INT_A (U7 TSL2591 Chamber A interrupt)
PIN_INT_B = 3         # GP3 -> /INT_B (U8 TSL2591 Chamber B interrupt)
PIN_I2C0_SDA = 4      # GP4 -> /SDA0  (U7 TSL2591 Chamber A)
PIN_I2C0_SCL = 5      # GP5 -> /SCL0  (U7 TSL2591 Chamber A)
PIN_I2C1_SDA = 6      # GP6 -> /SDA1  (U8 TSL2591 Chamber B)
PIN_I2C1_SCL = 7      # GP7 -> /SCL1  (U8 TSL2591 Chamber B)

PIN_HEATER_PWM = 8    # GP8 -> /HEATER (R7 -> Q1 AO3400A gate -> R8 15R 2W heater)
PIN_SERVO_PWR = 9     # GP9 -> /SERVO_P (R10 -> Q2 AO3400A gate -> J6 Servo GND switch)
PIN_SERVO_PWM = 10    # GP10 -> /SERVO_S (J6.3 MG90S PWM signal)

PIN_PRESS_CS = 11     # GP11 -> /PRESS_CS (U6 BMP280 CSB)
PIN_SPI1_MISO = 12    # GP12 -> /IMU_MISO (U4 ADXL375 & U6 BMP280 SDO)
PIN_IMU_CS = 13       # GP13 -> /IMU_CS   (U4 ADXL375 CS)
PIN_SPI1_SCLK = 14    # GP14 -> /IMU_SCLK (U4 ADXL375 & U6 BMP280 SCL/SCK)
PIN_SPI1_MOSI = 15    # GP15 -> /IMU_MOSI (U4 ADXL375 & U6 BMP280 SDA/SDI)

PIN_RA_MISO = 16      # GP16 -> /RA_MISO  (U5 Ra-02 SX1278 MISO)
PIN_RA_CS = 17        # GP17 -> /RA_CS    (U5 Ra-02 SX1278 NSS)
PIN_RA_SCLK = 18      # GP18 -> /RA_SCLK  (U5 Ra-02 SX1278 SCK)
PIN_RA_MOSI = 19      # GP19 -> /RA_MOSI  (U5 Ra-02 SX1278 MOSI)
PIN_RA_RESET = 20     # GP20 -> /RA_RESET (U5 Ra-02 SX1278 RESET)
PIN_RA_DIO0 = 21      # GP21 -> /RA_DIO0  (U5 Ra-02 SX1278 DIO0)

PIN_IMU_INT1 = 22     # GP22 -> /IMU_INT1 (U4 ADXL375 Shock Interrupt)
PIN_STATUS_LED = 25   # GP25 -> /LED      (R12 -> D1 Green Status LED)

PIN_BATT_SENSE = 26   # GP26 / ADC0 -> /BATT_SENSE (R15/R16 100k/100k divider)
PIN_NTC_A = 27        # GP27 / ADC1 -> /NTC_A      (TH1 Chamber A 10k NTC)
PIN_NTC_B = 28        # GP28 / ADC2 -> /NTC_B      (TH2 Chamber B 10k NTC)

# ==============================================================================
# 2. PDR Flight States (Appendix A & Appendix D)
# ==============================================================================
STATE_SELF_TEST = 0
STATE_PAD = 1
STATE_BOOST = 2
STATE_DESCENT = 3
STATE_LANDED = 4

STATE_NAMES = {
    STATE_SELF_TEST: "SELF_TEST",
    STATE_PAD: "PAD",
    STATE_BOOST: "BOOST",
    STATE_DESCENT: "DESCENT",
    STATE_LANDED: "LANDED",
}

# 34-byte little-endian binary telemetry frame format (PDR Appendix D):
#   0..1   pkt_num            uint16 (H)
#   2      state              uint8  (B)
#   3..6   mission_time_ms    uint32 (I)
#   7..10  pressure_pa        uint32 (I)
#   11..12 air_temp_c_x100    int16  (h)
#   13..14 chamber_a_temp_x100 int16 (h)
#   15..16 chamber_b_temp_x100 int16 (h)
#   17..20 light_a            uint32 (I)
#   21..24 light_b            uint32 (I)
#   25..26 peak_accel_g_x10   uint16 (H)
#   27..28 rms_jerk_gps       uint16 (H)
#   29..30 battery_mv         uint16 (H)
#   31     heater_duty_pct    uint8  (B)
#   32..33 crc16              uint16 (H)
TELEM_STRUCT_NO_CRC = "<HBIIhhhIIHHHB"
TELEM_STRUCT_FULL = "<HBIIhhhIIHHHBH"


def crc16_ccitt(data_bytes, poly=0x1021, init_val=0xFFFF):
    """Compute 16-bit CRC-CCITT over data_bytes (bytes 0..31 of the 34-byte frame)."""
    crc = init_val
    for b in data_bytes:
        crc ^= (b & 0xFF) << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ poly) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def pack_telemetry_frame(
    pkt_num,
    state,
    mission_time_ms,
    pressure_pa,
    air_temp_c,
    chamber_a_c,
    chamber_b_c,
    light_a_counts,
    light_b_counts,
    peak_accel_g,
    rms_jerk_gps,
    vbat_v,
    heater_pct,
):
    """Pack the 34-byte little-endian binary telemetry frame with CRC-16 (PDR Appendix D)."""
    body = struct.pack(
        TELEM_STRUCT_NO_CRC,
        pkt_num & 0xFFFF,
        state & 0xFF,
        mission_time_ms & 0xFFFFFFFF,
        max(0, int(round(pressure_pa))),
        max(-32768, min(32767, int(round(air_temp_c * 100.0)))),
        max(-32768, min(32767, int(round(chamber_a_c * 100.0)))),
        max(-32768, min(32767, int(round(chamber_b_c * 100.0)))),
        max(0, int(light_a_counts)) & 0xFFFFFFFF,
        max(0, int(light_b_counts)) & 0xFFFFFFFF,
        max(0, min(65535, int(round(peak_accel_g * 10.0)))),
        max(0, min(65535, int(round(rms_jerk_gps)))),
        max(0, min(65535, int(round(vbat_v * 1000.0)))),
        max(0, min(100, int(round(heater_pct)))),
    )
    crc = crc16_ccitt(body)
    return body + struct.pack("<H", crc)


class HeaterPID:
    """
    PI-D thermal controller for Chamber A 15-Ohm 2W heater.
    Target setpoint: 22.0 +/- 1.0 C (PDR Section 2.1 & Requirement S2).
    Safety cut-outs: battery < 3.50 V or Chamber A >= 30.0 C.
    """

    def __init__(self, setpoint_c=22.0, kp=20.0, ki=0.9, kd=4.0):
        self.setpoint = setpoint_c
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_err = 0.0

    def update(self, temp_c, dt_s, vbat_v):
        if vbat_v < 3.50 or temp_c >= 30.0:
            self.integral = 0.0
            return 0.0
        err = self.setpoint - temp_c
        if err <= 0.0:
            self.integral = max(0.0, self.integral + err * dt_s)
            self.prev_err = err
            return 0.0
        self.integral = max(0.0, min(40.0, self.integral + err * dt_s))
        deriv = (err - self.prev_err) / max(0.05, dt_s)
        self.prev_err = err
        out_pct = self.kp * err + self.ki * self.integral + self.kd * deriv
        return max(0.0, min(100.0, out_pct))


def release_landing_legs(servo_pwr_pin, servo_pwm, angle_deg=120.0, hold_ms=700):
    """
    Release the 3 spring-loaded curved landing legs in DESCENT before touchdown
    (at 150 m above pad or 15 s post-apogee). Powers MG90S via Q2 AO3400A low-side
    switch for hold_ms, then cuts power so idle servo current is 0 mA.
    """
    duty_us = int(500 + (max(0.0, min(180.0, angle_deg)) / 180.0) * 2000)
    servo_pwr_pin.value(1)
    servo_pwm.duty_ns(duty_us * 1000)
    time.sleep_ms(hold_ms)
    servo_pwm.duty_ns(0)
    servo_pwr_pin.value(0)


def main():
    led = Pin(PIN_STATUS_LED, Pin.OUT, value=1)
    uart = UART(0, baudrate=115200, tx=Pin(PIN_UART_TX), rx=Pin(PIN_UART_RX))

    # Heater PWM on GP8 (1 kHz)
    heater_pwm = PWM(Pin(PIN_HEATER_PWM))
    heater_pwm.freq(1000)
    heater_pwm.duty_u16(0)

    # Servo power gate (GP9) & 50 Hz PWM (GP10) - strictly OFF at boot
    servo_pwr = Pin(PIN_SERVO_PWR, Pin.OUT, value=0)
    servo_pwm = PWM(Pin(PIN_SERVO_PWM))
    servo_pwm.freq(50)
    servo_pwm.duty_ns(0)

    # Analog channels (ADC0=VBAT, ADC1=NTC_A, ADC2=NTC_B)
    adc_vbat = ADC(Pin(PIN_BATT_SENSE))
    adc_ntc_a = ADC(Pin(PIN_NTC_A))
    adc_ntc_b = ADC(Pin(PIN_NTC_B))

    # SPI1 for BMP280 (U6) and ADXL375 (U4)
    cs_press = Pin(PIN_PRESS_CS, Pin.OUT, value=1)
    cs_imu = Pin(PIN_IMU_CS, Pin.OUT, value=1)
    Pin(PIN_IMU_INT1, Pin.IN, Pin.PULL_DOWN)
    spi1 = SPI(
        1,
        baudrate=2_000_000,
        polarity=1,
        phase=1,
        sck=Pin(PIN_SPI1_SCLK),
        mosi=Pin(PIN_SPI1_MOSI),
        miso=Pin(PIN_SPI1_MISO),
    )

    # State 0: SELF_TEST
    state = STATE_SELF_TEST
    bmp = None
    imu = None
    try:
        bmp = BMP280_SPI(spi1, cs_press)
    except Exception as e:
        print("BMP280 init warning:", e)

    try:
        imu = ADXL375_SPI(spi1, cs_imu, shock_threshold_g=12.0)
    except Exception as e:
        print("ADXL375 init warning:", e)

    # I2C0 (Chamber A TSL2591 U7) & I2C1 (Chamber B TSL2591 U8)
    i2c0 = I2C(0, scl=Pin(PIN_I2C0_SCL), sda=Pin(PIN_I2C0_SDA), freq=400_000)
    i2c1 = I2C(1, scl=Pin(PIN_I2C1_SCL), sda=Pin(PIN_I2C1_SDA), freq=400_000)
    Pin(PIN_INT_A, Pin.IN, Pin.PULL_UP)
    Pin(PIN_INT_B, Pin.IN, Pin.PULL_UP)

    lux_a = None
    lux_b = None
    try:
        lux_a = TSL2591_I2C(i2c0)
    except Exception as e:
        print("TSL2591 Chamber A init warning:", e)

    try:
        lux_b = TSL2591_I2C(i2c1)
    except Exception as e:
        print("TSL2591 Chamber B init warning:", e)

    # SPI0 for Ai-Thinker Ra-02 SX1278 LoRa (U5) at 433.175 MHz (SF8, BW125, CR4/5)
    spi0 = SPI(
        0,
        baudrate=4_000_000,
        polarity=0,
        phase=0,
        sck=Pin(PIN_RA_SCLK),
        mosi=Pin(PIN_RA_MOSI),
        miso=Pin(PIN_RA_MISO),
    )
    lora = None
    try:
        lora = SX1278(
            spi0,
            cs=Pin(PIN_RA_CS, Pin.OUT, value=1),
            rst=Pin(PIN_RA_RESET, Pin.OUT, value=1),
            dio0=Pin(PIN_RA_DIO0, Pin.IN),
            freq_mhz=433.175,
            tx_power_dbm=17,
        )
    except Exception as e:
        print("SX1278 LoRa init warning:", e)

    # Calibrate ground baseline altitude (alt_0)
    ground_alt_m = 0.0
    if bmp is not None:
        _, _, ground_alt_m = bmp.read()

    pid = HeaterPID(setpoint_c=22.0)
    # Self-test complete -> transition to State 1: PAD
    state = STATE_PAD

    alt_max_m = 0.0
    prev_alt_m = 0.0
    t_apogee_ms = 0
    legs_deployed = False
    landed_stable_s = 0.0

    # 5-second RAM ring buffer (25 entries at 5 Hz loop rate) for pre-launch capture
    ring_buffer = []
    RING_BUFFER_MAX = 25

    pkt_num = 0
    boot_ms = time.ticks_ms()
    prev_ms = boot_ms
    last_tx_ms = boot_ms
    prev_amag_g = 1.0
    window_peak_g = 1.0
    jerk_sq_sum = 0.0
    jerk_samples = 0

    log_file = open("flight_log.csv", "a")
    log_file.write(
        "pkt_num,state,mission_time_ms,pressure_pa,air_temp_c,chamber_a_c,chamber_b_c,"
        "light_a,light_b,peak_accel_g,rms_jerk_gps,battery_mv,heater_duty_pct,"
        "rel_alt_m,legs_deployed,crc16_hex\n"
    )

    while True:
        now_ms = time.ticks_ms()
        mission_time_ms = time.ticks_diff(now_ms, boot_ms)
        dt_s = max(0.02, time.ticks_diff(now_ms, prev_ms) / 1000.0)
        prev_ms = now_ms

        vbat_v = read_battery_voltage(adc_vbat)
        ntc_a_c = read_ntc_temp_c(adc_ntc_a)
        ntc_b_c = read_ntc_temp_c(adc_ntc_b)

        if bmp is not None:
            temp_bmp_c, press_hpa, abs_alt_m = bmp.read()
            pressure_pa = press_hpa * 100.0
            rel_alt_m = abs_alt_m - ground_alt_m
        else:
            temp_bmp_c, pressure_pa, rel_alt_m = ntc_b_c, 101325.0, 0.0

        vz_mps = (rel_alt_m - prev_alt_m) / dt_s
        prev_alt_m = rel_alt_m

        if imu is not None:
            _, _, _, amag_g = imu.read_accel_g()
        else:
            amag_g = 1.0

        if amag_g > window_peak_g:
            window_peak_g = amag_g
        jerk_gps = abs(amag_g - prev_amag_g) / dt_s
        prev_amag_g = amag_g
        jerk_sq_sum += jerk_gps * jerk_gps
        jerk_samples += 1
        rms_jerk_gps = math.sqrt(jerk_sq_sum / max(1, jerk_samples))

        light_a_counts = lux_a.read_raw_and_lux()[2] if lux_a is not None else 0
        light_b_counts = lux_b.read_raw_and_lux()[2] if lux_b is not None else 0

        # Update Chamber A thermal PID (22.0 +/- 1.0 C, cut-out at <3.5 V or >30 C)
        heater_pct = pid.update(ntc_a_c, dt_s, vbat_v)
        heater_pwm.duty_u16(int((heater_pct / 100.0) * 65535))

        # Track maximum relative altitude
        if rel_alt_m > alt_max_m:
            alt_max_m = rel_alt_m

        # ======================================================================
        # PDR Flight State Machine Transitions (Appendix A)
        # ======================================================================
        if state == STATE_PAD:
            # Servo strictly locked out on PAD
            servo_pwr.value(0)
            if rel_alt_m > 25.0 or amag_g > 4.0:
                state = STATE_BOOST
                # Flush 5-second pre-launch RAM ring buffer to flash
                for buffered_line in ring_buffer:
                    log_file.write(buffered_line + "\n")
                ring_buffer.clear()
                log_file.flush()

        elif state == STATE_BOOST:
            # Servo strictly locked out during BOOST
            servo_pwr.value(0)
            if rel_alt_m < (alt_max_m - 5.0):
                state = STATE_DESCENT
                t_apogee_ms = now_ms

        elif state == STATE_DESCENT:
            # Pre-touchdown leg release: < 150 m above pad OR >= 15 s after apogee
            if not legs_deployed:
                post_apogee_ms = time.ticks_diff(now_ms, t_apogee_ms)
                if rel_alt_m < 150.0 or post_apogee_ms >= 15_000:
                    release_landing_legs(servo_pwr, servo_pwm, angle_deg=120.0, hold_ms=700)
                    legs_deployed = True

            # Touchdown detection: |vz| < 0.4 m/s for 5 s
            if abs(vz_mps) < 0.4 and rel_alt_m < 30.0:
                landed_stable_s += dt_s
                if landed_stable_s >= 5.0:
                    state = STATE_LANDED
            else:
                landed_stable_s = 0.0

        elif state == STATE_LANDED:
            # Ensure legs are deployed if DESCENT was very brief in a low-altitude drop test
            if not legs_deployed:
                release_landing_legs(servo_pwr, servo_pwm, angle_deg=120.0, hold_ms=700)
                legs_deployed = True

        # Pack 34-byte binary frame with CRC-16
        frame_bytes = pack_telemetry_frame(
            pkt_num,
            state,
            mission_time_ms,
            pressure_pa,
            temp_bmp_c,
            ntc_a_c,
            ntc_b_c,
            light_a_counts,
            light_b_counts,
            window_peak_g,
            rms_jerk_gps,
            vbat_v,
            heater_pct,
        )
        crc_val = struct.unpack_from("<H", frame_bytes, 32)[0]

        csv_line = (
            f"{pkt_num},{STATE_NAMES[state]},{mission_time_ms},{int(round(pressure_pa))},"
            f"{temp_bmp_c:.2f},{ntc_a_c:.2f},{ntc_b_c:.2f},"
            f"{light_a_counts},{light_b_counts},{window_peak_g:.1f},{int(round(rms_jerk_gps))},"
            f"{int(round(vbat_v * 1000.0))},{int(round(heater_pct))},"
            f"{rel_alt_m:.1f},{int(legs_deployed)},0x{crc_val:04X}"
        )

        if state == STATE_PAD:
            ring_buffer.append(csv_line)
            if len(ring_buffer) > RING_BUFFER_MAX:
                ring_buffer.pop(0)
        else:
            log_file.write(csv_line + "\n")
            if pkt_num % 5 == 0:
                log_file.flush()

        # Transmit 34-byte binary LoRa frame at 1 Hz (PDR Section 2.1)
        if time.ticks_diff(now_ms, last_tx_ms) >= 1000:
            last_tx_ms = now_ms
            pkt_num = (pkt_num + 1) & 0xFFFF
            uart.write(csv_line + "\r\n")
            if lora is not None:
                lora.send_packet(frame_bytes)
            window_peak_g = amag_g
            jerk_sq_sum = 0.0
            jerk_samples = 0

        led.toggle()
        time.sleep_ms(200)


if __name__ == "__main__":
    main()
