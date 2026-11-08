"""
CanSat Team Verticality - Flight Controller & Bioluminescence Payload Firmware
Target MCU: Raspberry Pi RP2040 (U1) + W25Q128JVS 16MB QSPI Flash (U2)
Mission:
  - Maintain Chamber A (experimental Pyrocystis lunula dinoflagellate culture) at 20.0 C
    using closed-loop PID control on a 15 Ohm 2W heating resistor (R8 via Q1 AO3400A on GP8),
    while Chamber B remains an unheated ambient flight control.
  - Measure bioluminescence emission intensity in both dark chambers using dual TSL2591
    sensors (U7 on I2C0 GP4/GP5, U8 on I2C1 GP6/GP7) at 9876x gain.
  - Track ascent, apogee, descent rate, and high-G shock events via BMP280 (SPI1 CS=GP11)
    and ADXL375 +-200g accelerometer (SPI1 CS=GP13, INT1=GP22).
  - Actuate MG90S landing-leg / mechanism release servo (J6) via low-side MOSFET power gate
    (Q2 AO3400A on GP9) and 50 Hz PWM (GP10).
  - Log full-rate CSV telemetry to onboard 16 MB QSPI flash and transmit 1 Hz LoRa
    packets at 433.0 MHz via Ai-Thinker Ra-02 (SX1278 on SPI0 GP16..GP21).
"""

import time
from machine import ADC, I2C, PWM, SPI, Pin, UART
from sensors import ADXL375_SPI, BMP280_SPI, TSL2591_I2C, read_battery_voltage, read_ntc_temp_c
from sx1278 import SX1278

# ==============================================================================
# 1.Exact Schematic Pin Definitions (CanSat.kicad_sch)
# ==============================================================================
PIN_UART_TX = 0       # GP0 -> /RX (TP4 UART debug)
PIN_UART_RX = 1       # GP1 -> /TX (TP5 UART debug)

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
# 2. Flight State Machine & Thermal PID Controller
# ==============================================================================
STATE_PREFLIGHT = 0
STATE_ASCENT = 1
STATE_DESCENT = 2
STATE_LANDED_DEPLOY = 3
STATE_BIOLUM_OPS = 4

STATE_NAMES = {
    STATE_PREFLIGHT: "PREFLIGHT",
    STATE_ASCENT: "ASCENT",
    STATE_DESCENT: "DESCENT",
    STATE_LANDED_DEPLOY: "DEPLOY",
    STATE_BIOLUM_OPS: "BIOLUM",
}


class HeaterPID:
    """PI-D thermal controller for Chamber A 15-Ohm 2W heater (target 20.0 C)."""

    def __init__(self, setpoint_c=20.0, kp=18.0, ki=0.8, kd=4.0):
        self.setpoint = setpoint_c
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_err = 0.0

    def update(self, temp_c, dt_s, vbat_v):
        # Protect Li-Ion battery from deep discharge (< 3.35 V) or thermal runaway (> 26 C)
        if vbat_v < 3.35 or temp_c >= 26.0:
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


def set_servo_angle(servo_pwr_pin, servo_pwm, angle_deg, hold_ms=600):
    """Power on MG90S via Q2 low-side switch, command PWM angle, then cut power to save battery."""
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

    # Servo power gate (GP9) & 50 Hz PWM (GP10)
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
    imu_int1 = Pin(PIN_IMU_INT1, Pin.IN, Pin.PULL_DOWN)
    spi1 = SPI(
        1,
        baudrate=2_000_000,
        polarity=1,
        phase=1,
        sck=Pin(PIN_SPI1_SCLK),
        mosi=Pin(PIN_SPI1_MOSI),
        miso=Pin(PIN_SPI1_MISO),
    )

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

    # SPI0 for Ai-Thinker Ra-02 SX1278 LoRa (U5)
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
            freq_mhz=433.0,
            tx_power_dbm=17,
        )
    except Exception as e:
        print("SX1278 LoRa init warning:", e)

    # Baseline ground altitude calibration
    ground_alt_m = 0.0
    if bmp is not None:
        _, _, ground_alt_m = bmp.read()

    pid = HeaterPID(setpoint_c=20.0)
    state = STATE_PREFLIGHT
    max_rel_alt_m = 0.0
    landed_stable_count = 0
    pkt_id = 0
    prev_ms = time.ticks_ms()

    log_file = open("flight_log.csv", "a")
    log_file.write(
        "pkt,ms,state,vbat_v,temp_bmp_c,press_hpa,rel_alt_m,ax_g,ay_g,az_g,amag_g,"
        "ntc_a_c,ntc_b_c,heater_pct,lux_a_mlux,lux_b_mlux\n"
    )

    while True:
        now_ms = time.ticks_ms()
        dt_s = max(0.05, time.ticks_diff(now_ms, prev_ms) / 1000.0)
        prev_ms = now_ms

        vbat_v = read_battery_voltage(adc_vbat)
        ntc_a_c = read_ntc_temp_c(adc_ntc_a)
        ntc_b_c = read_ntc_temp_c(adc_ntc_b)

        if bmp is not None:
            temp_bmp_c, press_hpa, abs_alt_m = bmp.read()
            rel_alt_m = abs_alt_m - ground_alt_m
        else:
            temp_bmp_c, press_hpa, rel_alt_m = ntc_b_c, 1013.25, 0.0

        if imu is not None:
            ax_g, ay_g, az_g, amag_g = imu.read_accel_g()
        else:
            ax_g, ay_g, az_g, amag_g = 0.0, 0.0, 1.0, 1.0

        mlux_a = lux_a.read_raw_and_lux()[3] if lux_a is not None else 0.0
        mlux_b = lux_b.read_raw_and_lux()[3] if lux_b is not None else 0.0

        # Update thermal PID for Chamber A
        heater_pct = pid.update(ntc_a_c, dt_s, vbat_v)
        heater_pwm.duty_u16(int((heater_pct / 100.0) * 65535))

        # Flight state machine transitions
        if rel_alt_m > max_rel_alt_m:
            max_rel_alt_m = rel_alt_m

        if state == STATE_PREFLIGHT:
            if rel_alt_m > 15.0 or amag_g > 3.0:
                state = STATE_ASCENT
        elif state == STATE_ASCENT:
            if (max_rel_alt_m - rel_alt_m) > 10.0:
                state = STATE_DESCENT
        elif state == STATE_DESCENT:
            if rel_alt_m < 25.0 and abs(amag_g - 1.0) < 0.35:
                landed_stable_count += 1
                if landed_stable_count >= 5 or imu_int1.value() == 1:
                    state = STATE_LANDED_DEPLOY
            else:
                landed_stable_count = 0
        elif state == STATE_LANDED_DEPLOY:
            # Release landing mechanism / legs via MG90S servo
            set_servo_angle(servo_pwr, servo_pwm, 120.0, hold_ms=700)
            state = STATE_BIOLUM_OPS

        pkt_id += 1
        csv_line = (
            f"{pkt_id},{now_ms},{STATE_NAMES[state]},{vbat_v:.2f},"
            f"{temp_bmp_c:.2f},{press_hpa:.2f},{rel_alt_m:.1f},"
            f"{ax_g:.2f},{ay_g:.2f},{az_g:.2f},{amag_g:.2f},"
            f"{ntc_a_c:.2f},{ntc_b_c:.2f},{heater_pct:.1f},"
            f"{mlux_a:.2f},{mlux_b:.2f}"
        )

        log_file.write(csv_line + "\n")
        if pkt_id % 5 == 0:
            log_file.flush()

        uart.write(csv_line + "\r\n")
        if lora is not None:
            lora.send_packet(csv_line)

        led.toggle()
        time.sleep_ms(200)


if __name__ == "__main__":
    main()
