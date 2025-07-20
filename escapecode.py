# Programme for Disabled Submarine Escape Time Calculation Device

# Import Libraries
import time
import threading
from smbus2 import SMBus
from gpiozero import Button
from grove.adc import ADC
import board
import busio
import adafruit_scd4x
import pandas as pd

# Setup
LCD_ADDRESS = 0x3e
LCD_CMD = 0x00
LCD_DATA = 0x40
BUTTON_INC = 5
BUTTON_DEC = 16
BUTTON_SUBMIT = 12
O2_MIN_VOLTAGE = 0.2
O2_MAX_VOLTAGE = 2.9
VREF = 3.3
INPUT_FIELDS = [
    ("# OF SURVIVORS", "SURV"),
    ("# FIT TO ESCAPE", "FIT"),
    ("INT. PRESSURE", "PRESS"),
    ("ESCAPE DEPTH", "DEPTH"),
    ("% FLOODING", "FLOOD"),
    ("# OF LiOH CANS", "LiOH"),
    ("# OF CANDLES", "CANDLE"),
    ("SSA PRESSURE", "SSA")
]

# Oxygen Sensor Setup
class GroveMix8410:
    def __init__(self, channel=0, address=0x08):
        self.channel = channel
        self.adc = ADC(address=address)

    def read_voltage(self):
        value = self.adc.read(self.channel)
        return value * VREF / 1024.0

    def read_o2_percent(self):
        voltage = self.read_voltage()
        if voltage < O2_MIN_VOLTAGE:
            return 0.0
        elif voltage > O2_MAX_VOLTAGE:
            return 25.0
        else:
            return (voltage - O2_MIN_VOLTAGE) * (25.0 / (O2_MAX_VOLTAGE - O2_MIN_VOLTAGE))

# LCD Setup
def lcd_write_cmd(bus, cmd):
    bus.write_byte_data(LCD_ADDRESS, LCD_CMD, cmd)

def lcd_write_data(bus, data):
    bus.write_byte_data(LCD_ADDRESS, LCD_DATA, data)

def lcd_set_cursor(bus, col, row):
    row_offsets = [0x00, 0x40]
    lcd_write_cmd(bus, 0x80 | (col + row_offsets[row]))

def lcd_print(bus, message):
    for char in message.ljust(16):
        lcd_write_data(bus, ord(char))

def lcd_clear(bus):
    lcd_write_cmd(bus, 0x01)
    time.sleep(0.002)

def lcd_init(bus):
    time.sleep(0.05)
    for cmd in [0x38, 0x39, 0x14, 0x70, 0x56, 0x6c]:
        lcd_write_cmd(bus, cmd)
    time.sleep(0.2)
    for cmd in [0x38, 0x0C, 0x01]:
        lcd_write_cmd(bus, cmd)
    time.sleep(0.002)

# LCD Inputs
input_data = {}
current_field_index = 0
number = 0
hold_threads = {}
lock = threading.Lock()

def update_display_field(bus):
    lcd_clear(bus)
    lcd_set_cursor(bus, 0, 0)
    lcd_print(bus, INPUT_FIELDS[current_field_index][0])
    lcd_set_cursor(bus, 0, 1)
    lcd_print(bus, str(number))

def inc():
    global number
    number += 1

def dec():
    global number
    if number > 0:
        number -= 1

def start_repeating(button_name, modify_fn, bus):
    def repeat_thread():
        time.sleep(1.0)
        while hold_threads.get(button_name, False):
            with lock:
                modify_fn()
            update_display_field(bus)
            time.sleep(0.1)

    with lock:
        modify_fn()
    update_display_field(bus)

    hold_threads[button_name] = True
    t = threading.Thread(target=repeat_thread, daemon=True)
    t.start()

def stop_repeating(button_name):
    hold_threads[button_name] = False

def submit_pressed(bus, scd4x, o2_sensor):
    global current_field_index, number
    key = INPUT_FIELDS[current_field_index][1]
    input_data[key] = str(number)
    number = 0
    current_field_index += 1
    if current_field_index >= len(INPUT_FIELDS):
        lcd_clear(bus)
        lcd_print(bus, "**CALCULATING**")

        try:
            scd4x.stop_periodic_measurement()
        except Exception:
            pass

        scd4x.start_periodic_measurement()
        time.sleep(5)
        if scd4x.data_ready:
            input_data["CO2"] = round(scd4x.CO2 / 10000.0, 4)
            input_data["O2"] = round(o2_sensor.read_o2_percent(), 2)
        run_calculations(bus)
    else:
        update_display_field(bus)

def run_calculations(bus):
    try:
        SURV = int(input_data['SURV'])
        FIT = int(input_data['FIT'])
        PRESS = int(input_data['PRESS'])
        DEPTH = int(input_data['DEPTH'])
        FLOOD = int(input_data['FLOOD'])
        CO2 = float(input_data['CO2'])
        O2 = float(input_data['O2'])
        LiOH = int(input_data['LiOH'])
        CANDLE = int(input_data['CANDLE'])
        SSA = int(input_data['SSA'])

        if FIT == 0:
            lcd_clear(bus)
            lcd_print(bus, "Error: FIT=0")
            print("Invalid input: FIT cannot be zero.")
            return

        FITRound = 5 * round(FIT / 5)
        NOT_FIT = SURV - FIT
        NOT_FITRound = 10 * round(NOT_FIT / 10)

        df = pd.read_excel('Look_Up.xlsx')
        row = df['NOT_FIT'] == f'N{NOT_FITRound}'

        if f'F{FITRound}' not in df.columns:
            lcd_clear(bus)
            lcd_print(bus, f"Missing col F{FITRound}")
            print(f"Lookup error: Column 'F{FITRound}' not found in Excel.")
            return

        CELL = df.loc[row, f'F{FITRound}']
        G = CELL.squeeze()

        A = (PRESS / 33) + 1
        B = (DEPTH / 33) + 1
        C = ((100 - FLOOD) * A) * 798
        D = (((C) / (A)) - 1658) - (67.7 * (FIT - 2))
        E = 108 * A * FIT
        F = (C + E) / D
        ESC_PRESS = (F - 1) * 33
        H = (LiOH * 73) / SURV
        J = (((D) * 0.06) - ((CO2 * C) / 100) - (G * 0.85)) / (SURV * 0.85)
        WAIT_CO2 = H + J
        K = (((O2 * C) / 100) - (0.13 * D) - G) / SURV
        M = (CANDLE * 115) / SURV
        WAIT_O2 = K + M
        N = (A + F) * G * 10
        P = (C + E + N) / D
        R = ((E + N) * (A + P)) / 2
        S = (C + R) / D
        PRESS_EAB = (S - 1) * 33
        SSA_ESC = (((A + F) * E) / 30.5) + 350
        SSA_EAB = (((A + P) * R) / 30.5) + 350

        results = [
            ("CO2 CONCENTRTN", f"  = {CO2:.4f}% SEV"),
            ("O2 CONCENTRTN", f"  = {O2:.2f}% SEV"),
            ("INT. PRESSURE", f"A = {A:.2f} ATA"),
            ("ESCAPE DEPTH", f"B = {B:.2f} ATA"),
            ("AIR VOLUME", f"C = {C:.0f} FT^3"),
            ("BREATHABLE VOL.", f"D = {D:.0f} FT^3"),
            ("AIR ADDED", f"E = {E:.0f} FT^3"),
            ("POST ESC. PRESS.", f"F = {F:.2f} ATA"),
            ("POST ESC. PRESS.", f"  = {ESC_PRESS:.2f} FSW"),
            ("MAN-HOURS WAIT.", f"G = {G:.2f} HRS"),
            ("PRE LiOH TIME", f"H = {H:.2f} HRS"),
            ("POST LiOH WAIT", f"J = {J:.2f} HRS"),
            ("WAIT_CO2", f"  = {WAIT_CO2:.2f} HRS"),
            ("O2 TIME TO 13%", f"K = {K:.2f} HRS"),
            ("PRE CANDLE TIME", f"M = {M:.2f} HRS"),
            ("WAIT_O2", f"  = {WAIT_O2:.2f} HRS"),
            ("EAB AIR USED", f"N = {N:.0f} FT^3"),
            ("INT. PRESS. EAB", f"P = {P:.2f} ATA"),
            ("REV. SSA USE", f"R = {R:.0f} FT^3"),
            ("REV. INT. PRESS.", f"S = {S:.2f} ATA"),
            ("PRESS_EAB", f"  = {PRESS_EAB:.2f} FSW"),
            ("SSA W/O EAB", f"  = {SSA_ESC:.2f} PSI"),
            ("SSA W/ EAB", f"  = {SSA_EAB:.2f} PSI")
        ]

        scroll_results(bus, results)
    except Exception as e:
        lcd_clear(bus)
        lcd_print(bus, "Error in calc")
        print("Calculation error:", e)

def scroll_results(bus, results):
    index = 0

    def show():
        lcd_clear(bus)
        lcd_set_cursor(bus, 0, 0)
        lcd_print(bus, results[index][0])
        lcd_set_cursor(bus, 0, 1)
        lcd_print(bus, results[index][1])

    def scroll_up():
        nonlocal index
        index = (index - 1) % len(results)
        show()

    def scroll_down():
        nonlocal index
        index = (index + 1) % len(results)
        show()

    def restart():
        global current_field_index, input_data, number
        input_data.clear()
        current_field_index = 0
        number = 0
        button_up.when_pressed = lambda: start_repeating("inc", inc, bus)
        button_up.when_released = lambda: stop_repeating("inc")
        button_down.when_pressed = lambda: start_repeating("dec", dec, bus)
        button_down.when_released = lambda: stop_repeating("dec")
        button_submit.when_released = lambda: submit_pressed(bus, scd4x, o2_sensor)
        update_display_field(bus)

    show()
    button_up.when_pressed = scroll_up
    button_down.when_pressed = scroll_down
    button_submit.when_released = restart

bus = SMBus(1)
lcd_init(bus)
i2c = busio.I2C(board.SCL, board.SDA)
scd4x = adafruit_scd4x.SCD4X(i2c)
o2_sensor = GroveMix8410()

button_up = Button(BUTTON_INC, pull_up=True, bounce_time=0.1)
button_down = Button(BUTTON_DEC, pull_up=True, bounce_time=0.1)
button_submit = Button(BUTTON_SUBMIT, pull_up=True, bounce_time=0.1)

button_up.when_pressed = lambda: start_repeating("inc", inc, bus)
button_up.when_released = lambda: stop_repeating("inc")
button_down.when_pressed = lambda: start_repeating("dec", dec, bus)
button_down.when_released = lambda: stop_repeating("dec")
button_submit.when_released = lambda: submit_pressed(bus, scd4x, o2_sensor)

update_display_field(bus)

try:
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    lcd_clear(bus)
    lcd_print(bus, "Program Stopped")


