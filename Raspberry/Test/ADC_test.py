import RPi.GPIO as GPIO
import time
import threading

# Setup GPIO mode
GPIO.setmode(GPIO.BCM)

# Define GPIO pins
data_pins = [8, 18, 23, 24, 25, 16, 12, 7]  # D0 to D7
addr_pins = [13, 19, 26]  # A, B, C
start_pin = 6  # START
eoc_pin = 21  # EOC
oe_pin = 5  # OE
ale_pin = 22  # ALE
clk_pin = 20  # CLK

# Setup GPIO pins
GPIO.setup(data_pins, GPIO.IN)
GPIO.setup(addr_pins, GPIO.OUT)
GPIO.setup(start_pin, GPIO.OUT)
GPIO.setup(eoc_pin, GPIO.IN)
GPIO.setup(oe_pin, GPIO.OUT)
GPIO.setup(ale_pin, GPIO.OUT)
GPIO.setup(clk_pin, GPIO.OUT)

def generate_clock():
    while True:
        GPIO.output(clk_pin, GPIO.HIGH)
        time.sleep(0.000005)  # Adjust this value for the clock frequency
        GPIO.output(clk_pin, GPIO.LOW)
        time.sleep(0.000005)

def read_adc(channel):
    # Select the channel
    for i in range(3):
        GPIO.output(addr_pins[i], (channel >> i) & 1)
    
    # Latch the address
    GPIO.output(ale_pin, GPIO.HIGH)
    time.sleep(0.001)  # Small delay
    GPIO.output(ale_pin, GPIO.LOW)
    
    # Start conversion
    GPIO.output(start_pin, GPIO.HIGH)
    time.sleep(0.001)
    GPIO.output(start_pin, GPIO.LOW)
    
    print("Starting conversion.....")
    # Wait for end of conversion
    while GPIO.input(eoc_pin) == GPIO.HIGH:
        time.sleep(0.01)  # Small delay to avoid tight loop
    print("Conversion done.")
    
    # Enable output
    GPIO.output(oe_pin, GPIO.HIGH)
    
    # Read the 8-bit data
    value = 0
    for i in range(8):
        if GPIO.input(data_pins[i]):
            value |= (1 << i)
    
    # Disable output
    GPIO.output(oe_pin, GPIO.LOW)
    
    return value

try:
    # Start the clock generation in a separate thread
    clock_thread = threading.Thread(target=generate_clock)
    clock_thread.start()
    
    while True:
        for ch in range(8):
            result = read_adc(ch)
            vout = (result*5 / 175)
            vin = vout*(7500/(30000+7500))*5
            print(f"Channel {ch}: {vin}")
        time.sleep(1)
finally:
    GPIO.cleanup()
