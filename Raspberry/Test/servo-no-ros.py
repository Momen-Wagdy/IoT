import board
import busio
import time
import adafruit_pca9685

i2c = busio.I2C(board.SCL, board.SDA)
pca = adafruit_pca9685.PCA9685(i2c, address=0x40)
pca.frequency = 50

channel = pca.channels[0]
channel2 = pca.channels[1]

# Function to stop (lock) the servo
def lock_servo():
    neutral_duty_cycle = 0x8000  # Adjust this value if needed
    channel.duty_cycle = neutral_duty_cycle
    channel2.duty_cycle = 0x1477

while True:
    # Spin
    channel.duty_cycle = 0x1200
    channel2.duty_cycle = 0x0A00
    print("Spinning")
    time.sleep(0.49)
    
    # Lock (stop) the servo
    print("Locking (Stopping)")
    lock_servo()
    time.sleep(1)
    
    # Spin opposite direction
    print("Spinning opposite")
    channel.duty_cycle = 0x17FF
    channel2.duty_cycle = 0x1F00
    time.sleep(0.49)
    
    # Lock (stop) again
    lock_servo()
    time.sleep(1)
