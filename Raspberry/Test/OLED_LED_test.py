#!/usr/bin/env python3

import rospy
from PIL import Image, ImageDraw, ImageFont
import Adafruit_SSD1306
import drivers


# Initialize the OLED display
RST = None
oled_i2c_address = 0x3c  # OLED I2C address
oled = Adafruit_SSD1306.SSD1306_128_64(rst=RST)
oled.begin()
oled.clear()
oled.display()

# Initialize the LCD display
display = drivers.Lcd()
# Create an image buffer for OLED
width = 128
height = 64
image = Image.new('1', (width, height))
draw = ImageDraw.Draw(image)

# Load a font
font = ImageFont.load_default()

def print_hello_world():
    # Clear the OLED image
    draw.rectangle((20, 20, width, height), outline=0, fill=0)
    # Draw the text on OLED
    draw.text((20, 20), "Hello World!", font=font, fill=255)
    # Display the image on OLED
    oled.image(image)
    oled.display()

    # Clear the LCD
    display.lcd_clear()
    # Print text on LCD
    display.lcd_display_string('Hello World!\nfrom ROS Node',1)

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('oled_lcd_hello_world_node', anonymous=True)

        # Print "Hello World" to both OLED and LCD screens
        print_hello_world()

        # Keep the node running
        rospy.loginfo("Hello World displayed on OLED and LCD screens.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
