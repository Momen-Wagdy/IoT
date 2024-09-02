#!/usr/bin/env python3

import rospy
from PIL import Image, ImageDraw, ImageFont
import Adafruit_SSD1306

# Initialize the OLED display
RST = None
i2c_address = 0x3C
oled = Adafruit_SSD1306.SSD1306_128_64(rst=RST)
oled.begin()
oled.clear()
oled.display()

# Create an image buffer
width = 128
height = 64
image = Image.new('1', (width, height))
draw = ImageDraw.Draw(image)

# Load a font
font = ImageFont.load_default()

def print_hello_world():
    # Clear the image
    draw.rectangle((0, 0, width, height), outline=0, fill=0)
    # Draw the text
    draw.text((0, 0), "Hello World!", font=font, fill=255)
    # Display the image
    oled.image(image)
    oled.display()

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('oled_hello_world_node', anonymous=True)
        
        # Print "Hello World" to the OLED screen
        print_hello_world()
        
        # Keep the node running
        rospy.loginfo("Hello World displayed on OLED screen.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
