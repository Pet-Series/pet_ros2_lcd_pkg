#!/usr/bin/env python3
# coding = utf-8
########################################################################################
## (c) https://github.com/Pet-Series
##     https://github.com/Pet-Series/pet_ros2_lcd_pkg
##
## Maintainer: stefan.kull@gmail.com
## The MIT License (MIT)
##
## Input:  Subscribe ROS2-topics
## Output: To LCD-display via I2C
##
## Behaviour:
##   1) Once: Read/Set all the parameters
##   2) Repeatedly: Subscribe ROS2-topics as text string
##   3) Repeatedly: Update corresponding row with text string on the LCD-display
##
## Prerequisite: Lunux/Ubuntu vs. Hardwere
##   Hardware/Display: LCD-display with an I2C "piggy back" I2C->Parallell converter (PC8574). Default I2C adr.=0x3F
##   Hardware/SBC    : Raspberry Pi 4(Ubuntu) with I2C
##
## Prerequisite: Linux/Ubuntu vs. Software
##   $ sudo apt install i2c-tools
##   $ sudo i2cdetect -y 1          <- Normaly 0x27 or 0x3F
##   $ sudo chmod a+rw /dev/i2c-1   <- Give members of user/group i2c r/w to i2c interface.
##
## Prerequisite: Linux/Ubuntu vs. Software
##   $ sudo apt install python3-pip
##   $ sudo pip3 install smbus2
##   $ sudo pip3 install smbus
##
## Launch sequence:
##   1) $ ros2 run pet_ros2_lcd_pkg lcd_node
##   2) $ ros2 topic pub /lcd_display/row3 std_msgs/msg/String "data: Text on row 3" -1
##

#  Include the ROS2 stuff...
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor

#  Include Linux/Ubuntu stuff...
import sys
import signal
from std_msgs.msg import String
from time import *

# Include library so that we can use the I2C-port on the Raspberry Pi
from smbus2 import SMBus

# Load LCD-driver utility files in directory ./util/*.py (see also setup.py)
import util.lcddriver   
import util.i2c_lib
LCD_DISPLAY_WIDTH = 20

# TODO: Add the following "behaviours to class "LcdDisplayNode"
# lcd.lcd_clear()
# lcd.lcd_backlight("ON")
# lcd.lcd_backlight("OFF)

class LcdDisplayNode(Node): 
    """ 
    Class to control the '16x2'/'20x4' I2C LCD display from ROS2 on a Raspberry Pi.
    Adopted for ROS2
    """     
    def __init__(self):
        super().__init__("pet_lcd_driver_node")
        
        # Set default Display-I2C address. Accessed via ROS Parameters...
        self.declare_parameter( 'lcd_i2c_address', "0x3F", ParameterDescriptor(description='LCD-display I2C address [default "0x3F"]') )
        self.LCD_I2C_ADDRESS_STR = self.get_parameter('lcd_i2c_address').get_parameter_value().string_value
        self.LCD_I2C_ADDRESS = int(self.LCD_I2C_ADDRESS_STR, 16)  # Convert Hex-string to Int

        # If topics ('lcd_display/row#') content update -> Then ...callback 
        self.subscription_row1 = self.create_subscription(String, 'lcd_display/row1', self.lcd_update_row1_callback, 10)
        self.subscription_row2 = self.create_subscription(String, 'lcd_display/row2', self.lcd_update_row2_callback, 10)
        self.subscription_row3 = self.create_subscription(String, 'lcd_display/row3', self.lcd_update_row3_callback, 10)
        self.subscription_row4 = self.create_subscription(String, 'lcd_display/row4', self.lcd_update_row4_callback, 10)

        # prevent 'unused variable' warning
        self.subscription_row1
        self.subscription_row2
        self.subscription_row3
        self.subscription_row4

        exit = False
        # Check we can open/contact the I2C-controller on the Display.
        try:
            # Initiate the LCD Display via I2C interface (include ./util/*.py)
            self.lcd = util.lcddriver.lcd(self.LCD_I2C_ADDRESS)

            # Some dummy text in the display
            self.lcd.display_string( "pet_lcd_driver_node", 4)
            self.lcd.display_string( "Initialization", 1)
            
            # Some basic information on the console
            self.get_logger().info("| pet_lcd_driver_node has started")
            self.get_logger().info("| I2C: <" + str(self.LCD_I2C_ADDRESS) + ">" )


        except:
            # Note: a permission error can be fixed with a "sudo chmod a+rw /dev/i2c-1"
            self.get_logger().error("pet_lcd_driver_node canceled:"  + str(sys.exc_info()[1]) )
            self.exit = True
        
    def lcd_update_row1_callback(self, msg):
        """ 
        Callback when topic 'lcd_display/row1' is updated -> Time to publish on the LCD-display. 
        + Right padding by adding ' '->row-width. Overwrites any garbage characters on the right
        """ 
        self.lcd.display_string( msg.data.ljust(LCD_DISPLAY_WIDTH,' '), 1) 
        self.get_logger().info("LCD-row 1.= '" + msg.data + "'" )

    def lcd_update_row2_callback(self, msg):
        """ 
        Callback when topic 'lcd_display/row2' is updated -> Time to publish on the LCD-display. 
        + Right padding by adding ' '->row-width. Overwrites any garbage characters on the right
        """ 
        self.lcd.display_string( msg.data.ljust(LCD_DISPLAY_WIDTH,' '), 2) 
        self.get_logger().info("LCD-row 2.= '" + msg.data + "'" )

    def lcd_update_row3_callback(self, msg):
        """ 
        Callback when topic 'lcd_display/row3' is updated -> Time to publish on the LCD-display. 
        + Right padding by adding ' '->row-width. Overwrites any garbage characters on the right
        """
        self.lcd.display_string( msg.data.ljust(LCD_DISPLAY_WIDTH,' '), 3) 
        self.get_logger().info("LCD-row 3.= '" + msg.data + "'" )

    def lcd_update_row4_callback(self, msg):
        """ 
        Callback when topic 'lcd_display/row4' is updated -> Time to publish on the LCD-display. 
        + Right padding by adding ' '->row-width. Overwrites any garbage characters on the right
        """
        self.lcd.display_string( msg.data.ljust(LCD_DISPLAY_WIDTH,' '), 4) 
        self.get_logger().info("LCD-row 4.= '" + msg.data + "'" )

def main(args=None):
    """ 
    ROS2 entrypoint. Se also setup.py
    """ 
    rclpy.init(args=args)
    pet_lcd_driver_node = LcdDisplayNode()

    try:
        rclpy.spin(pet_lcd_driver_node)

    except KeyboardInterrupt:
        print("**** ☠️ Ctrl-C detected...")
    
    finally:
        print("**** pet_lcd_driver_node ending... ")
        print( str(sys.exc_info()[1]) )
        
        # Time to clean up stuff!
        # - Destroy the node explicitly
        #   (optional - otherwise it will be done automatically
        #   when the garbage collector destroys the node object)
        pet_lcd_driver_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()