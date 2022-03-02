#!/usr/bin/env python3'
# coding = utf-8
########################################################################################
###
### Maintainer: stefan.kull@gmail.com
###
### Input:  Subscribe ROS2-topics
### Output: To LCD-display via I2C
###
### Behaviour:
### 1) Once: Read/Set all the parameters
### 2) Repeatedly: Subscribe ROS2-topics as text string
### 3) Repeatedly: Update corresponding row with text string on the LCD-display
###
### Prerequisite: Lunux/Ubuntu vs. Hardwere
### Hardware/Display: LCD-display with an I2C "piggy back" I2C->Parallell converter (PC8574A). Default I2C adr.=0x3F
### Hardware/SBC    : Raspberry Pi 4(Ubuntu) with I2C
### $ sudo apt install i2c-tools
### $ sudo i2cdetect -y 1          <- Normaly 0x27 or 0x3F
### $ sudo chmod a+rw /dev/i2c-1   <- Give members of user/group i2c r/w to i2c interface.
###
### Prerequisite: Linux/Ubuntu vs. Software
### $ sudo apt install python3-pip
### $ sudo pip3 install smbus2
### $ sudo pip3 install smbus
###
### Test01: Manually update row3. Publish only topic onence "-1"
### $ ros2 topic pub /lcd_display/row3 std_msgs/msg/String "data: Text on row 3" -1
###
### Launch sequence:
### 1) $ ros2 run pet_ros2_lcd_pkg lcd_node
### 2) ...but then you need somone to publish to the topics :-)
###

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

# Include I2C-port and Display hardware stuff (like ./lib/lcddriver.py + ./lib/i2c_lib.py)
from smbus2 import SMBus
sys.path.append("./lib/")  # TODO: PWD-sensitiv for the moment... import lcddriver.py + i2c_lib.py
import lcddriver
import i2c_lib  # TODO: This include file contains I2C-address

# TODO: Add the following "behaviours to class "LcdDisplayNode"
# lcd.lcd_clear()
# lcd.lcd_backlight("ON")
# lcd.lcd_backlight("OFF)

class LcdDisplayNode(Node): 
     
    def __init__(self):
        super().__init__("lcd_display_node")
        
        # Set default Display-I2C address. Accessed via ROS Parameters...
        self.declare_parameter( 'lcd_i2c_address', "0x3F", ParameterDescriptor(description='LCD-display I2C address [default "0x3F"]') )
        self.LCD_I2C_ADDRESS = self.get_parameter('lcd_i2c_address').get_parameter_value().string_value

        # Initiate the LCD Display via I2C interface
        self.lcd = lcddriver.lcd()   

        # If topics ('lcd_display/row#') content update -> Then ...callback 
        self.subscription_row1 = self.create_subscription(String, 'lcd_display/row1', self.lcd_update_row1_callback, 10)
        self.subscription_row2 = self.create_subscription(String, 'lcd_display/row2', self.lcd_update_row2_callback, 10)
        self.subscription_row3 = self.create_subscription(String, 'lcd_display/row3', self.lcd_update_row3_callback, 10)
        self.subscription_row4 = self.create_subscription(String, 'lcd_display/row4', self.lcd_update_row4_callback, 10)

        # prevent unused variable warning
        self.subscription_row1
        self.subscription_row2
        self.subscription_row3
        self.subscription_row4

        exit = False
        # Check we can open/contact the I2C-controller on the Display.
        try:
            self.lcd.lcd_display_string( "lcd_display_node initiating", 4)

            # Some basic information on the console
            self.get_logger().info("lcd_display_node has started")
            self.get_logger().info("- I2C: " + self.LCD_I2C_ADDRESS )

        except:
            # Note: a permission error can be fixed with a "sudo chmod a+rw /dev/i2c-1"
            self.get_logger().error("lcd_display_node canceled:"  + str(sys.exc_info()[1]) )
            self.exit = True
        

    # Callback when new String ('lcd_display/row1') to publish 
    def lcd_update_row1_callback(self, msg):
        self.lcd.lcd_display_string( msg.data, 1)
        self.row1_string = msg.data
        self.get_logger().info("LCD-row 1.= " + msg.data )


    # Callback when new String ('lcd_display/row2') to publish 
    def lcd_update_row2_callback(self, msg):
        self.lcd.lcd_display_string( msg.data, 2)
        self.row2_string = msg.data
        self.get_logger().info("LCD-row 2.= " + msg.data )

    # Callback when new String ('lcd_display/row3') to publish 
    def lcd_update_row3_callback(self, msg):
        self.lcd.lcd_display_string( msg.data, 3)
        self.row3_string = msg.data
        self.get_logger().info("LCD-row 3.= " + msg.data )

    # Callback when new String ('lcd_display/row4') to publish 
    def lcd_update_row4_callback(self, msg):
        self.lcd.lcd_display_string( msg.data, 4)
        self.row4_string = msg.data
        self.get_logger().info("LCD-row 4.= " + msg.data )

def main(args=None):
    rclpy.init(args=args)
    lcd_display_node = LcdDisplayNode()

    try:
        rclpy.spin(lcd_display_node)

    except KeyboardInterrupt:
        print("**** ☠️ Ctrl-C detected...")
    
    finally:
        print("**** lcd_display_node ending... ")
        print( str(sys.exc_info()[1]) )
        
        # Time to clean up stuff!
        # - Destroy the node explicitly
        #   (optional - otherwise it will be done automatically
        #   when the garbage collector destroys the node object)
        lcd_display_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
