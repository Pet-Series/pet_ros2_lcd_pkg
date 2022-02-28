#!/usr/bin/env python3'
# coding = utf-8
########################################################################################
##
## Maintainer: stefan.kull@gmail.com
##
## Input data: TODO: Different CPU, Linux / Ubuntu or ROS2 measured values. As human readeble/formated form.
## Output: Publish to selected topics. Like '/lcd_display/row2', ' IP 192.168.000.180 '
##
## Lunch sequence
## 1) Start the display subscriber...
##    $ cd ~/ws_ros2/src/pet_ros2_lcd_pkg/pet_ros2_lcd_pkg 
##    $ ros2 run pet_ros2_lcd_pkg pet_lcd_node
##
## 2) Start this message spam-generator
##    $ ros2 run pet_ros2_lcd_pkg display_publish_node 
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

class displayPublishNode(Node): 
    
    def __init__(self, timerHz):
        self.timerHz  = timerHz
        super().__init__("display_publish_node")

        self.publisher_row1 = self.create_publisher(String, 'lcd_display/row1', 10)
        self.publisher_row2 = self.create_publisher(String, 'lcd_display/row2', 10)
        self.publisher_row3 = self.create_publisher(String, 'lcd_display/row3', 10)
        self.publisher_row4 = self.create_publisher(String, 'lcd_display/row4', 10)
        self.timer = self.create_timer(1, self.timer_callback )
        self.i = 0

    def timer_callback(self):

        ### SECTION BELOW => ONLY TEST CONTENT... should be replace with some meaningful information :-)
        msgRow1 = String()
        msgRow1.data = '[Topic=row1,Lap=' + str(self.i) + "]"

        msgRow2 = String()
        msgRow2.data = '[Topic=row2,Lap=' + str(self.i) + "]"
        
        msgRow3 = String()
        msgRow3.data = '[Topic=row3,Lap=' + str(self.i) + "]"
        
        msgRow4 = String()
        msgRow4.data = '[Topic=row4,Lap=' + str(self.i) + "]"

        # msg.data = 'Publisher: %d' % self.i
        self.publisher_row1.publish(msgRow1)
        self.publisher_row2.publish(msgRow2)
        self.publisher_row3.publish(msgRow3)
        self.publisher_row4.publish(msgRow4)

        self.get_logger().info('Publishing: ' + str(self.i) )
        self.i += 1

def main(args=None):
    rclpy.init(args=args) 
    display_publish_node = displayPublishNode( 2 )

    try:
        rclpy.spin(display_publish_node)
        
    except KeyboardInterrupt:
        print("**** ☠️ Ctrl-C detected...")
    
    finally:
        print("**** display_publish_node ending... ")
        print( str(sys.exc_info()[1]) )
        
        # Time to clean up stuff!
        # - Destroy the node explicitly
        #   (optional - otherwise it will be done automatically
        #   when the garbage collector destroys the node object)
        display_publish_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()