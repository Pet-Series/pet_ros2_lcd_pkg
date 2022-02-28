# ROS2 Python LCD-controller
ROS2 node that publish text on a LCD display. 
Supported displays  LCD1604 + LCD2004 displays

**Input:** 3x topics <code>lcd_display/row1</code>...<code>lcd_display/row1</code><br>
**Output:** i2c connected display usiing PC8574A-interface to LCD1602 (16chr * 2 rows) or LCD2004 (20char * 4rows)
<table>
  <tr>
    <td>
      <img src="doc/ROS_pet_lcd_display_node.png" width="800px">
    </td>
    <td>
      .
    </td>
  </tr>
</table>

## Prerequisite: Hardware
* A/D converter: KY-053 Analog Digital Converter (ADS1115, 16-bit) via default I2C adr.=0x48
* Joystick: 3x analog 10K resistors. X-, Y- and Twist-axis.
* Single Board Computer(SBC): Raspberry Pi 4
<table>
  <tr>
    <td>
      <img src="doc/ldc1602+lcd2004.png" width="400px">
    </td>
    <td>
      .
    </td>
  </tr>
</table>

# Test
Launch ROS2 package + node. Then manually update each row on display with the following commands.
 ```
$ ros2 run pet_ros2_lcd_pkg pet_lcd_node
$ ros2 topic pub /lcd_display/row1 std_msgs/msg/String "data: Text at row 1" -1
$ ros2 topic pub /lcd_display/row2 std_msgs/msg/String "data: Text at row 2" -1
$ ros2 topic pub /lcd_display/row3 std_msgs/msg/String "data: Text at row 3" -1
$ ros2 topic pub /lcd_display/row4 std_msgs/msg/String "data: Text at row 4" -1
 ```
