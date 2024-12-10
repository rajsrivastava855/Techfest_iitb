#!/usr/bin/env python3

import rospy
import serial
import struct
import time
from std_msgs.msg import Float32, Float32MultiArray

def initialize_serial(port, baudrate):
    """Initialize the serial connection."""
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        rospy.loginfo(f"Connected to {port} at {baudrate} baud.")
        time.sleep(2)  # Allow time for microcontroller to reset
        return ser
    except serial.SerialException as e:
        rospy.logerr(f"Serial Connection Error: {e}")
        return None

def receive_float(ser):
    """Receive a float from the microcontroller."""
    try:
        if ser.in_waiting > 0:
            identifier = ser.read(2).decode('utf-8')  # Check for identifier
            if identifier == '#A':  # Check if correct command received
                data = struct.unpack('<f', ser.read(4))[0]  # Read 4-byte float
                rospy.loginfo(f"Received Float: {data}")
                return data
    except Exception as e:
        rospy.logerr(f"Error receiving float: {e}")
    return None

def send_array(ser, array_data):
    """Send an array of floats to the microcontroller."""
    try:
        ser.write(b'#B')  # Send command identifier for sending array
        ser.write(struct.pack('<i', len(array_data)))  # Send array size
        for value in array_data:
            ser.write(struct.pack('<f', value))  # Send each float
        rospy.loginfo(f"Sent Array: {array_data}")
    except Exception as e:
        rospy.logerr(f"Error sending array: {e}")

def main():
    rospy.init_node('serial_comm_node')
    rospy.loginfo("Starting Serial Communication Node...")

    port = rospy.get_param("~port", "/dev/ttyUSB0")  # Change as per your setup
    baudrate = rospy.get_param("~baudrate", 9600)

    pressure_pub = rospy.Publisher('/pressure', Float32, queue_size=10)
    rate = rospy.Rate(2)  # 2 Hz

    # Example array to send
    example_array = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]

    # Initialize Serial Communication
    ser = initialize_serial(port, baudrate)
    if ser is None:
        rospy.logerr("Failed to initialize serial connection.")
        return

    try:
        while not rospy.is_shutdown():
            # Check for incoming float when '#A' command is sent
            received_float = receive_float(ser)
            if received_float is not None:
                pressure_pub.publish(received_float)
                rospy.loginfo(f"Published Float: {received_float}")

            # Send array when '#B' command is detected
            command_to_send = input("Enter command ('#B' to send array, 'exit' to quit): ")
            if command_to_send.strip() == '#B':
                send_array(ser, example_array)

            elif command_to_send.strip().lower() == 'exit':
                rospy.loginfo("Exiting the node.")
                break

            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Node Interrupted.")
    finally:
        ser.close()
        rospy.loginfo("Serial connection closed.")

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        rospy.logerr(f"Unhandled Exception: {e}")
