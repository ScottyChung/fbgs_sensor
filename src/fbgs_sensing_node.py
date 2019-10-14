#!/usr/bin/env python
import rospy
import socket
import struct
import sys
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension

def get_peaks(message):
    data_message = message.split('\t')
    number_optical_lines = data_message[4]
    peaks = [float(d) for d in data_message[-5:]]
    return peaks


# Create ros publisher and start node
pub = rospy.Publisher('fbgs_strain', Float32MultiArray, queue_size=10)
rospy.init_node('fbgs_sensor', anonymous=True)

# Create TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to port where server is listening
host_computer = '1612-ultron'
port = 2055
server_address = (host_computer, port)

print('Connecting to %s:%s' % server_address)
sock.connect(server_address)

while True:
    try:
        # Read size of data
        data_size = sock.recv(4)
        data_size = struct.unpack(">i",data_size)[0] # Convert bytestring to int32

        # Collect expected amount of data
        data_message = b''
        data_rec = 0
        while data_rec < data_size:
            new_data = sock.recv(128)
            data_rec += len(new_data)
            data_message += new_data

        if data_message: # New message to publish
            peaks = get_peaks(data_message)
            # Create rosmessage
            dim = MultiArrayDimension('length', 5, 1)
            layout = MultiArrayLayout([dim], 0)
            ros_message = Float32MultiArray(layout, peaks)
            pub.publish(ros_message)


        #sys.stdout.write('\rMessage received: {0}\r'.format(data_message))

    except KeyboardInterrupt:
        print('Closing socket')
        sock.close()
        sys.exit()
