#!/usr/bin/env python
import rospy
import socket
import struct
import sys
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension

def recv_msg(sock):
    # Read size of data
    data_size = recv_all(sock,4)
    if not data_size:
        return None
    data_size = struct.unpack(">i",data_size)[0] # Convert bytestring to int32
    return recv_all(sock, data_size)

def recv_all(sock, n):
    #Helper to grab a spefic amount of bytes
    data = bytearray()
    while len(data) < n:
        packet = sock.recv(n-len(data))
        if not packet:
            return None
        data.extend(packet)
    return data
    
def parse_message(message):
    data_message = message.split('\t')
    # Reverse message to pop off easier
    dm = data_message[::-1]
    data = {}
    data['date'] = dm.pop()
    data['time'] = dm.pop()
    data['count'] = int(dm.pop())
    data['measured_channels'] = int(dm.pop())
   
    for i in xrange(data['measured_channels']):
        channel = {}
        channel['name'] = int(dm.pop())
        channel['sensor_count'] = int(dm.pop())
        channel['error'] = dm[-4:]
        dm=dm[0:-4]
        channel['peak_wavelengths'] = dm[-channel['sensor_count']:]; dm = dm[0:-channel['sensor_count']]
        channel['peak_wavelengths'] = [float(w) for w in channel['peak_wavelengths']] # Cast to float
        channel['peak_wavelengths'].reverse()   # maintain order
        channel['peak_powers'] = dm[-channel['sensor_count']:]; dm = dm[0:-channel['sensor_count']]
        channel['peak_powers'] = [float(p) for p in channel['peak_powers']] # Cast to float
        channel['peak_powers'].reverse()        # maintain order

        # Nest channels inside of data structure
        data['channel_{0}'.format(channel['name'])] = channel

    data['strain_count'] = int(dm.pop())
    data['strain'] = [float(d.split('\r')[0]) for d in dm[-data['strain_count']:]]
    data['strain'].reverse()    # maintain order
    return data

def get_wavelengths(fbgs):
    wavelengths = []
    for key, value in sorted(fbgs.iteritems()):
        if key.startswith('channel'):
            wavelengths.extend(fbgs[key]['peak_wavelengths'])
    return wavelengths

# Create ros publisher and start node
pub = rospy.Publisher('fbgs_strain', Float32MultiArray, queue_size=10)
pub_wavelength = rospy.Publisher('fbgs_wavelength', Float32MultiArray, queue_size=10)
rospy.init_node('fbgs_sensor', anonymous=True)

# Create TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to port where server is listening
#host_computer = 'biomd-thorn'
host_computer = 'AC-GR-214GXH'
port = 2055
server_address = (host_computer, port)

print('Connecting to %s:%s' % server_address)
sock.connect(server_address)
print('Connected attempting to publish data')
while True:
    try:

        # Collect data message
        data_message = recv_msg(sock)

        if data_message: # New message to publish
            data = parse_message(data_message)
            # Create strain message
            dim = MultiArrayDimension('length', data['strain_count'], 1)
            layout = MultiArrayLayout([dim], 0)
            strain_message = Float32MultiArray(layout, data['strain'])
            pub.publish(strain_message)
            
            # Create wavelength message
            wavelengths = get_wavelengths(data)
            dim = MultiArrayDimension('length', len(wavelengths), 1)
            layout = MultiArrayLayout([dim], 0)
            wavelength_message = Float32MultiArray(layout, wavelengths)
            pub_wavelength.publish(wavelength_message)


        #sys.stdout.write('\rMessage received: {0}\r'.format(data_message))

    except KeyboardInterrupt:
        print('Closing socket')
        sock.close()
        sys.exit()
