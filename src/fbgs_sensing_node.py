#!/usr/bin/env python
import rospy
import socket
import struct
import sys
import os
import rospkg
from PyQt4 import QtGui, uic
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension

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
        channel['error'] = dm[-4:]; dm=dm[0:-4]
        channel['peak_wavelengths'] = dm[-channel['sensor_count']:]; dm = dm[0:-channel['sensor_count']]
        channel['peak_wavelengths'] = [float(w) for w in channel['peak_wavelengths']] # Cast to float
        channel['peak_wavelengths'].reverse()   # maintain order
        channel['peak_powers'] = dm[-channel['sensor_count']:]; dm = dm[0:-channel['sensor_count']]
        channel['peak_powers'] = [float(p) for p in channel['peak_powers']] # Cast to float
        channel['peak_powers'].reverse()        # maintain order

        # Nest channels inside of data structure
        data['channel_{0}'.format(channel['name'])] = channel

    data['strain_count'] = int(dm.pop())
    data['strain'] = [float(d) for d in dm[-data['strain_count']:]]
    data['strain'].reverse()    # maintain order
    return data

def get_wavelengths(fbgs):
    wavelengths = []
    for key, value in sorted(fbgs.iteritems()):
        if key.startswith('channel'):
            wavelengths.extend(fbgs[key]['peak_wavelengths'])
    return wavelengths

class MyWindow(QtGui.QMainWindow):
    def __init__(self):
        super(MyWindow, self).__init__()
        rp = rospkg.RosPack()
        ui_path = os.path.join(rp.get_path('fbgs_sensor'), 'resources', 'fbgs_sensor.ui')
        uic.loadUi(ui_path, self)
        self.connectChk.clicked.connect(self.run)
        self.show()

    def run(self):
        #Handle no host
        if self.hostInput.text().isEmpty():
            self.connectChk.setChecked(False)
            return
        # Create ros publisher and start node
        pub = rospy.Publisher('fbgs_strain', Float32MultiArray, queue_size=10)
        pub_wavelength = rospy.Publisher('fbgs_wavelength', Float32MultiArray, queue_size=10)
        rospy.init_node('fbgs_sensor', anonymous=True)

        # Create TCP/IP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect to port where server is listening
        host_computer = self.hostInput.text().split(':')[0]
        port = int(self.hostInput.text().split(':')[-1])
        server_address = (host_computer, port)

        print('Connecting to %s:%s' % server_address)

        try:
            sock.connect(server_address)
        except:
            self.connectChk.setChecked(False)
            print('Error when connecting. Check host name')
            return

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


if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    window = MyWindow()
    sys.exit(app.exec_())
