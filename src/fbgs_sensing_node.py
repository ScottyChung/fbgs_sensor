#!/usr/bin/env python
import rospy
import socket
import struct
import sys
import os
import rospkg
import time
from threading import Thread
from PyQt4 import QtGui, uic
from PyQt4.QtCore import *
from PyQt4.QtGui import *
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

class FBGSWorker(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.running = True

    def connect(self, host):
        #Handle no host
        if host.isEmpty():
            self.connectChk.setChecked(False)
            return
        # Create ros publisher and start node
        self.pub = rospy.Publisher('fbgs_strain', Float32MultiArray, queue_size=10)
        self.pub_wavelength = rospy.Publisher('fbgs_wavelength', Float32MultiArray, queue_size=10)
        rospy.init_node('fbgs_sensor', anonymous=True)

        # Create TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect to port where server is listening
        host_computer = host.split(':')[0]
        port = int(host.split(':')[-1])
        server_address = (host_computer, port)

        print('Connecting to %s:%s' % server_address)

        self.sock.connect(server_address)
        print('Connected! Starting to publish data')

    def run(self):
        print('Trying to get data')
        while self.running:
            try:
                data_message = recv_msg(self.sock)

                if data_message: # New message to publish
                    data = parse_message(data_message)
                    # Create strain message
                    dim = MultiArrayDimension('length', data['strain_count'], 1)
                    layout = MultiArrayLayout([dim], 0)
                    strain_message = Float32MultiArray(layout, data['strain'])
                    self.pub.publish(strain_message)
                    
                    # Create wavelength message
                    wavelengths = get_wavelengths(data)
                    dim = MultiArrayDimension('length', len(wavelengths), 1)
                    layout = MultiArrayLayout([dim], 0)
                    wavelength_message = Float32MultiArray(layout, wavelengths)
                    self.pub_wavelength.publish(wavelength_message)
            except:
                time.sleep(0.001)
        self.stop()

    def stop(self):
        self.sock.close()

class MyWindow(QtGui.QMainWindow):
    def __init__(self):
        super(MyWindow, self).__init__()
        rp = rospkg.RosPack()
        ui_path = os.path.join(rp.get_path('fbgs_sensor'), 'resources', 'fbgs_sensor.ui')
        uic.loadUi(ui_path, self)
        self.connectChk.clicked.connect(self.clicked)
        self.show()

    def clicked(self):
        if self.connectChk.checkState():
            self.worker = FBGSWorker()
            self.worker.connect(self.hostInput.text())
            self.worker.start()
        else:
            print('unclick')
            self.worker.running = False

        


if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    window = MyWindow()
    sys.exit(app.exec_())

