#!/usr/bin/env python3
import rospy
import socket
import struct
import sys
import os
import rospkg
import time
import select
from threading import Thread
from PyQt5 import QtGui, uic
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
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
    data_message = message.split(b'\t')
    # Reverse message to pop off easier
    dm = data_message[::-1]
    data = {}
    data['date'] = dm.pop()
    data['time'] = dm.pop()
    data['count'] = int(dm.pop())
    data['measured_channels'] = int(dm.pop())
    for i in range(0,data['measured_channels']):
        channel = {}
        channel['name'] = int(dm.pop())
        channel['sensor_count'] = int(dm.pop())
        
        channel['error'] = [int(e) for e in dm[-4:]] #Get last four elements
        dm=dm[0:-4] #Remove last four elements from list
        
        channel['peak_wavelengths'] = dm[-channel['sensor_count']:]
        dm = dm[0:-channel['sensor_count']]
        channel['peak_wavelengths'] = [float(w) for w in channel['peak_wavelengths']] # Cast to float
        channel['peak_wavelengths'].reverse()   # maintain order
        
        channel['peak_powers'] = dm[-channel['sensor_count']:]; 
        dm = dm[0:-channel['sensor_count']]
        channel['peak_powers'] = [float(p) for p in channel['peak_powers']] # Cast to float
        channel['peak_powers'].reverse()        # maintain order

        # Nest channels inside of data structure
        data['channel_{0}'.format(channel['name'])] = channel


    data['strain_count'] = int(dm.pop())
    data['strain'] = [float(d.split(b'\r')[0]) for d in dm[-data['strain_count']:]]
    data['strain'].reverse()    # maintain order
    return data

def get_wavelengths(fbgs):
    wavelengths = []
    for key, value in sorted(fbgs.items()):
        if key.startswith('channel'):
            wavelengths.extend(fbgs[key]['peak_wavelengths'])
    return wavelengths

class FBGSWorker(QThread):
    #Use signal to update textbox from seperate thread
    print_message = pyqtSignal(str)
    #Signal to run the uncheckbox function of main
    uncheck = pyqtSignal()

    def __init__(self):
        QThread.__init__(self)
        self.running = True
        self.host = None

    def connect(self, host):
        #Handle no host
        if not host:
            self.print_message.emit('No host specified')

        # Create ros publisher and start node
        self.pub = rospy.Publisher('fbgs_strain', Float32MultiArray, queue_size=10)
        self.pub_wavelength = rospy.Publisher('fbgs_wavelength', Float32MultiArray, queue_size=10)

        # Create TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect to port where server is listening
        host_computer = host.split(':')[0]
        port = int(host.split(':')[-1])
        server_address = (host_computer, port)

        self.print_message.emit('Connecting to %s:%s' % server_address)
        #self.printText.append('Connecting to %s:%s' % server_address)

        try:
            print('trying to connect')
            self.sock.settimeout(5)
            self.sock.connect(server_address)
        except Exception as e:
            print('unable to connect')
            self.print_message.emit('Error! Could not connect')
            self.stop()
        self.print_message.emit('Connected! Starting to publish data')

    def run(self):
        self.connect(self.host)
        self.print_message.emit('Trying to get data')
        while self.running:
            #Check socket for data
            ready_read, _, _ = select.select([self.sock],[],[], 5)

            #If no data 
            if not ready_read:
                self.print_message.emit('ERROR: No data ready. Is Illumisense running?')
                self.stop()
            data_message = recv_msg(self.sock)

            if bool(data_message): # New message to publish
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
        self.stop()

    def stop(self):
        self.print_message.emit('Stopping Worker')
        self.sock.close()
        self.running = False
        self.uncheck.emit()
        rospy.signal_shutdown('exit')
        return

class MyWindow(QMainWindow):
    def __init__(self):
        super(MyWindow, self).__init__()
        rp = rospkg.RosPack()
        ui_path = os.path.join(rp.get_path('fbgs_sensor'), 'resources', 'fbgs_sensor.ui')
        uic.loadUi(ui_path, self)
        self.connectChk.clicked.connect(self.clicked)
        rospy.init_node('fbgs_sensor', anonymous=True)
        self.show()

    def clicked(self):
        print('clicked')
        print(self.connectChk.checkState())
        if self.connectChk.checkState():
            print('making worker')
            self.worker = FBGSWorker()
            print('made worker')
            self.worker.print_message.connect(self.print_msg)
            print('connected print message')
            self.worker.uncheck.connect(self.uncheck_box)
            print('connected uncheck box')
            self.worker.host = self.hostInput.text()
            self.worker.start()

        else:
            print('quitting worker')
            self.worker.running = False
            self.worker.quit()

    def print_msg(self,msg):
        self.printText.append(msg)

    def uncheck_box(self):
        print('uncheck')
        self.connectChk.setChecked(False)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MyWindow()
    sys.exit(app.exec_())

