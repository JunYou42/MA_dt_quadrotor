#!/usr/bin/env python3

"""
Open a shell over MAVLink and request for uORB message "actuator_outputs".

"""


from __future__ import print_function
import sys, select
import termios
from timeit import default_timer as timer
from argparse import ArgumentParser
import os
import re
import rospy
from std_msgs.msg import Float32MultiArray
import time

try:
    from pymavlink import mavutil
except ImportError as e:
    print("Failed to import pymavlink: " + str(e))
    print("")
    print("You may need to install it with:")
    print("    pip3 install --user pymavlink")
    print("")
    sys.exit(1)

try:
    import serial
except ImportError as e:
    print("Failed to import pyserial: " + str(e))
    print("")
    print("You may need to install it with:")
    print("    pip3 install --user pyserial")
    print("")
    sys.exit(1)


class MavlinkSerialPort():
    '''an object that looks like a serial port, but
    transmits using mavlink SERIAL_CONTROL packets'''
    def __init__(self, portname, baudrate, devnum=0, debug=0):
        self.baudrate = 0
        self._debug = debug
        self.buf = ''
        self.port = devnum
        self.debug("Connecting with MAVLink to %s ..." % portname)
        self.mav = mavutil.mavlink_connection(portname, autoreconnect=True, baud=baudrate)
        self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GENERIC, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        self.mav.wait_heartbeat()
        self.debug("HEARTBEAT OK\n")
        self.debug("Locked serial device\n")

        
    def debug(self, s, level=1):
        '''write some debug text'''
        if self._debug >= level:
            print(s)

    def write(self, b):
        '''write some bytes'''
        self.debug("sending '%s' (0x%02x) of len %u\n" % (b, ord(b[0]), len(b)), 2)
        while len(b) > 0:
            n = len(b)
            if n > 70:
                n = 70
            buf = [ord(x) for x in b[:n]]
            buf.extend([0]*(70-len(buf)))
            self.mav.mav.serial_control_send(self.port,
                                             mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                             mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                                             0,
                                             0,
                                             n,
                                             buf)
            b = b[n:]

    def close(self):
        self.mav.mav.serial_control_send(self.port, 0, 0, 0, 0, [0]*70)

    def _recv(self):
        '''read some bytes into self.buf'''
        m = self.mav.recv_match(condition='SERIAL_CONTROL.count!=0',
                                type='SERIAL_CONTROL', blocking=True,
                                timeout=0.03)
        if m is not None:
            if self._debug > 2:
                print(m)
            data = m.data[:m.count]
            self.buf += ''.join(str(chr(x)) for x in data)

    def read(self, n):
        '''read some bytes'''
        if len(self.buf) == 0:
            self._recv()
        if len(self.buf) > 0:
            if n > len(self.buf):
                n = len(self.buf)
            ret = self.buf[:n]
            self.buf = self.buf[n:]
            if self._debug >= 2:
                for b in ret:
                    self.debug("read 0x%x" % ord(b), 2)
            return ret
        return ''


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('port', metavar='PORT', nargs='?', default = None,
            help='Mavlink port name: serial: DEVICE[,BAUD], udp: IP:PORT, tcp: tcp:IP:PORT. Eg: \
    /dev/ttyUSB0 or 0.0.0.0:14550. Auto-detect serial if not given.')
    parser.add_argument("--baudrate", "-b", dest="baudrate", type=int,
                      help="Mavlink port baud rate (default=57600)", default=57600)
    args = parser.parse_args()
    rospy.init_node('actuator_outputs_publisher', anonymous=True)
    ros_publisher = rospy.Publisher('actuator_outputs', Float32MultiArray, queue_size=1)

    if args.port == None:
        if sys.platform == "darwin":
            args.port = "/dev/tty.usbmodem01"
        else:
            serial_list = mavutil.auto_detect_serial(preferred_list=['*FTDI*',
                "*Arduino_Mega_2560*", "*3D_Robotics*", "*USB_to_UART*", '*PX4*', '*FMU*', "*Gumstix*"])

            if len(serial_list) == 0:
                print("Error: no serial connection found")
                return

            if len(serial_list) > 1:
                print('Auto-detected serial ports are:')
                for port in serial_list:
                    print(" {:}".format(port))
            print('Using port {:}'.format(serial_list[0]))
            args.port = serial_list[0].device


    print("Connecting to MAVLINK...")
    mav_serialport = MavlinkSerialPort(args.port, args.baudrate, devnum=10)

    mav_serialport.write('\n') # make sure the shell is started

    # disable echo & avoid buffering on stdin
    fd_in = sys.stdin.fileno()
    try:
        old_attr = termios.tcgetattr(fd_in)
        new_attr = termios.tcgetattr(fd_in)
        new_attr[3] = new_attr[3] & ~termios.ECHO # lflags
        new_attr[3] = new_attr[3] & ~termios.ICANON
        termios.tcsetattr(fd_in, termios.TCSANOW, new_attr)
    except termios.error:
        # tcgetattr can fail if stdin is not a tty
        old_attr = None
    ubuf_stdin = os.fdopen(fd_in, 'rb', buffering=0)

    try:
        cur_line = ''
        command_history = []
        cur_history_index = 0

        def erase_last_n_chars(N):
            if N == 0: return
            CURSOR_BACK_N = '\x1b['+str(N)+'D'
            ERASE_END_LINE = '\x1b[K'
            sys.stdout.write(CURSOR_BACK_N + ERASE_END_LINE)

        next_heartbeat_time = timer()

        quit_time = None
        while quit_time is None or quit_time > timer():
            while True:
                i, o, e = select.select([ubuf_stdin], [], [], 0)
                if not i: break
                ch = 'listener actuator_outputs' # send this command to serial port

                mav_serialport.write(ch +'\n')
                break

            # ! read and extract actuator ouput data from 4 motors and publish as rostopic 
            data = mav_serialport.read(4096)
            if data and len(data) > 0:

                match = re.search(r"output: ", data)
                if match:
                    start_index = data.find("[")  # Find the index of "["

                    if start_index != -1 :  # Check if "[" and "]" are found
                        data_subset = data[start_index+1:]  # Extract the substring between "[" and "]"
                        char_sets = data_subset.split(",")  # Split the substring by "," to get individual char sets
                        
                        # Remove whitespace from each char set and store them in a list
                        char_list = [char_set.strip() for char_set in char_sets]
                        if len(char_list)>4:
                            float_list = [float(char_str) for char_str in char_list[:4]]

                            print("Float numbers:", float_list)
                            actuator_msg = Float32MultiArray(data=float_list)
                            ros_publisher.publish(actuator_msg)
                            time.sleep(1)

                sys.stdout.flush()

            # handle heartbeat sending
            heartbeat_time = timer()
            if heartbeat_time > next_heartbeat_time:
                mav_serialport.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GENERIC, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                next_heartbeat_time = heartbeat_time + 1
            
    except serial.serialutil.SerialException as e:
        print(e)

    except KeyboardInterrupt:
        mav_serialport.close()
        

    finally:
        if old_attr:
            termios.tcsetattr(fd_in, termios.TCSADRAIN, old_attr)


if __name__ == '__main__':
    main()
