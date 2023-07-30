#!/usr/bin/env python

"""
Read sensor data in weather station and print in terminal.

"""

import serial;
import io;
import time;
import os;

if __name__ == '__main__' :
    try :
        # configure the serial connections (the parameters differs on the device you are connecting to)
        with serial.Serial(port='/dev/ttyUSB0', baudrate=2400, timeout=1,
                       xonxoff=False, rtscts=False, dsrdtr=True) as s:
            for line in s:
                print(line)

    except :
        print('Program exit !')
        pass
    finally :
        print('Finished reading')

        s.close()
    pass