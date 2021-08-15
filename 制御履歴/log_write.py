# -*- coding: utf-8 -*-
import csv
import serial 
import datetime
import sys

i = 0  #カウント用
ser = serial.Serial("/dev/cu.SLAB_USBtoUART")  # Arduinoが接続されているCOMポートを指定

while(1):
    value = float(ser.readline().decode('utf-8').rstrip('\n'))
    #date = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    print(value)
    with open('test.csv', 'a') as f:
        print('{},{},{}'.format(value),file=f)
