import serial
import csv

ser = serial.Serial('/dev/cu.SLAB_USBtoUART',115200)
f = open('data1500.csv', 'wb')
csvWriter = csv.writer(f)

smoothing = 10000

while True:
    lineData = []
    line = float(ser.readline())
    data = line.split(",")
    del data[-1]
    #print data
    csvWriter.writerow(data)
    f.close()