"""
Reads EMG serial data and saves it as a CSV.

Saves data into the data folder, change filename to write onto file or 
to a new file. 

Current data was collected with 5 seconds periods. 5 seconds without flexion,
5 seconds of index finger flexion, 5 seconds without flexion, 5 seconds of 
middle finger flexion, and so on repeated twice. Mode is not automatically 
populated and must be manually labeled due to variability in user hold times.
Optionally you can automatically populate the mode but this may cause outliers. 

"""

import serial
import csv

arduino_port = "/dev/ttyACM0"  # based on port your device is connected to
baud = 9600
fileName = "./data/Combined_EMG_data40.csv"
samples = 100  # increase to record a greater number of samples
i = 0

ser = serial.Serial(arduino_port, baud)
print(f"Connected to Arduino port: {arduino_port}")
file = open(fileName, "w")
writer = csv.writer(file)

while i <= samples:
    # display data to the terminal
    getData = str(ser.readline().decode())
    data = getData[0:][:-2]
    print(data)

    # add data to file
    file.write(data + "\n")
    i += 1

print("Data collection complete")

file.close()
ser.close()