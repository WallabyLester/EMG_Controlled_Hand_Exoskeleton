import serial
import csv

# 1 Hz
arduino_port = "/dev/ttyACM0"
# baud = 115200
baud = 9600
fileName = "./data/Combined_EMG_data1.csv"
samples = 100
# print_labels = False
i = 0

ser = serial.Serial(arduino_port, baud)
print(f"Connected to Arduino port: {arduino_port}")
file = open(fileName, "w")
writer = csv.writer(file)
# print("File completed")

while i <= samples:
    # if print_labels:
    #     if i == 0:
    #         print("Printing column headers")   
    #     else:
    #         print(f"Line {str(i)}: writing...")

    # display data to the terminal
    getData = str(ser.readline().decode())
    data = getData[0:][:-2]
    print(data)

    # add data to file
    # file = open(fileName, "a")
    file.write(data + "\n")
    # writer.write(data + "\n")
    i += 1

print("Data collection complete")

# csvreader = csv.reader(file)
# rows = []

# for row in csvreader:
#     rows.append(row)

file.close()
ser.close()