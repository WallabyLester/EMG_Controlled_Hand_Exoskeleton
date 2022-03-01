import serial

arduino_port = "/dev/ttyACM0"
baud = 9600

ser = serial.Serial(arduino_port, baud)

# read data
getData = str(ser.readline().decode())
data = getData[0:][:-2]
print(data)

# write to arduino
ser.write(b'1') # ascii 

