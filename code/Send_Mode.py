import serial
import pickle
import numpy as np
from MLP import MLP
from KNN import KNN
from RFC import RFC

arduino_port = "/dev/ttyACM0"
baud = 9600

ser = serial.Serial(arduino_port, baud)

filename = './Trained_Models/MLP.sav'
# filename = './Trained_Models/KNN.sav'
# filename = './Trained_Models/RFC.sav'
model = pickle.load(open(filename, 'rb'))

while True:
    # read data
    getData = str(ser.readline().decode())
    data = getData[0:][:-2]
    output = eval(data)
    output = np.array([output])
    # print(data)

    pred, preds, score, _ = model.prediction(output)
    # print(f"Score: {score}")
    print(f"Prediction: {pred}")

    # write to arduino
    if pred == 0:
        ser.write(b'0')
    elif pred == 1:
        ser.write(b'1')
    elif pred == 2:
        ser.write(b'2')
    elif pred == 3:
        ser.write(b'3')
    elif pred == 4:
        ser.write(b'4')
