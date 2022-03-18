"""
Classifies EMG serial reading into the flexion mode. Uses serialized trained
models from the Trained_Models directory.

Change line: `filename` to use different ML models.

Modes:
    0 - no flexion
    1 - index finger flexed
    2 - middle finger flexed
    3 - fourth finger flexed 
    4 - pinky finger flexed

Models:
    MLP - multilayer perceptron
    KNN - K nearest neighbors
    RFC - random forest classifier
    
"""

import serial
import pickle
import numpy as np
from MLP import MLP
from KNN import KNN
from RFC import RFC

arduino_port = "/dev/ttyACM0"   # Based on port your device is connected to
baud = 9600

ser = serial.Serial(arduino_port, baud)

# Models to use for classification
filename = './Trained_Models/KNN.sav'
# filename = './Trained_Models/RFC.sav'
# filename = './Trained_Models/MLP.sav'
model = pickle.load(open(filename, 'rb'))

while True:
    # Read data
    getData = str(ser.readline().decode())
    data = getData[0:][:-2]
    output = eval(data)
    output = np.array([output])

    pred, preds, score, _ = model.prediction(output)
    print(f"Prediction: {pred}")

    # Write class to arduino
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
