# EMG Controlled Hand Exoskeleton

![Final_Demo](./gifs/Final_Demo.gif)

## Overview
The goal of this project was to design an EMG controlled hand exoskeleton that would aid stroke patients in playing the piano. The exoskeleton is 3D printed with the `.stl` and `.step` files located in the `cad` directory. 

Control is achieved using an Arduino Uno. There are three available control modes:
1. Using thresholds. Thresholding uses manually found thresholds from EMG readings and basic motor control in which the fingers move up and down a set amount based on an input power. 

2. Using machine learning (ML) classification. ML classification uses the contained ML models to predict which finger is being moved and moves the associated motor with basic motor control based on an input power.

3. Using ML classification with encoder position control from a PID feedback loop. ML classification with the encoders implements external and internal interrupts to read encoder values and perform position control using a PID feedback loop. The Arduino Uno only has two interrupt pins (2 and 3) so an internal interrupt was made using Timer1 (16 bit). 

The control signal is provided by Myoware EMG sensors which provide a processed EMG signal. The signals are amplified, rectified, and integrated before being output. The sensors also offer a raw EMG signal if you would like to perform your own processing. Code for reading from the EMG sensors, saving readings, and plotting data are in the `code` directory. 

Machine learning is used on the EMG readings received from four sensor readings, enabling classification of the signals in 5 modes: 0 - no flexion, 1 - index finger flexed, 2 - middle finger flexed, 3 - fourth finger flexed, and 4 - pinky finger flexed. `Classification_Notes` contains the various scikit-learn machine learning models as well as a Tensorflow model. Refer to it for further notes on the models and GridSearchCV to find the best hyperparameters for the models. The trained models have been saved serialized in the `Trained_models` directory.

There are four available models: 

1. K Nearest Neighbors - KNN predicts the class by calculating the distance between the input point and all of the other points. The algorithm then selects the K number of points which is closest to the input point and uses a majority vote for which class this input must be in. 

2. Random forest classifier - RFC is made up of many decision trees. It builds an uncorrelated forest of trees which can provide a more accurate prediction than just one tree. Decision trees predict an input class based on decision rules which causes branches from each "node". 

3. Multilayer perceptron - MLP is a feed forward neural network. It uses an input layer, output layer, and any number of hidden layers. With an increased number of hidden layers it is able to predict more complex classes. They can also solved non-linear classification problems.


## Usage Instructions
Compile `EMGControlEncoder.ino` onto the Arduino and run `Send_Mode.py` in a terminal with `python3 Send_Mode.py` to start predicting EMG readings. The code will print out the predicted mode. 

Use `EMGControl.ino` for running ML classification with basic motor control and `EMGControlThresholds.ino` to run basic motor control using thresholds. 

Use `EMGControlTest.ino` if you would like to test individual EMG sensor and motor control.

Use `EMGToSerial.ino` to output the EMG readings as serial data and run `Read_EMG_Data.py` to save the data. Run `Plotting.py` to plot individual data file EMG readings and modes.

Use `EncoderRead.ino` to read all the encoders and `EncoderPosition.ino` to control motor movements using all the encoders. 

Use `MotorControl.ino` to test movement of individual motors and `Stop.ino` to stop all motor movement if you are running things and would like to halt them.

## Configuration Instructions
The classification model used in `Send_Mode.py` can be changed based on which ML model you would like to use. Change the line: `filename` to use different models.

The encoder positions can be adjusted based on the desired amount to move the fingers. These would be the `pos_index`, `pos_middle`, `pos_fourth`, and `pos_pinky` variables in the encoder control files. 
