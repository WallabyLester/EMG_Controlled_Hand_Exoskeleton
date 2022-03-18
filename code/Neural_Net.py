"""
Tensorflow neural net to classify finger flexion based on EMG readings.
Model description in Classification Notes. Refer to the notes for further
details.

Training: Use to fit model using training data

Prediction: Use to predict class values

"""

import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import MinMaxScaler
from keras.models import Sequential
from keras.layers import Dense,Dropout
from keras.utils.all_utils import to_categorical
import os
import glob
import pickle


class NET():
    """
    Class to implement Tensorflow model.
    """
    def __init__(self):
        self.model = Sequential()
        
    def read_data(self):
        """ Function to read all data contained in the data folder and separate into training and test. 

        Remove test split if you would like to train on all your data. Adjust path as needed. 
        """
        path = os.getcwd()
        csv_files = glob.glob(os.path.join(path, "./data/Combined_EMG_data*"))
        data_list = []
        X_list = []
        y_list = []

        for i in csv_files:
            
            df = pd.read_csv(i)
            data_list.append(df)

        for j in data_list:
            X = j[['EMG Sensor 1', 'EMG Sensor 2', 'EMG Sensor 3', 'EMG Sensor 4']].to_numpy().reshape(-1,4)
            y = j['Mode'].to_numpy()
            X_list.append(X)
            y_list.append(y)
            
        X = np.vstack(X_list)
        y = np.hstack(y_list)

        self.X_train, self.X_test, self.y_train, self.y_test = train_test_split(X, y, test_size=0.30, random_state=20)

    def training(self):
        """ Function to train and compile model.
        """
        scaler = MinMaxScaler()
        self.X_train = scaler.fit_transform(self.X_train)
        self.X_test = scaler.transform(self.X_test)

        y_train_target = to_categorical(self.y_train)      

        self.model.add(Dense(200, input_dim=4, activation='relu'))    
        self.model.add(Dropout(0.25))                                
        self.model.add(Dense(190, activation='relu'))
        self.model.add(Dropout(0.30))
        self.model.add(Dense(150, activation='relu'))
        self.model.add(Dropout(0.30))
        self.model.add(Dense(100, activation='relu'))
        self.model.add(Dropout(0.30))
        self.model.add(Dense(50, activation='relu'))
        self.model.add(Dropout(0.30))
        self.model.add(Dense(50, activation='relu'))
        self.model.add(Dropout(0.25))
        self.model.add(Dense(5, activation='softmax'))

        self.model.compile(optimizer='adam', loss='categorical_crossentropy')

        self.model.fit(self.X_train, y_train_target, validation_split=0.1, epochs=70)


    def prediction(self, value):
        """ Function to predict class value.

        Args:
            value : the input features to predict on

        Returns:
            pred : the predicted class given the input
        """
        pred = np.argmax(self.model.predict(value), axis=1)

        return pred

    def main(self):
        """ Main function to read in data and train model.
        """
        self.read_data()
        self.training()


if __name__ == "__main__":
    """ Call of NET class in order to fit and train.

    Serializes the model to be used.
    """
    net = NET()
    net.main()
    filename = './Trained_Models/NET.sav'
    pickle.dump(net, open(filename, 'wb'))
