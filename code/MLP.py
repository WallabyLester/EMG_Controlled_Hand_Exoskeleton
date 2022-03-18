"""
Multilayer Perceptron algorithm to classify finger flexion based on EMG readings.
Hyperparameters found in Classification Notes. Refer to the notes to find 
new hyperparameters as needed.

Implements scikit-learn model.

Training: Use to fit model using training data

Prediction: Use to predict class values

"""

import pandas as pd
import numpy as np
from sklearn.neural_network import MLPClassifier
from sklearn.model_selection import train_test_split
import os
import glob
import pickle


class MLP():
    """
    Class to implement MLP model.
    """
    def __init__(self, hidden_layer_sizes = (200,200,200,), activation ='relu', solver ='adam', learning_rate = 'constant', random_state = 1, max_iter = 1200):
        self.hidden_layer_sizes = hidden_layer_sizes
        self.activation = activation
        self.solver = solver
        self.learning_rate = learning_rate
        self.random_state = random_state
        self.max_iter = max_iter
        self.mlp = MLPClassifier(hidden_layer_sizes = self.hidden_layer_sizes, activation = self.activation, solver = self.solver, learning_rate=self.learning_rate, random_state = self.random_state, max_iter = self.max_iter)

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
        """ Function to implement fit functionality.
        """
        self.mlp.fit(self.X_train, self.y_train)

    def prediction(self, value):
        """ Function to predict class value.

        Can predict and give probabilities and mean accuracy score. Use second 
        return if you would only like to view accuracy 

        Args:
            value : the input features to predict on

        Returns:
            pred : the predicted class given the input
            preds : the predictions for the test data
            score : the mean accuracy score for the test data
            probs : the probabilities for the test data
        """
        probs = self.mlp.predict_proba(self.X_test)
        preds = self.mlp.predict(self.X_test)
        score = self.mlp.score(self.X_test, self.y_test)

        pred = self.mlp.predict(value)

        return pred, preds, score, probs
        # return preds, score, probs

    def main(self):
        """ Main function to read in data and train model.
        """
        self.read_data()
        self.training()


if __name__ == "__main__":
    """ Call of MLP class in order to fit and train.

    Can either serialize the trained model or print out the score for the test data.
    """
    mlp = MLP()
    mlp.main()
    filename = './Trained_Models/MLP.sav'
    pickle.dump(mlp, open(filename, 'wb'))
    # preds, score, _ = mlp.prediction(0)
    # print(f"Score: {score}")
