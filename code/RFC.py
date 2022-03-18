"""
Random Forest algorithm to classify finger flexion based on EMG readings.
Hyperparameters found in Classification Notes. Refer to the notes to find 
new hyperparameters as needed.

Implements scikit-learn model.

Training: Use to fit model using training data

Prediction: Use to predict class values

"""

import pandas as pd
import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import train_test_split
import os
import glob
import pickle


class RFC():
    """
    Class to implement RFC model.
    """
    def __init__(self, n_estimators = 90, max_depth = 10, max_features = 'auto'):
        self.n_estimators = n_estimators
        self.max_depth = max_depth
        self.max_features = max_features
        
        self.rfc = RandomForestClassifier(n_estimators=self.n_estimators, max_depth=self.max_depth, max_features=self.max_features, random_state=1)

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
        self.rfc.fit(self.X_train, self.y_train)

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
        probs = self.rfc.predict_proba(self.X_test)
        preds = self.rfc.predict(self.X_test)
        score = self.rfc.score(self.X_test, self.y_test)

        pred = self.rfc.predict(value)

        return pred, preds, score, probs
        # return preds, score, probs

    def main(self):
        """ Main function to read in data and train model.
        """
        self.read_data()
        self.training()


if __name__ == "__main__":
    """ Call of RFC class in order to fit and train.

    Can either serialize the trained model or print out the score for the test data.
    """
    rfc = RFC()
    rfc.main()
    filename = './Trained_Models/RFC.sav'
    pickle.dump(rfc, open(filename, 'wb'))
    # preds, score, _ = rfc.prediction(0)
    # print(f"Score: {score}")
