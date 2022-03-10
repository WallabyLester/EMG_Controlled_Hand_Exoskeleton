import pandas as pd
import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import train_test_split
import os
import glob
import pickle


class RFC():
    def __init__(self, n_estimators = 27, max_depth = 15):
        self.n_estimators = n_estimators
        self.max_depth = max_depth
        
        self.rfc = RandomForestClassifier(n_estimators=self.n_estimators, max_depth=self.max_depth, random_state=1)

    def read_data(self):
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

        self.X_train, self.X_test, self.y_train, self.y_test = train_test_split(X, y, test_size=0.20, random_state=20)

    def training(self):
        
        self.rfc.fit(self.X_train, self.y_train)


    def prediction(self, value):
        probs = self.rfc.predict_proba(self.X_test)
        preds = self.rfc.predict(self.X_test)
        score = self.rfc.score(self.X_test, self.y_test)

        pred = self.rfc.predict(value)

        return pred, preds, score, probs
        # return preds, score, probs

    def main(self):
        self.read_data()
        self.training()


if __name__ == "__main__":
    rfc = RFC()
    rfc.main()
    filename = './Trained_Models/RFC.sav'
    pickle.dump(rfc, open(filename, 'wb'))
    # preds, score, _ = rfc.prediction(0)
    # print(f"Score: {score}")
