import pandas as pd
import numpy as np
from sklearn.neighbors import KNeighborsClassifier
from sklearn.model_selection import train_test_split
import os
import glob
import pickle


class KNN():
    def __init__(self, n_neighbors = 23, weights = 'uniform', metric = 'manhattan'):
        self.n_neighbors = n_neighbors
        self.weights = weights
        self.metric = metric
    
        self.knn = KNeighborsClassifier(n_neighbors=self.n_neighbors, weights=self.weights, metric=self.metric)

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
        
        self.knn.fit(self.X_train, self.y_train)


    def prediction(self, value):
        probs = self.knn.predict_proba(self.X_test)
        preds = self.knn.predict(self.X_test)
        score = self.knn.score(self.X_test, self.y_test)

        pred = self.knn.predict(value)

        return pred, preds, score, probs
        # return preds, score, probs

    def main(self):
        self.read_data()
        self.training()


if __name__ == "__main__":
    knn = KNN()
    knn.main()
    filename = './Trained_Models/KNN.sav'
    pickle.dump(knn, open(filename, 'wb'))
    # preds, score, _ = knn.prediction(0)
    # print(f"Score: {score}")
