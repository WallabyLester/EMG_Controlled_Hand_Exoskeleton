import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.neural_network import MLPClassifier
import os
import glob
import pickle


class MLP():
    def __init__(self, hidden_layer_sizes = (200,), activation='relu', solver='adam', random_state=1, max_iter = 1200):
        self.hidden_layer_sizes = hidden_layer_sizes
        self.activation = activation
        self.solver = solver
        self.random_state = random_state
        self.max_iter = max_iter
        self.mlp = MLPClassifier(hidden_layer_sizes = self.hidden_layer_sizes, activation = self.activation, solver = self.solver, random_state = self.random_state, max_iter = self.max_iter)

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

        self.X_train, self.X_test, self.y_train, self.y_test = train_test_split(X, y, test_size=0.20, random_state=42)

    def training(self):
        
        self.mlp.fit(self.X_train, self.y_train)


    def prediction(self, value):
        probs = self.mlp.predict_proba(self.X_test)
        preds = self.mlp.predict(self.X_test)
        score = self.mlp.score(self.X_test, self.y_test)

        pred = self.mlp.predict(value)

        return pred, preds, score, probs

    def main(self):
        self.read_data()
        self.training()


if __name__ == "__main__":
    mlp = MLP()
    mlp.main()
    filename = './Trained_Models/MLP.sav'
    pickle.dump(mlp, open(filename, 'wb'))
    # _, preds, score, _ = mlp.prediction(0)
    # print(f"Score: {score}")
