import pandas as pd
import numpy as np
from sklearn.neighbors import KNeighborsClassifier
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import train_test_split, cross_val_score, GridSearchCV
from sklearn.metrics import confusion_matrix
from sklearn.neural_network import MLPClassifier

df1 = pd.read_csv('./data/Combined_EMG_data1.csv')
df2 = pd.read_csv('./data/Combined_EMG_data2.csv')
df3 = pd.read_csv('./data/Combined_EMG_data3.csv')
df4 = pd.read_csv('./data/Combined_EMG_data4.csv')
df5 = pd.read_csv('./data/Combined_EMG_data5.csv')
df6 = pd.read_csv('./data/Combined_EMG_data6.csv')
df7 = pd.read_csv('./data/Combined_EMG_data7.csv')
df8 = pd.read_csv('./data/Combined_EMG_data8.csv')
df9 = pd.read_csv('./data/Combined_EMG_data9.csv')
df10 = pd.read_csv('./data/Combined_EMG_data10.csv')

# separate the features and the labels into X and y variables
X1 = df1[['EMG Sensor 1', 'EMG Sensor 2', 'EMG Sensor 3', 'EMG Sensor 4']].to_numpy().reshape(-1,4)
X2 = df2[['EMG Sensor 1', 'EMG Sensor 2', 'EMG Sensor 3', 'EMG Sensor 4']].to_numpy().reshape(-1,4)
X3 = df3[['EMG Sensor 1', 'EMG Sensor 2', 'EMG Sensor 3', 'EMG Sensor 4']].to_numpy().reshape(-1,4)
X4 = df4[['EMG Sensor 1', 'EMG Sensor 2', 'EMG Sensor 3', 'EMG Sensor 4']].to_numpy().reshape(-1,4)
X5 = df5[['EMG Sensor 1', 'EMG Sensor 2', 'EMG Sensor 3', 'EMG Sensor 4']].to_numpy().reshape(-1,4)
X6 = df6[['EMG Sensor 1', 'EMG Sensor 2', 'EMG Sensor 3', 'EMG Sensor 4']].to_numpy().reshape(-1,4)
X7 = df7[['EMG Sensor 1', 'EMG Sensor 2', 'EMG Sensor 3', 'EMG Sensor 4']].to_numpy().reshape(-1,4)
X8 = df8[['EMG Sensor 1', 'EMG Sensor 2', 'EMG Sensor 3', 'EMG Sensor 4']].to_numpy().reshape(-1,4)
X9 = df9[['EMG Sensor 1', 'EMG Sensor 2', 'EMG Sensor 3', 'EMG Sensor 4']].to_numpy().reshape(-1,4)
X10 = df10[['EMG Sensor 1', 'EMG Sensor 2', 'EMG Sensor 3', 'EMG Sensor 4']].to_numpy().reshape(-1,4)
y1 = df1['Mode'].to_numpy()
y2 = df2['Mode'].to_numpy()
y3 = df3['Mode'].to_numpy()
y4 = df4['Mode'].to_numpy()
y5 = df5['Mode'].to_numpy()
y6 = df6['Mode'].to_numpy()
y7 = df7['Mode'].to_numpy()
y8 = df8['Mode'].to_numpy()
y9 = df9['Mode'].to_numpy()
y10 = df10['Mode'].to_numpy()


X = np.vstack((X1, X2, X3, X4, X5, X6, X7, X8, X9, X10))
y = np.hstack((y1, y2, y3, y4, y5, y6, y7, y8, y9, y10))

# Separate the data into training and test sets
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.30, random_state=42)


##### Classifier #####
MLP = MLPClassifier(hidden_layer_sizes=(200,), activation='relu', solver='adam', random_state=1, max_iter=800)
MLP.fit(X_train, y_train)

probs = MLP.predict_proba(X_test)
preds = MLP.predict(X_test)
score = MLP.score(X_test, y_test)

print(f"Score: {score}")


index = np.array([0.9, 0.1, 0.4, 0.5]).reshape(-1,4)
pred = MLP.predict(index)
print(pred)

middle = np.array([1.6, 0.2, 1.5, 1.6]).reshape(-1,4)
pred = MLP.predict(middle)
print(pred)

fourth = np.array([1.9, 0.3, 0.6, 1.1]).reshape(-1,4)
pred = MLP.predict(fourth)
print(pred)

pinky = np.array([0.6, 0.2, 0.6, 1.1]).reshape(-1,4)
pred = MLP.predict(pinky)
print(pred)