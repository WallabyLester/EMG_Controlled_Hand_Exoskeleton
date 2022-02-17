import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

df = pd.read_csv('./data/Combined_EMG_data1.csv')
x_list = np.arange(df.size)
y = pd.DataFrame(df['EMG Sensor 1']).to_numpy()
y_list = np.zeros(len(y))

for i in range(len(y)):
    y_list[i] = y[i]

plt.figure(figsize=(20,15))
plt.title('EMG Data 6', fontsize=20)
plt.xlabel('Time (s)', fontsize=15)
plt.ylabel('Voltage', fontsize=15)
plt.xlim(0, 100)
plt.xticks(np.arange(0, 100, 5))
plt.yticks(np.arange(0, 2, 0.2))
plt.plot(x_list, y_list)
# plt.savefig("./plots/EMG_data6.png")
plt.show()
