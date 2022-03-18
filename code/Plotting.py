"""
Plots time series EMG readings for each sensor and the modes during the timeframe.

Modes:
    0 - no flexion
    1 - index finger flexed
    2 - middle finger flexed
    3 - fourth finger flexed 
    4 - pinky finger flexed

Use line: `savefig` in order to save the plot.

"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read individual data files, change path for whichever data file desired to plot
df = pd.read_csv('./data/Combined_EMG_data1.csv')
y1 = pd.DataFrame(df['EMG Sensor 1']).to_numpy()
y2 = pd.DataFrame(df['EMG Sensor 2']).to_numpy()
y3 = pd.DataFrame(df['EMG Sensor 3']).to_numpy()
y4 = pd.DataFrame(df['EMG Sensor 4']).to_numpy()
mode = pd.DataFrame(df['Mode']).to_numpy()
x_list = np.arange(len(y1))
y_list1 = np.zeros(len(y1))
y_list2 = np.zeros(len(y2))
y_list3 = np.zeros(len(y3))
y_list4 = np.zeros(len(y4))
mode_list = np.zeros(len(mode))

for i in range(len(y1)):
    y_list1[i] = y1[i]

for i in range(len(y2)):
    y_list2[i] = y2[i]

for i in range(len(y3)):
    y_list3[i] = y3[i]

for i in range(len(y4)):
    y_list4[i] = y4[i]

for i in range(len(mode)):
    mode_list[i] = mode[i]

fig, ax = plt.subplots(5, 1, sharex=True, figsize=(15,15))
ax[0].plot(x_list, y_list1, label='EMG Sensor 1')
ax[1].plot(x_list, y_list2, label='EMG Sensor 2')
ax[2].plot(x_list, y_list3, label='EMG Sensor 3')
ax[3].plot(x_list, y_list4, label='EMG Sensor 4')
ax[4].plot(x_list, mode_list, label='Mode')

ax[0].set_title('EMG Data', fontsize=20)
ax[4].set_xlabel('Time (s)', fontsize=15)
ax[0].set_ylabel('Voltage (V)', fontsize=15)
ax[1].set_ylabel('Voltage (V)', fontsize=15)
ax[2].set_ylabel('Voltage (V)', fontsize=15)
ax[3].set_ylabel('Voltage (V)', fontsize=15)
ax[4].set_ylabel('Mode', fontsize=15)

ax[0].set_xlim(0, 100)
ax[0].set_xticks(np.arange(0, 100, 5))

for ax in plt.gcf().axes:
    ax.legend(bbox_to_anchor=(1, 1),loc='upper left')

# plt.savefig("./plots/Combined_EMG_data10.png")
plt.show()
