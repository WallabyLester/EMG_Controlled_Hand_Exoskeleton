import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import find_peaks

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
# ax[0].set_ylim(0, 3)
# ax[1].set_ylim(0, 3)
# ax[2].set_ylim(0, 3)
# ax[3].set_ylim(0, 3)
# ax[0].set_yticks(np.arange(0, 2, 0.2))

for ax in plt.gcf().axes:
    ax.legend(bbox_to_anchor=(1, 1),loc='upper left')

# plt.savefig("./plots/Combined_EMG_data10.png")
plt.show()

# plt.figure(figsize=(20,15))
# plt.title('EMG Data 6', fontsize=20)
# plt.xlabel('Time (s)', fontsize=15)
# plt.ylabel('Voltage', fontsize=15)
# plt.xlim(0, 100)
# plt.xticks(np.arange(0, 100, 5))
# plt.yticks(np.arange(0, 2, 0.2))
# plt.plot(x_list, y_list)
# plt.savefig("./plots/EMG_data6.png")
# plt.show()
