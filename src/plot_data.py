#!/usr/bin/env python3

import matplotlib.pyplot as plt

from utils import *

fig, ax = plt.subplots()

ax.set_title('Données d\'accélération en temps réel')
ax.set_xlabel('Temps')
ax.set_ylabel('Accélération X')

x_step = 0.005
x_values = np.arange(0, data_len * x_step, x_step)

print(f"plot toutes les {data_len * 0.005}s")


try:
    while True:
        try:
            accx, _, _ = Rx_accel_data()
        except:
            continue

        data.append(accx)

        if len(data) == data_len:
            ax.clear()
            ax.plot(x_values, data)

            ax.set_ylim(0, 2)

            data.clear()

except KeyboardInterrupt:
    plt.close()