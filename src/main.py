#!/usr/bin/env python3

import numpy as np
from joblib import load

from utils import *
from train_data import train_model

model = train_model()

try:
    while True:
        accx, _, _ = Rx_accel_data()

        data.append(accx)

        if len(data) > data_len:
            histo = histogramme(data)[:-1]
            data.clear()
            num_features = 19
            X_new = np.reshape(histo, (1, num_features))
            print(model.predict(X_new))

except KeyboardInterrupt:
    pass