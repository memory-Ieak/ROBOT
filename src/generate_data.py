#!/usr/bin/env python3

import sys

from utils import *
from train_data import train_model

filename = "data"
datafolder = "human"

if len(sys.argv) > 1:
    filename = sys.argv[1]

if len(sys.argv) > 2:
    datafolder = sys.argv[2]

model = train_model()

try:
    i = 0
    while True:
        accx, _, _ = Rx_accel_data()

        data.append(accx)

        if len(data) > data_len:
            sauvegarder_en_json(f"../data/{datafolder}/" + filename + str(i) + ".json", data)
            data.clear()

            histo = histogramme(recuperer_de_json(f"../data/{datafolder}/" + filename + str(i) + ".json"))[:-1]
            num_features = 19
            X_new = np.reshape(histo, (1, num_features))
            print(model.predict(X_new))

            i += 1

except KeyboardInterrupt:
    pass