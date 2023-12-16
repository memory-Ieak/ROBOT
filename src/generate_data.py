#!/usr/bin/env python3

import sys
import argparse

from utils import *
from train_data import train_model

filename = "data"
datafolder = "human"

# Initialisation de l'objet ArgumentParser
parser = argparse.ArgumentParser(description='Script de collecte de données et de prédiction avec modèle.')

# Ajout des arguments en ligne de commande
parser.add_argument('--filename', default="data", help='Nom du fichier pour sauvegarder les données.')
parser.add_argument('--label', default="normal", help='Dossier pour sauvegarder les données.')
parser.add_argument('--index', type=int, default=0, help='Indice de départ pour les fichiers.')

args = parser.parse_args()

# Utilisation des arguments
filename = args.filename
datafolder = args.label
i = args.index

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