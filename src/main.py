#!/usr/bin/env python3

import argparse 
import numpy as np
import matplotlib.pyplot as plt

from datetime import datetime

from utils import *
from train_data import train_model

# Initialisation de l'objet ArgumentParser
parser = argparse.ArgumentParser(description='Script de collecte de données et de prédiction avec modèle.')

# Ajout de l'argument en ligne de commande pour activer ou désactiver le tracé
parser.add_argument('--plot', action='store_true', help='Activer le tracé des données.')
parser.add_argument('--val', action='store_true', help='Activer le print des données.')

# Récupération des arguments
args = parser.parse_args()

# Utilisation de l'argument
plot = args.plot
val = args.val

if plot:
    # Initialiser la figure et les deux sous-graphiques (subplots)
    fig, (ax1, ax2) = plt.subplots(2, 1)

    ax1.set_title('Données d\'accélération en temps réel')
    ax1.set_ylabel('Accélération X')

    ax2.set_xlabel('Fréquence')
    ax2.set_ylabel('Amplitude')

    x_step = 0.005
    x_values = np.arange(0, data_len * x_step, x_step)

print(f"utilisation du modèle toutes les {data_len * 0.005}s")

model = train_model()

dict = {
    0 : "manipulé par un humain",
    1 : "normal",
    2 : "a subit un choc",
}

try:
    while True:
        accx, _, _ = Rx_accel_data()

        if val:
            print(accx)

        data.append(accx)

        if len(data) == data_len:
            histo = histogramme(data)[:-1]

            if plot:
                # Plot de la data brut
                ax1.clear()
                ax1.plot(x_values, data)
                ax1.set_ylim(0, 2)

                # Plot de l'histogramme
                ax2.clear()
                ax2.bar(range(len(histo)), histo)
                ax2.set_ylim(0, 2000)

                plt.pause(0.1)

            data.clear()
            num_features = 19
            X_new = np.reshape(histo, (1, num_features))

            # Obtenez l'heure actuelle au format hh:mm:ss
            current_time = datetime.now().strftime('%H:%M:%S')
            
            # Affiche l'heure et l'état de sortie du modèle
            print(f"{current_time}: Le capteur est {dict[model.predict(X_new)[0]]}")

except KeyboardInterrupt:
    pass