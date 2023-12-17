#!/usr/bin/env python3

import os
import argparse
import pandas as pd
import numpy as np

from sklearn.model_selection import train_test_split

from sklearn.ensemble import RandomForestClassifier

from utils import *

def train_model():
    # Créer un DataFrame avec les noms de colonnes
    noms_colonnes = [f'Range{i}' for i in range(1, 20)] + ['Label']
    df = pd.DataFrame(columns=noms_colonnes)


    i = 0
    for folder in folders:
        directory = os.fsencode(f'../data/{folder}')
        for file in os.listdir(directory):
            filename = os.fsdecode(file)

            dataf = recuperer_de_json(f'../data/{folder}/{filename}')
            df.loc[i] = histogramme(dataf, folder)
            i += 1


    
    df["Label"] = df["Label"].astype(int)


    X = df.drop("Label", axis=1).values
    y = df["Label"].values

    X_train, _, y_train, _ = train_test_split(X, y, test_size=0.2, random_state=42)



    ## Entraîner un modèle (exemple avec une forêt aléatoire)
    model = RandomForestClassifier()
    model.fit(X_train, y_train)

    return model

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Faire des prédictions avec un modèle.')
    parser.add_argument('--file_path', type=str, default='../data/normal/test0.json',
                        help='Chemin du fichier JSON de test')
    args = parser.parse_args()

    model = train_model()

    data = histogramme(recuperer_de_json(args.file_path))
    num_features = 19  # Replace with the correct number of features
    X_new = np.reshape(data, (1, num_features))


    print(model.predict(X_new))