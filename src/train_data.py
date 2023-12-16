#!/usr/bin/env python3

import os
import pandas as pd

from sklearn.model_selection import train_test_split

from sklearn.ensemble import RandomForestClassifier

from utils import *

def train_model():
    # Créer un DataFrame avec les noms de colonnes
    noms_colonnes = [f'Range{i}' for i in range(1, 20)] + ['Label']
    df = pd.DataFrame(columns=noms_colonnes)


    folders = ["human", "normal", "obstacle"]
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

    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)



    ## Entraîner un modèle (exemple avec une forêt aléatoire)
    model = RandomForestClassifier()
    model.fit(X_train, y_train)

    return model