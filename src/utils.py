import json
import serial
import struct

import numpy as np

def recuperer_de_json(nom_fichier):
    with open(nom_fichier, 'r') as fichier:
        donnees = json.load(fichier)
    return donnees

def sauvegarder_en_json(nom_fichier, liste):
    with open(nom_fichier, 'w+') as fichier:
        json.dump(liste, fichier)
    print(f"path du fichier: {nom_fichier}")

# Exemple d'utilisation
config_file_path = 'config.json'
config = recuperer_de_json(config_file_path)

# Accès aux valeurs du fichier de configuration
folders = config['folders']

# Configuration du port série
port = config['serial']['port']
baudrate = config['serial']['baudrate']
ser = serial.Serial(port, baudrate, timeout=1) 


# Structure des données
data_format = config['data']['format']
data_begin = config['data']['begin'].to_bytes(1, 'big')
data_end = config['data']['end'].to_bytes(1, 'big')
data_sampling = config['data']['sampling']
data_time = config['data']['time']
data_len = (data_time * 1000) / data_sampling
data = []

# Labels
label_dict = config['labels']
prompt_dict = config['prompts']

def Rx_accel_data():
    # Lecture des données depuis le port série
    SOH = ser.read(1)
    if SOH == data_begin:
        raw_data = SOH + ser.read(struct.calcsize(data_format) - 1)

        # Décodage des données en utilisant la structure
        tram = struct.unpack(data_format, raw_data)

        # Check le header
        if tram[-1].to_bytes(1, 'big') == data_end:
            return tram[1:-1]
    return Rx_accel_data()


def histogramme(data_x, label=None) :
    # Calculer la transformée de Fourier
    fft_x = np.fft.fft(data_x)
    fft_freqs = np.fft.fftfreq(len(fft_x), d=1)  # fréquences associées à la transformée de Fourier

    # Trier les indices pour un tracé correct
    idx = np.argsort(fft_freqs)

    # Sélectionner les fréquences positives seulement
    positive_freqs = fft_freqs[idx][fft_freqs[idx] >= 0]
    positive_fft = np.abs(fft_x)[idx][fft_freqs[idx] >= 0]

    # Discrétiser en 20 plages
    num_bins = 20
    hist, _ = np.histogram(positive_freqs, bins=num_bins, weights=positive_fft)

    if label:
        hist = np.append(hist, label_dict[label])

    return hist[1:]