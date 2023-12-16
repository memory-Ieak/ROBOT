import json
import serial
import struct

import numpy as np

# Configuration du port série
port = "/dev/ttyACM0"
baudrate = 115200

# Configuration du port série
ser = serial.Serial(port, baudrate, timeout=1) 

# Structure des données
data_format = '=BfffB'  # 2 bytes et 3 flottants non alignés

data = []
data_len = 2680

def Rx_accel_data():
    # Lecture des données depuis le port série
    SOH = ser.read(1)
    if SOH == b'\x01':
        raw_data = SOH + ser.read(struct.calcsize(data_format) - 1)

        # Décodage des données en utilisant la structure
        _, accX, accY, accZ, eot = struct.unpack(data_format, raw_data)

        # Check le header
        if eot.to_bytes(1, 'big') == b'\x04':
            return accX, accY, accZ
    return Rx_accel_data()
        


label_dict = {
    "human": 0,
    "normal": 1,
    "obstacle": 2
}

def histogramme(data_x, label="normal") :
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
    hist = np.append(hist, label_dict[label])
    return hist[1:]


def recuperer_de_json(nom_fichier):
    with open(nom_fichier, 'r') as fichier:
        donnees = json.load(fichier)
    return donnees

def sauvegarder_en_json(nom_fichier, liste):
    with open(nom_fichier, 'w+') as fichier:
        json.dump(liste, fichier)
    print(f"path du fichier: {nom_fichier}")