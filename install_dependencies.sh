#!/bin/bash

# Vérifier si Python 3 est installé
if ! command -v python3 &> /dev/null; then
    echo "Python 3 n'est pas installé. Veuillez installer Python 3 avant de continuer."
    exit 1
fi

# Installer les bibliothèques Python nécessaires
python3 -m pip install --upgrade pip
python3 -m pip install numpy matplotlib scikit-learn

chmod +x src/*.py

echo "Les bibliothèques ont été installées avec succès."