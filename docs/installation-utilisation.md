# Installation et utilisation sur Raspberry Pi

## Portée

Le point d'entrée est `software/dashboard.py`. Il initialise des GPIO, le ventilateur, la caméra et des boucles d'acquisition dès son lancement. Il ne s'exécute donc pas comme une simple application web sur un ordinateur sans matériel. La procédure ci-dessous est une base de reprise, **non validée sur une image Raspberry Pi OS neuve**.

La [documentation Raspberry Pi OS](https://www.raspberrypi.com/documentation/computers/os.html#use-python-on-a-raspberry-pi) recommande `apt` pour les paquets système et un environnement virtuel pour les dépendances `pip`. La [documentation caméra officielle](https://www.raspberrypi.com/documentation/computers/camera_software.html#use-libcamera-from-python-with-picamera2) décrit l'installation de Picamera2 via `apt`.

## Préparer le Pi

1. Vérifier le [câblage réel](electronique-cablage.md), en particulier le HX711 et l'étage ventilateur, avant de mettre le banc sous tension.
2. Installer Raspberry Pi OS et activer caméra, I²C et SPI selon les périphériques réellement présents. Vérifier la caméra avec les outils du système avant de lancer le tableau de bord.
3. Installer les modules système nécessaires, par exemple `python3-picamera2`, `python3-opencv`, `python3-numpy`, `python3-matplotlib`, `python3-rpi.gpio`, `python3-spidev`, `python3-venv` et `ffmpeg` pour les conversions vidéo. La disponibilité et la compatibilité de ces paquets dépendent de l'image choisie.
4. Depuis la racine de ce dépôt, créer l'environnement puis installer les dépendances Python restantes :

   ```sh
   python3 -m venv --system-site-packages .venv
   . .venv/bin/activate
   python -m pip install -r software/requirements.txt
   ```

5. Depuis `software/`, vérifier les valeurs de `config.py`, puis lancer `python dashboard.py`. Le répertoire de données est `software/data/` par défaut ; `SOUFFLERIE_DATA_DIR` permet de le déplacer. Les exports et calibrations restent locaux et sont ignorés par Git.

## Accès et sécurité réseau

Par défaut, le serveur écoute sur `127.0.0.1:8080`. Sur le Pi, ouvrir `http://127.0.0.1:8080`. Depuis un poste de confiance, un tunnel SSH permet de conserver cette écoute locale :

```sh
ssh -L 8080:127.0.0.1:8080 utilisateur@adresse-du-pi
```

Ouvrir ensuite `http://127.0.0.1:8080` sur le poste. Pour une démonstration sur un réseau isolé, l'opérateur peut définir `SOUFFLERIE_BIND_HOST=0.0.0.0` avant le lancement. **L'application n'a ni authentification ni chiffrement HTTP** ; cet accès donne aux autres clients du réseau la possibilité de commander le ventilateur, changer la calibration, agir sur des fichiers de session et voir le flux caméra. Ne pas l'exposer à Internet ou à un réseau partagé non maîtrisé. La configuration Wi-Fi depuis l'interface est désactivée par défaut ; elle demande `SOUFFLERIE_ENABLE_WIFI_CONTROL=1` et des droits NetworkManager configurés sur le Pi. Aucun mot de passe administrateur n'est stocké dans le code.

## Utiliser et enregistrer

1. Contrôler l'arrêt initial du ventilateur, l'image caméra et les indicateurs d'erreur.
2. Faire une tare et vérifier l'échelle de la cellule avec une référence connue, sans extrapoler de précision à partir d'une seule masse.
3. Vérifier séparément les estimations de vitesse. Les constantes de `config.py` sont des paramètres de départ, pas un étalonnage du banc assemblé.
4. Commander progressivement le ventilateur, surveiller les pièces libres et enregistrer une session.
5. À la fin, ramener la consigne à zéro, arrêter l'enregistrement et exporter les données.

Les dépendances déclarées par le projet historique utilisaient notamment des versions Python plus anciennes ; elles n'ont pas été réinstallées ni testées sur un Pi pour cette copie. `software/requirements.txt` ne contient que `smbus2`, utilisé pour l'I²C et à installer dans l'environnement virtuel ; les autres imports principaux sont fournis par les paquets système indiqués ci-dessus. Le module `hx711` historique a été retiré des dépendances : le tableau de bord lit le composant par GPIO directement.
