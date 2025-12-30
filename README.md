# COVAPSy Simulation Environment — Sorbonne Université 2025

Ce dépôt contient l’environnement de **simulation du cours COVAPSy** (Conception et Validation de Systèmes Cyber-Physiques), proposé à **Sorbonne Université en 2025**.

Il fournit une plateforme complète basée sur **Webots** et **ROS 2**, permettant de **concevoir, tester et visualiser des algorithmes de navigation autonome** pour un véhicule de type voiture.

---

##  Objectif du projet

L’objectif principal de ce projet est de mettre en place un **simulateur réaliste de voiture autonome**, proche des conditions rencontrées sur un véhicule réel.

Dans un premier temps, le travail a consisté à :

- Mettre en place une **architecture Webots + ROS 2 fonctionnelle**
- Modéliser un véhicule de type **TT02** avec une cinématique **Ackermann**
- Intégrer plusieurs **capteurs réalistes**
- Publier correctement les **topics ROS 2**, l’**odométrie** et les **transformations TF**
- Préparer le simulateur pour :
  - la navigation autonome
  - l’évitement d’obstacles
  - le SLAM (cartographie)
  - la perception par caméra

Ce simulateur sert de **base expérimentale** pour les TPs et projets du cours COVAPSy.

---

##  Description du simulateur

Le véhicule simulé est une **voiture de type TT02**, intégrée dans Webots avec :

- une dynamique réaliste
- une direction Ackermann
- une interface directe avec ROS 2 via un **contrôleur Webots personnalisé**

Chaque véhicule est **namespacé** (ex. `/TT02_jaune/...`), ce qui permet une extension future vers des scénarios multi-robots.

---

##  Capteurs intégrés

###  LiDAR
- **RpLidar A2**
- Utilisé pour l’évitement d’obstacles et le SLAM
- Topics :
  - `/TT02_jaune/RpLidarA2`
  - `/TT02_jaune/RpLidarA2/point_cloud`

###  Caméra RGB-D (type Qualcomm RB5)
Une caméra inspirée du **kit Qualcomm RB5** a été intégrée sur le toit du véhicule.

- **Caméra RGB**
  - `/TT02_jaune/rb5_rgb/image_color`
  - `/TT02_jaune/rb5_rgb/camera_info`

- **Caméra Depth**
  - `/TT02_jaune/rb5_depth/image`
  - `/TT02_jaune/rb5_depth/point_cloud`

Cette caméra permet de travailler sur :
- la perception visuelle
- le suivi de ligne
- la navigation basée vision
- des approches futures en deep learning

###  Autres capteurs
- IMU / Gyroscope
- Capteur sonar arrière

---

##  Prérequis système

###  Système d’exploitation
- **Ubuntu 24.04 LTS (recommandé)**

###  ROS 2
- **ROS 2 Jazzy **

Installation officielle :  
https://docs.ros.org/en/jazzy/Installation.html

## Installer les outils nécessaires :

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  git
```
---


##  Installation

### 1. Initialisation de rosdep
À faire une seule fois avant la compilation :

```bash
sudo rosdep init
rosdep update
```
### 2. Installer les dépendances ROS 2 supplémentaires
Assurez-vous d'être sous ROS 2 Jazzy.
```bash
sudo apt install -y \
  ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-bringup \
  ros-jazzy-navigation2 \
  ros-jazzy-rviz2 \
  ros-jazzy-rqt-image-view \
  ros-jazzy-tf2-tools
```
### 3. Création du workspace et clonage
```bash
# Création du dossier src
mkdir -p ~/covapsy_ws/src

# Accéder au dossier et cloner le dépôt
cd ~/covapsy_ws/src
git clone https://github.com/Ferras007/covapsy_simulation.git
```
### 4. Installation des dépendances du projet
```bash
cd ~/covapsy_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 5.Compilation
```bash
cd ~/covapsy_ws
colcon build

# Sourcer l'environnement
source install/setup.bash
```
⚠️ Le sourcing (`source install/setup.bash`) doit être refait dans chaque nouveau terminal.

##  Lancer le simulateur

Ouvrez un terminal et exécutez :
```bash
source /opt/ros/jazzy/setup.bash
source ~/covapsy_ws/install/setup.bash

ros2 launch webot_simulation simulation.launch.py
```
## Visualisation:
### Caméra RGB (RQT):
Pour voir le flux vidéo :
```bash
ros2 run rqt_image_view rqt_image_view
```



