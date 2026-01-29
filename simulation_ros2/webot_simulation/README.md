# 🚗 Autonomous Navigation – LiDAR + Depth Camera (ROS 2)

## 📌 Description

Ce projet implémente une **navigation autonome robuste** basée sur la **fusion d’une caméra depth et d’un LiDAR**, développée et testée sous **ROS 2 avec Webots**.

L’objectif est de permettre à un véhicule autonome :

- de **suivre un parcours**,
- d’**anticiper les virages** grâce à la caméra depth,
- d’**éviter les obstacles** avec le LiDAR,
- tout en garantissant une **sécurité intelligente** (arrêt d’urgence uniquement en ligne droite).

Le comportement est optimisé pour être :

- fluide,
- stable,
- sans oscillations,
- et capable de **terminer entièrement le circuit**.

---

## 📂 Arborescence (extrait)

```text
webot_simulation/
├── obstacle_avoider.py
├── teleop.py
├── vision_lane_follower.py
├── voiture_driver.py
├── __init__.py
└── README.md
```
# Vision Lane Follower

## 📁 Fichier principal
**`vision_lane_follower.py`**

## ⚙️ Dépendances
- **ROS 2** ( Jazzy)
- **Webots**
- **Python 3**
- **Packages ROS** : 
  - `rclpy`
  - `sensor_msgs`
  - `ackermann_msgs`
  - `cv_bridge`
  - `numpy`

## 🛠️ Compilation du workspace
Depuis la racine du workspace ROS 2 :
```bash
cd ~/covapsy_ws
colcon build
source install/setup.bash
```
## ▶️ Lancement de la simulation
**Lancer Webots** (selon le setup habituel) apres 
**Lancer la navigation autonome :**
```bash
ros2 run webot_simulation vision_lane_follower.py
```
## 🧠 Principe de fonctionnement

### 1️⃣ Caméra depth – Anticipation locale
L'image depth est découpée en trois zones : gauche, centre et droite. Pour chaque zone, on calcule la distance médiane.

**Cela permet :**
- D'anticiper les virages
- De commencer à tourner avant d'être face à un mur

### 2️⃣ Commande de direction (steering)
Le système utilise deux niveaux de décision :

**Anticipation douce :**

```python
steering = -k_side * (depth_left - depth_right)
```
#### Décision forte en présence d’obstacle

Lorsque la distance mesurée devant le véhicule devient inférieure à un seuil prédéfini, une décision plus agressive est appliquée afin d’éviter une collision :

```python
if depth_center < depth_threshold:
    if depth_left > depth_right:
        steering = -steering_gain
    else:
        steering = +steering_gain
```
Un **lissage exponentiel** est ensuite appliqué sur la commande de direction afin d’éviter les oscillations et les changements brusques :

```python
steering = steer_smooth * prev_steering + (1.0 - steer_smooth) * steering
```

### 3️⃣ LiDAR – Sécurité intelligente

Le LiDAR est utilisé comme capteur de sécurité afin de prévenir les collisions frontales.  
Il surveille une **zone frontale étroite** (±10°) devant le véhicule.

L’arrêt d’urgence est déclenché **uniquement si** :
- un obstacle est détecté à très courte distance,
- **et** le véhicule est quasi en ligne droite (pas en virage).

```python
if lidar_front < lidar_stop_dist and abs(steering) < 0.12:
    lidar_stop_counter += 1
else:
    lidar_stop_counter = 0
```
Après plusieurs détections consécutives, la vitesse est annulée :

```python
if lidar_stop_counter >= lidar_stop_count_req:
    speed = 0.0
```
Cette logique permet d’éviter les faux arrêts causés par les murs latéraux dans les virages.


