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
