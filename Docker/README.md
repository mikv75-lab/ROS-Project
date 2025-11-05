# Thesis Dev Env (MoveIt2 Rolling + GZ Ionic + ros_gz_bridge)

## Voraussetzungen
- Docker Desktop (Windows)
- XLaunch / VcXsrv (mit "Disable access control" gestartet)
- (Optional) NVIDIA Container Toolkit für GPU
- build:
    docker compose build
    bash ./volume_init.sh
    # CPU
    docker compose --profile cpu up -d
    docker exec -it thesis-cpu zsh
    # GPU
    # docker compose --profile gpu up -d
    # docker exec -it thesis-gpu zsh



## 1) Build
```powershell
cd C:\Users\miklv\thesis-docker
docker compose build

2) Volume initialisieren

bash ./volume_init.sh

Dieser Schritt legt im Volume die Ordnerstruktur an:

/data
 ├─ app
 └─ ros_ws
    ├─ src
    ├─ build
    ├─ install
    └─ log

3) Starten (CPU-Only, überall lauffähig)

docker compose --profile cpu up -d

3b) Starten (GPU, nur auf NVIDIA-PCs)

docker compose --profile gpu up -d

    Tipp: Wenn thesis-gpu mit deploy: nicht greift, ersetze in docker-compose.yml den deploy-Block durch gpus: all.

4) In den Container

docker exec -it thesis bash

4a) (Optional) Eigene App-venv aktivieren

source /data/app/venv/bin/activate  # falls vorhanden

5) ROS2 Workspace vorbereiten

cd /data/ros_ws
# Repos klonen, z.B.:
# git clone https://github.com/ros-planning/moveit2_tutorials.git -b main src/moveit2_tutorials
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
# Dank Autosource ist install/setup.bash künftig automatisch geladen

6) Tests
RViz2

rviz2

Gazebo (gz) Ionic leeres Projekt

gz sim -v 2

ROS <-> GZ Bridge check

    Bridge-Package ist installiert (ros-rolling-ros-gz)

    Beispiel: Topics auflisten

gz topic -l
ros2 topic list

7) Container stoppen/entfernen (Volume bleibt erhalten!)

docker compose down

8) Backups des Volumes
Backup erstellen

# PowerShell im Projektordner:
docker run --rm -v workspace_data:/data -v ${PWD}:/backup alpine sh -c "tar -czf /backup/workspace_backup_$(date +%Y%m%d_%H%M%S).tar.gz -C / data"

Backup wiederherstellen

# Ersetzt Inhalte des Volumes:
docker run --rm -v workspace_data:/data -v ${PWD}:/backup alpine sh -c "cd /data && tar -xzf /backup/<DEIN_BACKUP>.tar.gz --strip 1"

    Hinweis: Auf Windows kann ${PWD} je nach Shell variieren. Alternativ -v C:\Users\miklv\thesis-docker:/backup angeben.

9) Nützlich

    DISPLAY wird via host.docker.internal:0.0 gesetzt – stelle sicher, dass XLaunch läuft (Disable access control).

    Falls Fenster nicht erscheinen: Firewall-Regeln für VcXsrv prüfen.

    Falls mehrere Container/PCs: ROS_DOMAIN_ID anpassen.


---

## Quick-Start (Kurzfassung)

```powershell
cd C:\Users\miklv\thesis-docker
docker compose build
bash ./volume_init.sh
# CPU (überall):
docker compose --profile cpu up -d
# ODER GPU:
docker compose --profile gpu up -d
docker exec -it thesis bash

# Im Container:
cd /data/ros_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
rviz2
gz sim -v 2

Backup & Restore (alternativ, ohne Powershell-Var)

Backup nach aktuelles Verzeichnis:

docker run --rm -v workspace_data:/data -v "$(pwd)":/backup alpine \
  sh -c 'tar -czf /backup/workspace_backup.tar.gz -C / data'

Restore:

docker run --rm -v workspace_data:/data -v "$(pwd)":/backup alpine \
  sh -c 'cd /data && tar -xzf /backup/workspace_backup.tar.gz --strip 1'