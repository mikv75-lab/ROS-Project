# 🚀 Docker Backup & Restore für Thesis Workspace

Dieses Projekt enthält ein PowerShell-Skript (`docker-backup.ps1`), um den
aktuellen ROS2/MoveIt2 Docker-Container **thesis-gpu** und das dazugehörige Docker Volume **thesis_root** zu sichern und auf einem anderen Rechner wiederherzustellen.

---

## ✅ Voraussetzungen

- Windows PowerShell
- Docker installiert (`docker --version`)
- PS-Skript entsperren, falls nötig:
  ```powershell
  Set-ExecutionPolicy -Scope Process Bypass

📦 Backup erstellen

Sichert:
✅ Containerzustand (inkl. ROS, MoveIt, Config, Tools)
✅ Volume (Workspace, Code, Einstellungen)

.\docker-backup.ps1 -Action backup -ContainerName thesis-gpu -VolumeName thesis_root

Backup wird im Ordner backup/ gespeichert:

backup/
 ├─ thesis-gpu-image-YYYYMMDD-HHMMSS.tar.gz
 └─ thesis_root-volume-YYYYMMDD-HHMMSS.tar.gz

🔄 Restore auf beliebigem Rechner
1. Backup-Dateien übertragen

Kopiere den Backup-Ordner auf den Zielrechner und stelle sicher, dass Docker läuft.
2. Container-Image wiederherstellen

.\docker-backup.ps1 -Action restore -BackupFile .\backup\thesis-gpu-image-*.tar.gz

3. Volume wiederherstellen

.\docker-backup.ps1 -Action restore -BackupFile .\backup\thesis_root-volume-*.tar.gz

4. Container wieder starten

docker run -it --gpus all --name thesis-gpu --volume thesis_root:/root thesis-gpu-backup:latest

    Der Container hat danach alle Daten & Configs wie beim Backup und du kannst direkt weiterarbeiten.

🛠 Optionen
Aktion	Beschreibung	Beispiel
Backup	Container + Volume sichern	.\docker-backup.ps1 -Action backup -ContainerName thesis-gpu -VolumeName thesis_root
Restore Image	Container wiederherstellen	.\docker-backup.ps1 -Action restore -BackupFile .\backup\thesis-gpu-image-123.tar.gz
Restore Volume	Daten wiederherstellen	.\docker-backup.ps1 -Action restore -BackupFile .\backup\thesis_root-volume-123.tar.gz