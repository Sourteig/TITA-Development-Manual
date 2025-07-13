# TITA ROS2 Troubleshooting Guide

Dieses Repository enthält eine umfassende Analyse und praktische Tools zur Diagnose und Behebung von Konnektivitäts- und Datenflussproblemen im TITA ROS2-Ökosystem.

## Dateien im Repository

### 1. `TITA_ROS2_Diagnose_Analyse.md`
Eine detaillierte technische Analyse der häufigsten Probleme mit dem TITA-Robotersystem, einschließlich:
- Systematische Problemdiagnose
- Lösungsansätze für ROS2-DDS-Middleware-Probleme
- RViz-Konfiguration und Troubleshooting
- Best Practices für Multi-Host-ROS2-Deployments

### 2. `tita_troubleshooting_script.sh`
Ein interaktives Bash-Skript für automatisierte Diagnose und Problemlösung mit folgenden Funktionen:
- Netzwerkverbindung testen
- ROS2-Kommunikation validieren
- Sensordaten überprüfen
- Roboter-Konfiguration reparieren
- Lokale Umgebung einrichten
- RViz-Konfiguration erstellen

## Schnellstart

### Voraussetzungen
- ROS2 (Humble/Iron/Rolling) installiert
- SSH-Zugang zum TITA-Roboter
- Grundlegende Linux-Kenntnisse

### Verwendung des Troubleshooting-Skripts

1. **Skript ausführbar machen:**
   ```bash
   chmod +x tita_troubleshooting_script.sh
   ```

2. **Vollständige Diagnose ausführen:**
   ```bash
   ./tita_troubleshooting_script.sh
   ```
   Wählen Sie Option 1 für eine komplette Systemdiagnose.

3. **Manuelle Schritte für häufige Probleme:**

   **Problem: ROS2-Knoten nicht sichtbar**
   ```bash
   # Umgebung konfigurieren
   export ROS_DOMAIN_ID=45
   export ROS_LOCALHOST_ONLY=0
   export LC_NUMERIC="en_US.UTF-8"
   
   # Verbindung testen
   ros2 node list
   ros2 topic list
   ```

   **Problem: Roboter-Service reparieren**
   ```bash
   # SSH zum Roboter
   ssh user@<robot_ip>
   
   # Service-Datei bearbeiten
   sudo nano /lib/systemd/system/tita-bringup.service
   
   # Im [Service]-Abschnitt hinzufügen:
   Environment="ROS_DOMAIN_ID=45"
   Environment="ROS_LOCALHOST_ONLY=0"
   
   # Service neu starten
   sudo systemctl daemon-reload
   sudo systemctl restart tita-bringup.service
   ```

   **Problem: RViz zeigt nichts an**
   ```bash
   # RViz mit korrekter Locale starten
   export LC_NUMERIC="en_US.UTF-8"
   rviz2 -d tita_rviz_config.rviz
   ```

## Diagnostik-Workflow

```
1. Netzwerk-Ping → 2. ROS2-Knoten-Erkennung → 3. Topic-Validierung → 4. Sensordaten-Test → 5. RViz-Visualisierung
```

## Häufige Probleme und Lösungen

| Problem | Ursache | Lösung |
|---------|---------|---------|
| ROS2-Knoten nicht sichtbar | `ROS_DOMAIN_ID` oder `ROS_LOCALHOST_ONLY` falsch konfiguriert | Umgebungsvariablen korrigieren |
| Mapping-Service hängt | Keine Lidar-Daten empfangen | Netzwerkverbindung und `/scan` Topic prüfen |
| RViz zeigt weißes Robotermodell | LC_NUMERIC-Bug | `LC_NUMERIC="en_US.UTF-8"` setzen |
| Keine Kamerabilder | QoS-Inkompatibilität | QoS auf "Best Effort" ändern |

## Automatisierte Umgebungseinrichtung

Nach dem Ausführen des Skripts werden folgende Dateien erstellt:

- `setup_tita_env.sh`: Umgebung für neue Terminals konfigurieren
- `tita_rviz_config.rviz`: Vorkonfigurierte RViz-Einstellungen

**Verwendung:**
```bash
# In jedem neuen Terminal
source setup_tita_env.sh

# RViz starten
rviz2 -d tita_rviz_config.rviz
```

## Erweiterte Diagnose

### DDS-Multicast testen
```bash
# Terminal 1: Multicast-Empfang
ros2 multicast receive

# Terminal 2: Multicast-Senden
ros2 multicast send
```

### Sensordaten-Latenz messen
```bash
ros2 topic hz /scan
ros2 topic hz /camera/image_raw
```

### Firewall-Konfiguration (falls erforderlich)
```bash
sudo ufw allow 7400:7412/udp
```

## Support und Fehlerbehebung

Bei anhaltenden Problemen:

1. Führen Sie das Troubleshooting-Skript aus
2. Überprüfen Sie die Systemlogs: `journalctl -u tita-bringup.service`
3. Konsultieren Sie die detaillierte Analyse in `TITA_ROS2_Diagnose_Analyse.md`

## Lizenz

Dieses Troubleshooting-Tool ist für die Verwendung mit TITA-Robotersystemen optimiert und kann unter MIT-Lizenz frei verwendet werden.



