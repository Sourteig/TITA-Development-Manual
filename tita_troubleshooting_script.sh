#!/bin/bash

# TITA ROS2 Troubleshooting Script
# Basiert auf dem Diagnose- und Lösungsbericht für das TITA-Robotersystem

set -e

# Farben für Output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Konfiguration
ROBOT_IP=""
TOWER_IP=""
ROS_DOMAIN_ID=45

# Funktionen
print_header() {
    echo -e "${BLUE}================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}================================${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

# IP-Adressen abfragen
get_ip_addresses() {
    print_header "IP-Konfiguration"
    
    if [ -z "$ROBOT_IP" ]; then
        read -p "Bitte geben Sie die IP-Adresse des TITA-Roboters ein: " ROBOT_IP
    fi
    
    if [ -z "$TOWER_IP" ]; then
        read -p "Bitte geben Sie die IP-Adresse des Towers ein: " TOWER_IP
    fi
    
    print_info "Roboter IP: $ROBOT_IP"
    print_info "Tower IP: $TOWER_IP"
}

# Layer 3 Konnektivität testen
test_layer3_connectivity() {
    print_header "Layer 3 Konnektivitätstests"
    
    echo "Teste Ping zum Roboter..."
    if ping -c 3 "$ROBOT_IP" >/dev/null 2>&1; then
        print_success "Ping zum Roboter erfolgreich"
    else
        print_error "Ping zum Roboter fehlgeschlagen"
        return 1
    fi
    
    echo "Teste Ping zum Tower..."
    if ping -c 3 "$TOWER_IP" >/dev/null 2>&1; then
        print_success "Ping zum Tower erfolgreich"
    else
        print_error "Ping zum Tower fehlgeschlagen"
        return 1
    fi
    
    echo "Teste SSH-Verbindung zum Roboter..."
    if ssh -o ConnectTimeout=5 -o BatchMode=yes "$ROBOT_IP" exit 2>/dev/null; then
        print_success "SSH-Verbindung zum Roboter erfolgreich"
    else
        print_warning "SSH-Verbindung zum Roboter fehlgeschlagen (möglicherweise Authentifizierung erforderlich)"
    fi
}

# ROS2-Umgebung konfigurieren
configure_ros2_environment() {
    print_header "ROS2-Umgebungskonfiguration"
    
    # Überprüfen, ob ROS2 installiert ist
    if ! command -v ros2 &> /dev/null; then
        print_error "ROS2 ist nicht installiert oder nicht im PATH"
        return 1
    fi
    
    # Aktuelle Umgebungsvariablen anzeigen
    print_info "Aktuelle ROS2-Umgebungsvariablen:"
    echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-nicht gesetzt}"
    echo "ROS_LOCALHOST_ONLY: ${ROS_LOCALHOST_ONLY:-nicht gesetzt}"
    echo "LC_NUMERIC: ${LC_NUMERIC:-nicht gesetzt}"
    
    # Umgebung konfigurieren
    export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
    export ROS_LOCALHOST_ONLY=0
    export LC_NUMERIC="en_US.UTF-8"
    
    print_success "ROS2-Umgebung konfiguriert"
    print_info "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    print_info "ROS_LOCALHOST_ONLY=0"
    print_info "LC_NUMERIC=en_US.UTF-8"
}

# ROS2-Kommunikation testen
test_ros2_communication() {
    print_header "ROS2-Kommunikationstests"
    
    print_info "Teste Multicast-Empfang (5 Sekunden)..."
    timeout 5 ros2 multicast receive 2>/dev/null | head -10 || print_warning "Keine Multicast-Nachrichten empfangen"
    
    print_info "Verfügbare ROS2-Knoten:"
    ros2 node list 2>/dev/null || print_error "Keine ROS2-Knoten gefunden"
    
    print_info "Verfügbare ROS2-Topics:"
    ros2 topic list 2>/dev/null || print_error "Keine ROS2-Topics gefunden"
    
    # Wichtige Topics überprüfen
    local important_topics=("/scan" "/tf" "/odom" "/camera/image_raw" "/map")
    for topic in "${important_topics[@]}"; do
        if ros2 topic list 2>/dev/null | grep -q "^$topic$"; then
            print_success "Topic $topic gefunden"
        else
            print_warning "Topic $topic nicht gefunden"
        fi
    done
}

# Sensordaten validieren
validate_sensor_data() {
    print_header "Sensordaten-Validierung"
    
    # Lidar-Daten testen
    if ros2 topic list 2>/dev/null | grep -q "/scan"; then
        print_info "Teste Lidar-Daten (/scan)..."
        if timeout 5 ros2 topic echo /scan --once >/dev/null 2>&1; then
            print_success "Lidar-Daten empfangen"
        else
            print_error "Keine Lidar-Daten empfangen"
        fi
    fi
    
    # TF-Daten testen
    if ros2 topic list 2>/dev/null | grep -q "/tf"; then
        print_info "Teste TF-Daten (/tf)..."
        if timeout 5 ros2 topic echo /tf --once >/dev/null 2>&1; then
            print_success "TF-Daten empfangen"
        else
            print_error "Keine TF-Daten empfangen"
        fi
    fi
    
    # Kamera-Daten testen
    if ros2 topic list 2>/dev/null | grep -q "/camera/image_raw"; then
        print_info "Teste Kamera-Daten (/camera/image_raw)..."
        if timeout 5 ros2 topic echo /camera/image_raw --once >/dev/null 2>&1; then
            print_success "Kamera-Daten empfangen"
        else
            print_error "Keine Kamera-Daten empfangen"
        fi
    fi
}

# Roboter-Konfiguration reparieren
repair_robot_configuration() {
    print_header "Roboter-Konfiguration reparieren"
    
    if [ -z "$ROBOT_IP" ]; then
        print_error "Roboter-IP nicht gesetzt"
        return 1
    fi
    
    print_info "Versuche, die tita-bringup.service-Datei zu reparieren..."
    
    # SSH-Befehl zum Reparieren der Service-Datei
    ssh_command="
        sudo cp /lib/systemd/system/tita-bringup.service /lib/systemd/system/tita-bringup.service.backup &&
        sudo sed -i '/Environment=\"ROS_DOMAIN_ID=/d' /lib/systemd/system/tita-bringup.service &&
        sudo sed -i '/Environment=\"ROS_LOCALHOST_ONLY=/d' /lib/systemd/system/tita-bringup.service &&
        sudo sed -i '/\[Service\]/a Environment=\"ROS_DOMAIN_ID=$ROS_DOMAIN_ID\"' /lib/systemd/system/tita-bringup.service &&
        sudo sed -i '/\[Service\]/a Environment=\"ROS_LOCALHOST_ONLY=0\"' /lib/systemd/system/tita-bringup.service &&
        sudo systemctl daemon-reload &&
        sudo systemctl restart tita-bringup.service
    "
    
    if ssh "$ROBOT_IP" "$ssh_command"; then
        print_success "Roboter-Konfiguration erfolgreich repariert"
        print_info "Service wurde neu gestartet"
    else
        print_error "Fehler beim Reparieren der Roboter-Konfiguration"
        return 1
    fi
}

# Lokale Konfiguration erstellen
create_local_config() {
    print_header "Lokale Konfiguration erstellen"
    
    # Setup-Skript erstellen
    cat > setup_tita_env.sh << EOF
#!/bin/bash
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
export ROS_LOCALHOST_ONLY=0
export LC_NUMERIC="en_US.UTF-8"
echo "TITA-Umgebung konfiguriert"
echo "ROS_DOMAIN_ID=\$ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY=\$ROS_LOCALHOST_ONLY"
echo "LC_NUMERIC=\$LC_NUMERIC"
EOF
    
    chmod +x setup_tita_env.sh
    print_success "Setup-Skript 'setup_tita_env.sh' erstellt"
    
    # .bashrc erweitern
    if ! grep -q "export ROS_DOMAIN_ID=$ROS_DOMAIN_ID" ~/.bashrc; then
        echo "export ROS_DOMAIN_ID=$ROS_DOMAIN_ID" >> ~/.bashrc
        echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
        echo "export LC_NUMERIC=\"en_US.UTF-8\"" >> ~/.bashrc
        print_success "~/.bashrc erweitert"
    else
        print_info "~/.bashrc bereits konfiguriert"
    fi
}

# RViz-Konfiguration testen
test_rviz_configuration() {
    print_header "RViz-Konfiguration testen"
    
    if ! command -v rviz2 &> /dev/null; then
        print_error "RViz2 ist nicht installiert"
        return 1
    fi
    
    print_info "Erstelle RViz-Konfigurationsdatei..."
    
    # Basis-RViz-Konfiguration erstellen
    cat > tita_rviz_config.rviz << EOF
Panels:
  - Class: rviz_common/Tool Properties
    Name: Tool Properties
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Collision Enabled: false
      Description File: ""
      Description Source: Topic
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Enabled: true
      Links:
        All Links Enabled: true
        Expand Joint Details: false
        Expand Link Details: false
        Expand Tree: false
        Link Tree Style: Links in Alphabetic Order
      Name: RobotModel
      TF Prefix: ""
      Update Interval: 0
      Value: true
      Visual Enabled: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Default Light: true
    Fixed Frame: odom
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.785398006439209
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz_default_plugins)
      Yaw: 0.785398006439209
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 846
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State: 000000ff00000000fd000000040000000000000156000002b4fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d000002b4000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f000002b4fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000003d000002b4000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004420000003efc0100000002fb0000000800540069006d00650100000000000004420000000000000000fb0000000800540069006d006501000000000000045000000000000000000000023f000002b400000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: false
  Width: 1200
  X: 0
  Y: 0
EOF
    
    print_success "RViz-Konfigurationsdatei 'tita_rviz_config.rviz' erstellt"
    print_info "Starten Sie RViz mit: rviz2 -d tita_rviz_config.rviz"
}

# Vollständige Diagnose ausführen
run_full_diagnosis() {
    print_header "Vollständige TITA-Systemdiagnose"
    
    get_ip_addresses
    configure_ros2_environment
    
    if test_layer3_connectivity; then
        test_ros2_communication
        validate_sensor_data
    else
        print_error "Grundlegende Netzwerkverbindung fehlgeschlagen"
        return 1
    fi
    
    create_local_config
    test_rviz_configuration
    
    print_header "Diagnose abgeschlossen"
    print_info "Verwenden Sie 'source setup_tita_env.sh' in neuen Terminals"
    print_info "Starten Sie RViz mit: rviz2 -d tita_rviz_config.rviz"
}

# Hauptmenü
show_menu() {
    echo -e "${BLUE}TITA ROS2 Troubleshooting Tool${NC}"
    echo "1. Vollständige Diagnose ausführen"
    echo "2. Netzwerkverbindung testen"
    echo "3. ROS2-Kommunikation testen"
    echo "4. Sensordaten validieren"
    echo "5. Roboter-Konfiguration reparieren"
    echo "6. Lokale Konfiguration erstellen"
    echo "7. RViz-Konfiguration testen"
    echo "8. Beenden"
    echo
    read -p "Wählen Sie eine Option (1-8): " choice
}

# Hauptprogramm
main() {
    while true; do
        show_menu
        case $choice in
            1)
                run_full_diagnosis
                ;;
            2)
                get_ip_addresses
                test_layer3_connectivity
                ;;
            3)
                configure_ros2_environment
                test_ros2_communication
                ;;
            4)
                configure_ros2_environment
                validate_sensor_data
                ;;
            5)
                get_ip_addresses
                repair_robot_configuration
                ;;
            6)
                create_local_config
                ;;
            7)
                test_rviz_configuration
                ;;
            8)
                print_info "Programm beendet"
                exit 0
                ;;
            *)
                print_error "Ungültige Option"
                ;;
        esac
        echo
        read -p "Drücken Sie Enter, um fortzufahren..."
    done
}

# Wenn das Skript direkt ausgeführt wird
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi