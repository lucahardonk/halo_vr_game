# halo_vr_game — Kinect v1 Test Suite

Questo repo raccoglie i test preliminari per il progetto VR: stream video, profondità,
controllo motore, microfoni e rilevamento AprilTag con la Kinect v1.

## Struttura

```
test_preliminari/kinect_base/
├── 1_kinect_dashboard/   # Webapp Flask: tutte le funzionalità Kinect v1
└── 2_apriltag_pose/      # AprilTag detection con stima posa 6DOF
```

---

## Setup sistema — una volta sola (NUC, Ubuntu 24+)

### Pacchetti

```bash
sudo apt update
sudo apt install -y \
    libfreenect-dev freenect \
    portaudio19-dev \
    p7zip-full
```

### Firmware audio dei microfoni Kinect

```bash
# installa (può fallire nella postinst, è normale)
sudo apt install -y kinect-audio-setup || true

# calcola l'hash MD5 reale del file Microsoft e aggiorna lo script
URL=$(grep -o 'http[^" ]*\.msi' /usr/sbin/kinect_fetch_fw | head -1)
wget -q -O /tmp/kinect_sdk.msi "$URL"
MD5=$(md5sum /tmp/kinect_sdk.msi | cut -d' ' -f1)
sudo sed -i "s/^SDK_MD5=.*/SDK_MD5=\"$MD5\"/" /usr/sbin/kinect_fetch_fw
sudo dpkg --configure -a
ls /lib/firmware/kinect/   # deve comparire UACFirmware
rm /tmp/kinect_sdk.msi
```

### Regole udev (accesso Kinect senza sudo)

```bash
sudo tee /etc/udev/rules.d/51-kinect.rules > /dev/null << 'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02b0", MODE="0666", GROUP="plugdev"
EOF

sudo adduser $USER plugdev
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**Dopo questo passo: logout e login** (o riavvio).

### Prima connessione Kinect

Collega la Kinect **direttamente a una porta USB del PC**, senza hub né prolunghe.
Dopo il login scollega e ricollega la Kinect (udev carica il firmware audio al collegamento).
Verifica: `arecord -l` → deve comparire un dispositivo "Kinect".

### Setup Python

```bash
python3 -m venv ~/.venvs/halo_vr_game
source ~/.venvs/halo_vr_game/bin/activate
```

---

## Demo 1 — Kinect Dashboard

**Feature:** RGB · Depth · IR · tilt motore · LED · accelerometro real-time · direzione microfono

```bash
source ~/.venvs/halo_vr_game/bin/activate
cd ~/Documents/halo_vr_game/test_preliminari/kinect_base/1_kinect_dashboard
pip install -r requirements.txt
python app.py
# apri http://localhost:5000
```

Documentazione completa (opzioni CLI, API, troubleshooting): [`1_kinect_dashboard/README.md`](test_preliminari/kinect_base/1_kinect_dashboard/README.md)

---

## Demo 2 — AprilTag Pose

**Feature:** tag36h11 · overlay assi X/Y/Z · posa 6DOF · FPS counter

```bash
source ~/.venvs/halo_vr_game/bin/activate
cd ~/Documents/halo_vr_game/test_preliminari/kinect_base/2_apriltag_pose
pip install -r requirements.txt
python apriltag_pose.py           # Kinect
python apriltag_pose.py --opencv 0  # webcam di fallback
# premi Q per uscire
```

---

## Troubleshooting rapido

| Problema | Soluzione |
|----------|-----------|
| `freenect: cannot find libfreenect` | `sudo apt install libfreenect-dev` |
| `usb_open error: insufficient permissions` | udev rules + logout/login (vedi Setup) |
| `No Kinect device found` | `lsusb \| grep 045e`; prova `sudo freenect-glview` |
| `kinect-audio-setup` errore MD5 | ripeti la sezione "Firmware audio" (calcola l'MD5 dal file reale) |
| Microfoni non trovati dopo il firmware | scollega e ricollega la Kinect; `arecord -l`; `python app.py --list-audio` |
| Depth OK ma RGB nero/fermo | Kinect diretta al PC senza hub; `freenect-glview` per isolare; `python app.py --video-mode ir` |
| `pyaudio: No module named _portaudio` | `sudo apt install portaudio19-dev && pip install pyaudio` |
| `cannot import pupil_apriltags` | `pip install pupil-apriltags` |
| Dashboard lenta | abbassa JPEG quality in `kinect_streams.py` (default 85 → 70) |
