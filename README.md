# halo_vr_game — Kinect v1 Test Suite

## Struttura
```
test_preliminari/kinect_base/
├── 1_kinect_dashboard/   # Webapp Flask: tutte le funzionalità Kinect v1
└── 2_apriltag_pose/      # AprilTag detection con overlay assi 6DOF
```

## Setup sistema (una volta sola, sul NUC)
```bash
sudo apt update
sudo apt install -y libfreenect-dev freenect python3-freenect libudev-dev portaudio19-dev
# Regola udev per usare Kinect senza sudo
sudo adduser $USER plugdev
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="045e", MODE="0666", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/51-kinect.rules
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE="0666", GROUP="plugdev"' | sudo tee -a /etc/udev/rules.d/51-kinect.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
# Logout/login per applicare gruppo plugdev
```

## Demo 1 — Kinect Dashboard (webapp locale)

**Feature:** RGB, Depth, IR stream · tilt motore · LED · accelerometro real-time · direzione microfono

```bash
cd ~/Documents/halo_vr_game
git pull origin main
cd test_preliminari/kinect_base/1_kinect_dashboard
pip install -r requirements.txt
python app.py
# Apri il browser su http://localhost:5000
```

## Demo 2 — AprilTag Pose (finestra OpenCV)

**Feature:** detect tag36h11 · overlay assi X/Y/Z · posa 6DOF · FPS counter

```bash
cd ~/Documents/halo_vr_game
git pull origin main
cd test_preliminari/kinect_base/2_apriltag_pose
pip install -r requirements.txt
python apriltag_pose.py
# Premi Q per uscire
# Fallback webcam: python apriltag_pose.py --opencv 0
```

## Troubleshooting rapido
| Problema | Soluzione |
|---|---|
| `freenect: cannot find libfreenect` | `sudo apt install libfreenect-dev` |
| `usb_open error: insufficient permissions` | Aggiungi udev rules (vedi Setup), logout/login |
| `No Kinect device found` | `lsusb` → cerca `045e:02ae`; prova `sudo freenect-glview` |
| `pyaudio: No module named _portaudio` | `sudo apt install portaudio19-dev && pip install pyaudio` |
| AprilTag: `cannot import pupil_apriltags` | `pip install pupil-apriltags` |
| Dashboard lenta | Abbassa JPEG quality in `kinect_streams.py` (default 85→70) |
