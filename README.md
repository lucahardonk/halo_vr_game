# halo_vr_game — Test Preliminari

Gioco VR/AR in sviluppo. Questa cartella contiene i test preliminari per verificare
l'integrazione della **Kinect v1** (via libfreenect) e il rilevamento degli **AprilTag**
tramite OpenCV, prima di integrare il tutto nel gioco.

---

## Struttura del repository

```
halo_vr_game/
├── README.md
├── .gitignore
└── test_preliminari/
    ├── requirements.txt
    ├── kinect_base/
    │   ├── 1_kinect_dashboard/   # webapp Flask: RGB, IR, depth, tilt, LED, accelerometro, microfoni
    │   └── 2_apriltag_pose/      # posa 6DOF AprilTag con assi X/Y/Z sul video Kinect
    └── april_tag_detection/      # (legacy) vecchi script di rilevamento
```

> `april_tag_detection/rileva_tag.py` importava il vecchio `kinect_base/kinect.py`, ora rimosso:
> usa `kinect_base/2_apriltag_pose/` al suo posto.

---

## Setup ambiente (NUC — nuc@nuc-NUC13ANHi7)

### 1. Scarica il repo
```bash
cd ~/Documents/halo_vr_game
git pull origin main
ls test_preliminari
```
Devi vedere: `requirements.txt`, `kinect_base/`, `april_tag_detection/`.

### 2. Dipendenze di sistema
```bash
sudo apt update
sudo apt install freenect libfreenect-dev python3-venv
```

### 3. Crea il venv (esterno al repo)
```bash
python3 -m venv ~/.venvs/halo_vr_game
source ~/.venvs/halo_vr_game/bin/activate
pip install -r ~/Documents/halo_vr_game/test_preliminari/requirements.txt
```

---

## Test 1 — Dashboard Kinect completa

Istruzioni complete: [`test_preliminari/kinect_base/1_kinect_dashboard/README.md`](test_preliminari/kinect_base/1_kinect_dashboard/README.md)

```bash
sudo apt install portaudio19-dev kinect-audio-setup   # poi scollega/ricollega la Kinect
source ~/.venvs/halo_vr_game/bin/activate
cd ~/Documents/halo_vr_game/test_preliminari/kinect_base/1_kinect_dashboard
pip install -r requirements.txt
python app.py        # apri http://localhost:5000
```

---

## Test 2 — AprilTag pose 6DOF

Istruzioni complete: [`test_preliminari/kinect_base/2_apriltag_pose/README.md`](test_preliminari/kinect_base/2_apriltag_pose/README.md)

```bash
source ~/.venvs/halo_vr_game/bin/activate
cd ~/Documents/halo_vr_game/test_preliminari/kinect_base/2_apriltag_pose
pip install -r requirements.txt
python apriltag_pose.py --generate 0     # stampa il PNG, misura il quadrato nero
python apriltag_pose.py --tag-size 0.10
```

---

## Comandi utili rapidi

| Azione | Comando |
|--------|---------|
| Attiva venv | `source ~/.venvs/halo_vr_game/bin/activate` |
| Verifica Kinect USB | `lsusb -d 045e:` |
| Verifica microfoni Kinect | `arecord -l` |
| Dashboard Kinect | `cd ~/Documents/halo_vr_game/test_preliminari/kinect_base/1_kinect_dashboard && python app.py` |
| Genera AprilTag | `cd ~/Documents/halo_vr_game/test_preliminari/kinect_base/2_apriltag_pose && python apriltag_pose.py --generate 0` |
| AprilTag pose | `python apriltag_pose.py --tag-size 0.10` |
| Aggiorna repo | `cd ~/Documents/halo_vr_game && git pull origin main` |

---

## Prossimi passi

1. **Calibrazione camera RGB** — acquisire i parametri intrinseci (lunghezza focale, centro ottico, distorsione) con una scacchiera, necessari per la stima della posa 3D.
2. **Stima posa 3D** — una volta calibrata, `cv2.solvePnP` restituisce posizione e orientamento del tag nello spazio.
3. **Allineamento RGB–Depth** — registrare i due flussi per usare la profondità nelle coordinate RGB.
4. **Integrazione nel gioco** — portare il tracking nel motore VR.
