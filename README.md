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
    │   ├── kinect.py          # wrapper libfreenect (ctypes)
    │   └── test_kinect.py     # test RGB, profondità e IR
    └── april_tag_detection/
        ├── genera_tag.py      # genera tag36h11_id0.png
        └── rileva_tag.py      # rilevamento live via Kinect
```

> Il file `tag36h11_id0.png` è generato localmente con `genera_tag.py` e **non** è tracciato da git.

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

## Test 1 — Kinect base (RGB + Profondità + IR)

### Prerequisiti
- Alimentatore della Kinect collegato alla presa **e** cavo USB collegato al NUC.
- Nessun altro programma che usa la Kinect (es. `freenect-glview`) deve essere aperto.

### Verifica rapida opzionale
```bash
lsusb -d 045e:
```
Devono comparire 2–3 righe Microsoft (camera, motor, audio).

```bash
freenect-glview    # prova visiva veloce — chiudilo prima di lanciare Python
```

### Esegui il test
```bash
source ~/.venvs/halo_vr_game/bin/activate
cd ~/Documents/halo_vr_game/test_preliminari/kinect_base

# RGB + mappa di profondità
python test_kinect.py

# Solo infrarosso
python test_kinect.py --mode ir
```

### Cosa guardare
| Modalità | Cosa deve succedere |
|----------|---------------------|
| `rgbd` | Finestra RGB fluida + mappa di profondità colorata che cambia muovendo la mano |
| `ir` | Immagine grigia con pattern di puntini, reagisce alla scena |

Ogni 2 secondi il terminale stampa le iterazioni/s (misura il ciclo Python, non il frame rate hardware).

**Chiudi** con **Q** o **Esc**.

### Problemi comuni
| Sintomo | Causa probabile | Soluzione |
|---------|-----------------|-----------|
| `lsusb` non mostra Microsoft | Kinect non alimentata o USB non collegata | Controlla cavi |
| `LIBUSB_ERROR_ACCESS` | Mancano le regole udev | `sudo apt install libfreenect-dev` poi scollega/ricollega |
| `Libreria freenect_sync non trovata` | libfreenect non installata | `sudo apt install libfreenect-dev` |
| Nessun frame / dispositivo occupato | `freenect-glview` aperto | Chiudi glview prima di eseguire Python |

---

## Test 2 — AprilTag detection (tag36h11, ID 0)

### Passo 1 — Genera il tag PNG
```bash
source ~/.venvs/halo_vr_game/bin/activate
cd ~/Documents/halo_vr_game/test_preliminari/april_tag_detection
python genera_tag.py
```
Viene creato `tag36h11_id0.png` nella stessa cartella.

### Passo 2 — Stampa il tag
- Stampa `tag36h11_id0.png` in **bianco e nero**, senza ridimensionare.
- Il quadrato nero deve essere circa **10 cm** di lato.
- Incolla il foglio su un **cartoncino piatto** per evitare deformazioni.

### Passo 3 — Rilevamento live
```bash
python rileva_tag.py
```

### Cosa guardare
- **Contorno verde** attorno al tag rilevato.
- **Punto giallo** al centro geometrico.
- In alto: `ID 0 | centro: x=..., y=... px`

### Test di robustezza
1. Avvicina e allontana il tag: le coordinate `y` cambiano.
2. Sposta a destra/sinistra: `x` cambia.
3. Inclinalo di 45°: deve continuare a rilevarlo.
4. Coprilo completamente: la label deve diventare `ID 0 non rilevato`.
5. Scoprilo: il contorno e le coordinate ricompaiono subito.

---

## Comandi utili rapidi

| Azione | Comando |
|--------|---------|
| Attiva venv | `source ~/.venvs/halo_vr_game/bin/activate` |
| Verifica Kinect USB | `lsusb -d 045e:` |
| Prova rapida Kinect | `freenect-glview` |
| Test RGB + Depth | `cd ~/Documents/halo_vr_game/test_preliminari/kinect_base && python test_kinect.py` |
| Test IR | `python test_kinect.py --mode ir` |
| Genera AprilTag PNG | `cd ~/Documents/halo_vr_game/test_preliminari/april_tag_detection && python genera_tag.py` |
| Rileva AprilTag live | `python rileva_tag.py` |
| Aggiorna repo | `cd ~/Documents/halo_vr_game && git pull origin main` |

---

## Prossimi passi

1. **Calibrazione camera RGB** — acquisire i parametri intrinseci (lunghezza focale, centro ottico, distorsione) con una scacchiera, necessari per la stima della posa 3D.
2. **Stima posa 3D** — una volta calibrata, `cv2.solvePnP` restituisce posizione e orientamento del tag nello spazio.
3. **Allineamento RGB–Depth** — registrare i due flussi per usare la profondità nelle coordinate RGB.
4. **Integrazione nel gioco** — portare il tracking nel motore VR.
