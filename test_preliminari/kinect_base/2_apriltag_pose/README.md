# 2 — AprilTag 6DOF pose (Kinect v1 RGB)

Script standalone: apre la camera RGB della Kinect, rileva i tag **tag36h11** con
`pupil_apriltags`, calcola la posa 6DOF con `cv2.solvePnP` (metodo `SOLVEPNP_IPPE_SQUARE`)
e disegna sul video, alla massima velocità possibile (~30 fps, limite hardware):

- contorno verde del tag + etichetta `ID n  distanza m`
- assi del tag: **X rosso** (destra), **Y verde** (su), **Z blu** (esce dal tag verso la camera)
- contatore FPS in alto a sinistra

In console stampa (5 volte/s) ID, traslazione `t` in metri, `rvec` in gradi e roll/pitch/yaw.

## Installazione

```bash
sudo apt install freenect libfreenect-dev
source ~/.venvs/halo_vr_game/bin/activate
pip install -r ~/Documents/halo_vr_game/test_preliminari/kinect_base/2_apriltag_pose/requirements.txt
```

Il modulo Python `freenect` è **opzionale**: se presente viene usato
(`freenect.sync_get_video`), altrimenti lo script usa `libfreenect_sync` via ctypes.

## 1. Genera e stampa un tag

```bash
cd ~/Documents/halo_vr_game/test_preliminari/kinect_base/2_apriltag_pose
python apriltag_pose.py --generate 0     # crea tag36h11_id0.png
```

Stampa in bianco e nero senza ridimensionare, incolla su cartoncino piatto e **misura il lato del
quadrato nero** (bordo nero incluso, margine bianco escluso): è il valore di `--tag-size` in metri.

> Il PNG è generato nell'orientamento **ufficiale** AprilTag. I tag creati dal vecchio
> `april_tag_detection/genera_tag.py` (OpenCV) sono ruotati di 180°: vengono rilevati lo stesso,
> ma X e Y risultano invertiti (Z resta corretto).

## 2. Avvia

```bash
python apriltag_pose.py --tag-size 0.10    # tag con quadrato nero da 10 cm
```

Esci con **q**, **Esc** o chiudendo la finestra.

| Opzione | Default | Significato |
|---------|---------|-------------|
| `--tag-size` | 0.05 | lato del quadrato nero in metri (sbagliarlo scala la distanza) |
| `--axis-len` | 0.03 | lunghezza assi disegnati in metri |
| `--decimate` | 1.0 | `quad_decimate`: 2.0 = più veloce, portata minore |
| `--threads` | 4 | thread del detector |
| `--print-hz` | 5 | frequenza stampa in console (0 = off) |
| `--index` | 0 | indice Kinect |
| `--opencv N` | — | usa la webcam OpenCV N al posto della Kinect (test senza Kinect) |
| `--generate ID` | — | salva il PNG del tag ed esce |

Intrinseci usati (Kinect v1 RGB 640×480): `fx = fy = 525`, `cx = 319.5`, `cy = 239.5`,
distorsione nulla. Si modificano in cima al file.

## Cosa verificare

1. FPS ~30 con un tag in vista.
2. Avvicinando/allontanando il tag la distanza cambia coerentemente (verifica con un metro).
3. Ruotando il tag gli assi lo seguono; Z blu punta sempre verso la camera.
4. Più tag con ID diversi vengono rilevati contemporaneamente.

## Problemi comuni

| Sintomo | Soluzione |
|---------|-----------|
| `freenect_sync_get_video failed` | alimentatore + USB, chiudi `freenect-glview` e la dashboard (`1_kinect_dashboard`) |
| `No module named pupil_apriltags` | `pip install pupil-apriltags` |
| Distanza sbagliata di un fattore costante | `--tag-size` non corrisponde al quadrato nero stampato |
| Assi X/Y ruotati di 180° | tag generato con OpenCV: rigeneralo con `--generate` |
| FPS bassi | `--decimate 2` |
