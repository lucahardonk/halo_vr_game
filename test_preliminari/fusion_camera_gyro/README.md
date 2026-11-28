# fusion_camera_gyro — AprilTag (Kinect v1) + gyro IMU, orientamento fluido con slerp

Fonde il **tracking AprilTag** della Kinect v1 (dal test `mouse_control`) con il **giroscopio
dell'IMU** (Nano 33 BLE Sense Rev2, via BLE, dal test `bluetooth_test`).

L'idea concordata in fase di brainstorming:

- la **camera** da' la posa **assoluta** del tag, ma solo a ~**30 Hz**;
- il **gyro** dell'IMU — montata **solidale all'AprilTag** — propaga l'orientamento ad **alta
  frequenza** (~100–200 Hz) tra un frame camera e l'altro;
- a ogni frame camera la deriva accumulata dal gyro viene **ritirata con uno slerp dolce**
  (NON un reset secco) verso la posa assoluta.

Niente Madgwick e **niente magnetometro**: la camera e' la verita' a lungo termine, il gyro
da' la fluidita' a breve termine. Risultato: orientamento liscio e a bassa latenza che non
deriva nel tempo.

> IMU e tag sono considerati **lo stesso corpo rigido**: per questo test il frame del gyro
> coincide con quello del tag (nessuna trasformazione, `R_GYRO_TO_TAG = identita'` in
> `fusion.py`). Se un giorno monterai l'IMU ruotata rispetto al tag, basta cambiare quella
> matrice costante.

## Come funziona (due passi)

```
predict(gyro, t)   ad ogni campione IMU :  q <- q (x) dq(omega*dt)     [integrazione corpo-frame]
correct(q_cam, p_cam, t) ad ogni frame  :  q <- slerp(q, q_cam, gain)  [aggancio alla camera]
```

- `--slerp` (default **0.15**) e' il guadagno della correzione: **basso** = piu' liscio, piu'
  "gyro" tra i frame (ma recupera la verita' camera piu' lentamente); **alto** = segue la
  camera piu' da vicino (ma piu' a scatti). 0.15 e' un buon punto di partenza.
- Il **bias statico** del gyro viene stimato automaticamente ai primi ~150 campioni **tenendo
  la scheda ferma** all'avvio (vedi il messaggio "stima bias..." / "bias ok").
- Un **vuoto temporale** oltre 0.5 s (es. tag perso a lungo) non viene integrato, per non
  accumulare un salto.

## Lo ZERO (INVIO)

Quando premi **INVIO** registro la **posizione iniziale e l'orientamento correnti**: quello
diventa lo **0**. Da quel momento l'uscita e' **relativa** allo zero:

- orientamento relativo `R/P/Y` in gradi;
- posizione relativa `x/y/z` in metri, espressa nel frame dello zero.

Premi INVIO di nuovo in qualunque momento per ri-azzerare.

## File

| file | cosa fa |
|------|---------|
| `fusion_camera_gyro.py` | avvio: apre camera + IMU, ciclo predict/correct, stampa, INVIO=zero |
| `fusion.py` | `GyroCameraFusion`: predict (gyro), correct (slerp verso camera), set_zero, snapshot |
| `quaternion.py` | quaternioni Hamilton (solo numpy): multiply, slerp, matrice, rotvec, euler |
| `camera_tag.py` | `CameraSource` (Kinect v1 + AprilTag, da mouse_control) e `SyntheticCamera` |
| `imu_ble.py` | `ImuBle` (gyro via BlueZ/D-Bus, da bluetooth_test) e `SyntheticImu` |
| `test_fusion.py` | test della matematica, **senza hardware** (solo numpy) |

## Setup (sul NUC, Ubuntu)

Si usa il **venv unico** dei test preliminari. Dalla cartella `test_preliminari/`:

```bash
# pacchetti di sistema (una volta):
sudo apt install python3-venv libfreenect-dev bluez

cd test_preliminari
./setup_venv.sh                 # crea/aggiorna .venv e installa TUTTE le dipendenze
source .venv/bin/activate
```

(Le dipendenze di questo test — numpy, opencv-contrib-python, dbus-fast — sono gia' incluse
nel `requirements.txt` aggregato.)

## Uso reale (Kinect + IMU)

1. **Connetti l'IMU dalla GUI Bluetooth di Ubuntu** (come nel test `bluetooth_test`): lo
   script **non** scansiona e **non** si connette, trova via D-Bus il dispositivo `IMU_BTN`
   gia' connesso e attiva le notifiche.
2. Inquadra l'AprilTag con la Kinect. Tieni la **scheda ferma** qualche secondo (stima bias).
3. Avvia:

```bash
cd test_preliminari/fusion_camera_gyro
python3 fusion_camera_gyro.py --tag-size 0.05        # lato del quadrato nero del tag, in metri
# opzionale: --address 18:FE:03:D9:D4:4B  (MAC dell'IMU gia' connessa)
# opzionale: --tag-id 0   --index 0   --slerp 0.12   --no-preview
```

- **INVIO** = fissa lo zero · **q / ESC / Ctrl+C** = esci.
- La finestra mostra il tag rilevato e, in sovrimpressione, Hz gyro/camera, stato bias e
  orientamento/posizione relativi.

## Prova senza hardware (demo sintetica)

Camera e IMU vengono simulate con lo **stesso** moto noto, cosi' puoi vedere la fusione
funzionare senza Kinect ne' BLE:

```bash
cd test_preliminari/fusion_camera_gyro
python3 fusion_camera_gyro.py --demo --no-preview --duration 6 --auto-zero 1.5
```

Dovresti vedere `~150 Hz gyro / ~30 Hz cam` e un RPY/posizione relativi che seguono il moto.

E i test della sola matematica:

```bash
python3 test_fusion.py
```

## Note

- Verificato **in sintetico** su questa macchina (demo + `test_fusion.py`): la matematica e
  l'avvio sono a posto. La catena **hardware reale** (Kinect v1 + BLE) **non** e' testabile
  qui — va provata sul NUC.
- Se durante i movimenti la fusione "combatte" (scatti quando arriva il frame camera), prova
  ad abbassare `--slerp`; se invece l'IMU risulta ruotata rispetto al tag, imposta
  `R_GYRO_TO_TAG` in `fusion.py`.
- Il mapping orientamento/posizione -> mouse/schermo (come in `mouse_control`) e' un passo
  successivo: qui l'obiettivo e' solo avere l'orientamento fuso, fluido e senza deriva.
