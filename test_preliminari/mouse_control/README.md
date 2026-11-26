# Mouse control con AprilTag (Kinect v1)

## Cosa fa

`mouse_control.py` trasforma un AprilTag **tag36h11** in un puntatore tipo **laser**: la Kinect v1
vede il tag e lo script calcola dove il "raggio" del tag (la sua normale) incontra il piano
dello schermo. Il punto viene convertito in pixel con le **omografie** di calibrazione e il cursore lo segue.

- **Posizione**: spostando il tag il cursore si muove.
- **Angolo**: inclinando il tag il cursore si sposta tanto più quanto più sei lontano
  (spostamento ≈ distanza · tan(inclinazione)), come un puntatore laser.
- **Distanza**: avvicinando il tag alla camera si **tiene premuto il tasto sinistro**, allontanandolo si rilascia.

Si ipotizza che la Kinect stia **sul monitor** (piano dello schermo = piano della camera). Se sta
più avanti o più indietro, correggi con `--screen-offset` (metri, positivo = schermo dietro la Kinect).

Un piccolo HUD in alto a destra mostra stato, rotazione e inclinazione del tag, distanza Z, click e FPS.

## Installazione

### 1. Pacchetti e dipendenze Python

```bash
sudo apt install -y libfreenect-dev python3-tk python3-dev build-essential
source ~/.venvs/halo_vr_game/bin/activate
cd ~/Documents/halo_vr_game/test_preliminari/mouse_control
pip install -r requirements.txt
```

- `python3-tk`: HUD e finestra di calibrazione (tkinter).
- `python3-dev build-essential`: servono a compilare `evdev` (pip non ha un pacchetto precompilato).
- Il modulo Python `freenect` è opzionale: senza, lo script usa `libfreenect_sync` via ctypes.

### 2. Permessi per Wayland (una volta sola)

Su **Wayland** (default di Ubuntu) un programma non può muovere il cursore né leggere i tasti
delle altre finestre con le librerie X11 (pynput). Lo script quindi crea un **mouse virtuale
a livello kernel** (`/dev/uinput`) e legge la tastiera da `/dev/input`: funziona con
qualsiasi finestra. Serve il gruppo `input`:

```bash
echo 'KERNEL=="uinput", SUBSYSTEM=="misc", GROUP="input", MODE="0660", OPTIONS+="static_node=uinput"' \
  | sudo tee /etc/udev/rules.d/60-uinput.rules
echo uinput | sudo tee /etc/modules-load.d/uinput.conf   # carica il modulo a ogni avvio
sudo modprobe uinput
sudo usermod -aG input $USER
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**Poi logout e login** (o riavvio). Verifica:

```bash
groups | grep -o input        # deve stampare: input
ls -l /dev/uinput             # deve essere: crw-rw---- root input
```

> Sicurezza: il gruppo `input` permette ai tuoi programmi di leggere la tastiera. Su un PC
> personale va bene; per toglierlo: `sudo gpasswd -d $USER input`.

Su **X11** ("Ubuntu su Xorg") questo passo non serve: lo script usa pynput.

## Avvio e calibrazione

1. **Genera il tag** (se non ce l'hai già):
   ```bash
   cd ~/Documents/halo_vr_game/test_preliminari/kinect_base/2_apriltag_pose
   python apriltag_pose.py --generate 0
   ```
2. **Stampa e misura** il lato del quadrato nero (bordo nero incluso, margine bianco escluso).
   Default: 6.5 cm → `--tag-size 0.065`.
3. **Chiudi** la dashboard e altri programmi che usano la Kinect, poi **avvia**:
   ```bash
   cd ~/Documents/halo_vr_game/test_preliminari/mouse_control
   python mouse_control.py --tag-size 0.065
   ```
4. **Calibra** con il tag **dritto** (rivolto verso la camera, non inclinato). Compare un overlay
   scuro con un cerchio colorato, e **il cursore viene tenuto sul cerchio** (se non c'è, il backend
   mouse non funziona: guarda la riga `mouse:` nell'HUD e "Problemi comuni"). In basso a destra
   c'è l'anteprima della camera (specchiata), sullo schermo un **mirino** dove sta il tag
   (verde = visto, arancione = non visto).

   **Set 1/2 – LONTANO** (cerchio giallo): tieni il tag alla distanza a cui lo userai normalmente,
   spostalo nella posizione che vuoi far corrispondere al cerchio e premi **INVIO** (oppure il
   pulsante *Salva punto*). Il punto diventa verde e si passa al successivo. Fai 5–8 punti.

   **Set 2/2 – VICINO** (cerchio blu): stessa cosa, ma con il tag **avvicinato alla camera** come
   quando vuoi cliccare (almeno 4 cm più vicino del set lontano, altrimenti il punto viene rifiutato).

   - **Salta**: salta un punto scomodo (torna in coda).
   - **Conferma set**: chiude il set dopo almeno 5 punti; con 8 punti si chiude da solo.
   - INVIO senza tag visibile non salva niente (lo dice l'overlay).
5. Finiti i due set lo script calcola le **soglie del click** dalle due distanze (premuto al 40%
   del tratto dal VICINO al LONTANO, rilasciato al 70%) e passa in **ACTIVE**.

> Dopo la calibrazione (e dopo ogni ripresa) il click si abilita solo quando il tag
> è stato portato oltre la soglia di rilascio almeno una volta, così non parte un click involontario.

## Controllo

| Azione | Effetto |
|--------|---------|
| Tag inclinato oltre la zona morta (default 10°) | il cursore si sposta come un raggio laser |
| Tag avvicinato (soglia dalla calibrazione, scritta nell'HUD) | tiene premuto il tasto sinistro (trascinamento possibile) |
| Tag allontanato | rilascia il tasto sinistro |
| Tag non visto per più di 0.5 s | rilascia il tasto (sicurezza) |
| **Invio** (in calibrazione) | salva il punto evidenziato |
| **Spazio** | FERMO: il cursore resta fermo dov'è e il tasto viene rilasciato; riposiziona il tag |
| **Spazio** o **Invio** (da FERMO) | riprende dal punto in cui il cursore era fermo, rotazione riazzerata |
| **c** | ricomincia la calibrazione |
| **q** / **Esc** | esce (il tasto viene sempre rilasciato) |

Spazio + Invio funzionano come "sollevare il mouse dal tappetino": riprendendo, il cursore
continua dall'ultima posizione impostata dal tag, con l'offset ricalcolato rispetto alla nuova
posizione del tag. Su Wayland la posizione reale del cursore non è leggibile: se in FROZEN muovi
il mouse fisico, lo script lo riporta subito al punto fermo (il cursore è "suo" finché lo script gira).

I tasti sono globali: funzionano anche se la finestra in primo piano è un'altra. **Non vengono
bloccati**: lo Spazio scrive anche uno spazio nell'app in primo piano, e `q`/`c` agiscono sullo
script anche mentre scrivi altrove. Funzionano inoltre su HUD e finestra camera.

## Angolo

L'HUD mostra due valori:

- `↺ XX.X°`: rotazione del tag **attorno al proprio asse Z** (come girare una manopola), relativa
  all'orientamento a fine calibrazione o all'ultima ripresa. Non dipende dalla posizione.
- `incl XX°`: quanto il tag è inclinato rispetto alla direzione della camera; è questo che sposta il raggio.

**Zona morta**: con un tag piccolo la stima dell'orientamento è rumorosa. In simulazione, con un
tag perfettamente dritto da 6,5 cm a 0,7 m, l'errore arrivava a **8°**: senza zona morta il cursore
ballerebbe di un terzo dello schermo. Per questo i primi `--tilt-deadzone` gradi (default 10°) sono
ignorati e conta solo l'eccesso, senza scatti. Con un tag più grande (es. 15 cm stampato) il rumore
cala e puoi abbassare la zona morta.

## Problemi comuni

| Sintomo | Soluzione |
|---------|-----------|
| All'avvio `Mouse: pynput (X11)` su Wayland, cursore fermo | `/dev/uinput` non scrivibile: rifai "Permessi per Wayland" e **logout/login**; per vedere l'errore esatto: `python mouse_control.py --mouse uinput` |
| `[keys] evdev unavailable` | utente non nel gruppo `input` (logout/login dopo `usermod`); intanto i tasti funzionano cliccando sull'HUD |
| `pip install evdev` fallisce (`Python.h` / `gcc`) | `sudo apt install python3-dev build-essential` |
| Cursore mappato male con più monitor | il mouse virtuale copre l'intero desktop: usa un solo monitor per il test |
| Alternativa a uinput | sessione **"Ubuntu su Xorg"** al login, poi `python mouse_control.py --mouse pynput --keys pynput` |
| `freenect_sync_get_video failed` | la Kinect è occupata (dashboard `1_kinect_dashboard`, `freenect-glview`, `apriltag_pose.py`): chiudili; controlla alimentatore e USB |
| Click che parte da solo o non parte mai | ricalibra (`c`) con il set VICINO più vicino; oppure forza le soglie guardando `Z:` nell'HUD, es. `--click-near 0.35 --click-far 0.45` |
| Il cursore trema quando inclini | aumenta `--tilt-deadzone` (es. 15) o riduci `--tilt-gain` (es. 0.5); `--tilt-gain 0` = solo posizione |
| Inclinando il cursore va troppo / troppo poco | la Kinect non è sul piano dello schermo: regola `--screen-offset` (es. 0.2 se lo schermo è 20 cm dietro) o `--tilt-gain` |
| `Ignoring XDG_SESSION_TYPE=wayland on Gnome` | avviso di Qt (finestra camera OpenCV): lo script ora forza `QT_QPA_PLATFORM=xcb`, quindi non dovrebbe più comparire; è innocuo |
| Cursore che trema | allontana meno il tag, migliora la luce, stampa un tag più grande; il filtro One-Euro (`FILTER_MIN_CUTOFF` in cima al file) si può abbassare |
| Calibrazione imprecisa ai bordi | usa tutti gli 8 punti e copri un'area ampia con il tag; ricalibra con `c` |
| Overlay non trasparente | serve un compositor (GNOME lo ha); la calibrazione funziona lo stesso |
| `No module named tkinter` | `sudo apt install python3-tk` |
| Finestra camera assente, messaggio `OpenCV has no GUI support` | è installato `opencv-python-headless`: `pip uninstall opencv-python-headless && pip install opencv-contrib-python` |

## Opzioni CLI

| Opzione | Default | Significato |
|---------|---------|-------------|
| `--tag-size` | 0.065 | lato del quadrato nero del tag, in metri |
| `--index` | 0 | indice Kinect |
| `--opencv N` | — | usa la webcam OpenCV N al posto della Kinect |
| `--click-near` | dalla calibrazione | forza la Z (m) sotto cui il tasto sinistro viene premuto |
| `--click-far` | dalla calibrazione | forza la Z (m) sopra cui il tasto viene rilasciato |
| `--screen-offset` | 0.0 | Z del piano dello schermo nel sistema camera (m); 0 = Kinect sul monitor |
| `--tilt-gain` | 1.0 | quanto l'inclinazione sposta il cursore (0 = solo posizione, 1 = raggio vero) |
| `--tilt-deadzone` | 10 | gradi di inclinazione ignorati (rumore della posa) |
| `--decimate` | 1.0 | `quad_decimate` del detector (2.0 = più veloce, portata minore) |
| `--threads` | 4 | thread del detector |
| `--no-display` | — | nessuna finestra camera, solo HUD |
| `--mouse` | auto | `uinput` (Wayland/X11, kernel), `pynput` (solo X11); auto = uinput su Wayland |
| `--keys` | auto | `evdev` (tastiera da `/dev/input`), `pynput`; auto = evdev se leggibile |
