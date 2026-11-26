# Mouse control con AprilTag (Kinect v1)

## Cosa fa

`mouse_control.py` trasforma un AprilTag **tag36h11** in un puntatore: la Kinect v1 inquadra il tag,
la posizione X/Y del tag (in metri, sistema camera) viene convertita in coordinate schermo con
un'**omografia** calcolata da 5–8 punti di calibrazione, e il cursore del mouse lo segue.
Avvicinando il tag alla camera si **tiene premuto il tasto sinistro**, allontanandolo si rilascia.
Un piccolo HUD in alto a destra mostra stato, angolo di rotazione del tag, distanza Z, stato del click e FPS.

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
4. **Calibra**: compare un overlay scuro a tutto schermo con un cerchio giallo.
   - Porta il tag nella posizione che vuoi far corrispondere a quel punto dello schermo, tenendolo **lontano** (Z > 0.50 m).
   - **Avvicinalo** alla camera (Z < 0.40 m) e tienilo lì per 0.15 s: l'anello verde si completa, il punto
     diventa verde e si passa al successivo.
   - Allontana di nuovo il tag (Z > 0.50 m) prima del punto successivo.
   - **Salta** salta un punto difficile da raggiungere (torna in coda più tardi).
5. **Conferma**: dopo 5 punti si abilita **Conferma**; con 8 punti la calibrazione si chiude da sola.
   Lo script passa in **ACTIVE** e il cursore segue il tag.

> Dopo la calibrazione (e dopo ogni ripresa con INVIO) il click si abilita solo quando il tag
> è stato allontanato oltre `--click-far` almeno una volta, così non parte un click involontario.

## Controllo

| Azione | Effetto |
|--------|---------|
| Tag vicino (Z < `--click-near`, default 0.40 m) | tiene premuto il tasto sinistro (trascinamento possibile) |
| Tag lontano (Z > `--click-far`, default 0.50 m) | rilascia il tasto sinistro |
| Tag non visto per più di 0.5 s | rilascia il tasto (sicurezza) |
| **Spazio** | FROZEN: rilascia il tasto, il mouse torna libero, puoi spostare il tag |
| **Invio** | riprende (ACTIVE) dalla posizione attuale del cursore, angolo riazzerato |
| **c** | ricomincia la calibrazione |
| **q** / **Esc** | esce (il tasto viene sempre rilasciato) |

Spazio + Invio funzionano come "sollevare il mouse dal tappetino": riprendendo, il cursore
continua dall'ultima posizione impostata dal tag, con l'offset ricalcolato rispetto alla nuova
posizione del tag. Su Wayland la posizione reale del cursore non è leggibile: se in FROZEN muovi
il mouse fisico, alla ripresa il cursore torna dove l'aveva lasciato il tag.

I tasti sono globali: funzionano anche se la finestra in primo piano è un'altra. **Non vengono
bloccati**: lo Spazio scrive anche uno spazio nell'app in primo piano, e `q`/`c` agiscono sullo
script anche mentre scrivi altrove. Funzionano inoltre su HUD e finestra camera.

## Angolo

L'HUD mostra `↺ XX.X°`: è la rotazione del tag **attorno al proprio asse Z** (come girare una
manopola), **relativa** all'orientamento che il tag aveva alla fine della calibrazione o
all'ultima ripresa con INVIO (`R_rel = R0ᵀ · R`). Non dipende dalla posizione del tag:
spostandolo nello spazio senza ruotarlo l'angolo resta ~0°.

## Problemi comuni

| Sintomo | Soluzione |
|---------|-----------|
| All'avvio `Mouse: pynput (X11)` su Wayland, cursore fermo | `/dev/uinput` non scrivibile: rifai "Permessi per Wayland" e **logout/login**; per vedere l'errore esatto: `python mouse_control.py --mouse uinput` |
| `[keys] evdev unavailable` | utente non nel gruppo `input` (logout/login dopo `usermod`); intanto i tasti funzionano cliccando sull'HUD |
| `pip install evdev` fallisce (`Python.h` / `gcc`) | `sudo apt install python3-dev build-essential` |
| Cursore mappato male con più monitor | il mouse virtuale copre l'intero desktop: usa un solo monitor per il test |
| Alternativa a uinput | sessione **"Ubuntu su Xorg"** al login, poi `python mouse_control.py --mouse pynput --keys pynput` |
| `freenect_sync_get_video failed` | la Kinect è occupata (dashboard `1_kinect_dashboard`, `freenect-glview`, `apriltag_pose.py`): chiudili; controlla alimentatore e USB |
| Click che parte da solo o non parte mai | regola le soglie guardando `Z:` nell'HUD, es. `--click-near 0.35 --click-far 0.45`; tieni almeno 5–10 cm di isteresi |
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
| `--click-near` | 0.40 | Z (m) sotto cui il tasto sinistro viene premuto |
| `--click-far` | 0.50 | Z (m) sopra cui il tasto viene rilasciato (deve essere > `--click-near`) |
| `--click-hold` | 0.15 | secondi sotto `--click-near` per registrare un punto di calibrazione |
| `--decimate` | 1.0 | `quad_decimate` del detector (2.0 = più veloce, portata minore) |
| `--threads` | 4 | thread del detector |
| `--no-display` | — | nessuna finestra camera, solo HUD |
| `--mouse` | auto | `uinput` (Wayland/X11, kernel), `pynput` (solo X11); auto = uinput su Wayland |
| `--keys` | auto | `evdev` (tastiera da `/dev/input`), `pynput`; auto = evdev se leggibile |
