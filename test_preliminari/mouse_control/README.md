# Mouse control con AprilTag (Kinect v1) — metodo T1/T2/T3

Script singolo e autonomo: `mouse_control.py`. La Kinect v1 vede un AprilTag **tag36h11**
(`cv2.aruco.DICT_APRILTAG_36h11`) e il cursore segue lo spostamento del tag.

## Flusso matematico

Tutte le trasformazioni sono matrici omogenee 4×4 nel frame camera (x destra, y giù, z avanti).
Il frame del tag è espresso con gli stessi assi (x destra, y giù, z dentro il tag), quindi un tag
rivolto verso la camera ha rotazione ≈ identità.

| Simbolo | Significato |
|---|---|
| `T1` | posa camera → tag, calcolata a ogni frame (`solvePnP IPPE_SQUARE`) |
| `T2` | posa di riferimento fissa: traslazione `(0, 0, D)`, rotazione identità = tag davanti al centro dello schermo a distanza `D` (`--calib-dist`, default **0.5 m**) |
| `T1_ref` | `T1` catturata quando premi **INVIO** |
| `T3 = T2 · inv(T1_ref)` | trasformazione virtuale camera → monitor (schermo = piano `z = 0`) |

A ogni frame (proiezione a raggio, come un puntatore laser):

```
T_rel  = inv(T1_ref) · T1_corrente      # spostamento/rotazione del tag rispetto alla referenza
T_virt = T3 · T1_corrente  (= T2 · T_rel)
p      = traslazione di T_virt          # posizione del tag nel frame monitor
d      = R_virt · (0, 0, -1)            # normale del tag = direzione in cui punta
hit    = p + s·d,  s = -p_z / d_z       # intersezione con il piano dello schermo (metri)
px     = 1920 ± hit_x · 1920/reach      # W×H del monitor scelto con --monitor (es. 3840×2160)
py     = 1080 ± hit_y · 1920/reach
```

- Al momento di INVIO `T_rel = I` → `hit = (0, 0)` → il cursore parte dal **centro** (1920, 1080).
- Inclinando il tag di θ dal centro il punto si sposta di `D · tan θ`: ruotando a destra/sinistra
  il cursore percorre una **linea retta** orizzontale (es. D = 0.5 m, 45° → 0.5 m).
- Traslando il tag senza ruotarlo il punto si sposta 1:1 sul piano dello schermo.
- `--reach` = metri sul piano dello schermo dal centro al bordo orizzontale. Per un puntatore
  "laser" esatto mettilo uguale a **metà della larghezza fisica** del monitor (27" 16:9 ≈ 0.30).
  Valori più piccoli = più sensibile. Il cursore è limitato a `[0, 3839] × [0, 2159]`.

**Precisione:** la rotazione stimata da un tag piccolo nell'immagine è rumorosa. Con 65 mm a
~80 cm il tag è ~43 px su 640×480 e (test sintetico) l'errore di rotazione è ~5° medio, fino a
~12°; a D = 0.5 m 12° ≈ 10 cm sullo schermo. Per migliorare: tag più grande, tag più vicino alla
Kinect, buona luce, `--smooth` più alto.

Parametri Kinect v1 fissi nel codice: `fx = fy = 525`, `cx = 319.5`, `cy = 239.5`, 640×480,
distorsione nulla.

## Avvio

```bash
sudo apt install -y libfreenect-dev
~/Documents/halo_vr_game/test_preliminari/setup_venv.sh      # venv unico, una volta sola
source ~/Documents/halo_vr_game/test_preliminari/.venv/bin/activate
cd ~/Documents/halo_vr_game/test_preliminari/mouse_control
python3 mouse_control.py --tag-size 0.05      # lato del quadrato NERO del tag, in metri
```

1. Lo script dice: *"Posiziona il tag al centro dello schermo a ~50 cm, rivolto verso lo schermo. Premi INVIO per calibrare."*
2. **INVIO** → cattura `T1_ref`, calcola `T3` e stampa
   *"Calibrazione OK — controllo mouse attivo. [SPAZIO] per pausa, Ctrl+C per uscire."*
3. Muovi il tag: il cursore segue.
4. **SPAZIO** → pausa/ripresa del controllo mouse (il loop continua, cursore libero).
5. **INVIO** di nuovo → ricalibra.  **Ctrl+C** (o `q`/ESC nella finestra preview) → esce.

Tasti: INVIO dal terminale; INVIO/SPAZIO/q nella finestra preview; INVIO/SPAZIO globali da
qualsiasi finestra tramite `evdev` (`/dev/input`, funziona anche su Wayland), con fallback `pynput`
(solo X11). Se la Kinect non è collegata lo script stampa un errore ed esce.

## Mouse su Wayland (uinput)

Su Wayland `pyautogui`/`pynput` non possono muovere il cursore. Lo script crea quindi un
**mouse virtuale a livello kernel** (`/dev/uinput`, puntatore assoluto mappato su 3840×2160),
come la versione precedente. Se l'avevi già configurato non serve rifare nulla. Altrimenti:

```bash
echo 'KERNEL=="uinput", SUBSYSTEM=="misc", GROUP="input", MODE="0660", OPTIONS+="static_node=uinput"' \
  | sudo tee /etc/udev/rules.d/60-uinput.rules
echo uinput | sudo tee /etc/modules-load.d/uinput.conf
sudo modprobe uinput
sudo usermod -aG input $USER
sudo udevadm control --reload-rules && sudo udevadm trigger
# poi logout/login. Verifica: groups | grep input ; ls -l /dev/uinput  (root input)
```

All'avvio lo script stampa `Mouse: uinput ...`. Se stampa `pyautogui`, uinput non è utilizzabile:
`python mouse_control.py --mouse uinput ...` mostra il motivo esatto.

## Più monitor

Con più monitor il compositor distribuisce il puntatore assoluto uinput su **tutto il desktop**
(es. due 4K affiancati = 7680×2160), quindi il "centro" cadeva tra i due schermi. Ora lo script
legge la disposizione da `xrandr --listmonitors` (funziona anche su Wayland via XWayland),
limita il cursore al solo monitor scelto (default: il **primario**) e alla calibrazione lo porta
al centro di quel monitor. All'avvio stampa la geometria rilevata, ad esempio:

```
Monitor:
  xrandr 0: DP-2 (primario) 3840x2160+0+0
  xrandr 1: DP-1 3840x2160+3840+0
  -> cursore limitato a 3840x2160+0+0 dentro desktop 7680x2160+0+0
```

Se xrandr non è disponibile: `--monitor 0,0,3840,2160 --desktop 7680x2160`.

## Opzioni

| Flag | Default | Descrizione |
|---|---|---|
| `--tag-size` | `0.05` | lato del tag in metri |
| `--tag-id` | primo visto | usa solo un ID specifico |
| `--calib-dist` | `0.5` | distanza tag–schermo (m) quando premi INVIO (`T2 = (0, 0, D)`) |
| `--reach` | `0.30` | metri sul piano schermo dal centro al bordo orizzontale (= mezza larghezza fisica del monitor) |
| `--no-mirror` | off | non specchia l'asse X (se il cursore va al contrario in orizzontale) |
| `--invert-y` | off | inverte l'asse Y |
| `--smooth` | `0.5` | filtro esponenziale (0 = disattivato) |
| `--no-preview` | off | nessuna finestra camera |
| `--index` | `0` | indice device Kinect |
| `--monitor` | `primary` | monitor dove resta il cursore: `primary`, indice/nome di `xrandr --listmonitors`, oppure `X,Y,W,H` |
| `--desktop` | `auto` | dimensione totale del desktop `WxH` (tutti i monitor); auto = da xrandr |
| `--mouse` | `auto` | `uinput` (kernel, Wayland/X11), `pyautogui` (solo X11); auto = uinput se disponibile |

Nota: di default l'asse X è specchiato perché la Kinect guarda l'utente (destra in immagine =
sinistra dell'utente). Se il tag viene tenuto ruotato di 180° gli assi si invertono: usa
`--no-mirror` / `--invert-y`.
