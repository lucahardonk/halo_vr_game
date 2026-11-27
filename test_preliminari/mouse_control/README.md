# Mouse control con AprilTag (Kinect v1) — metodo T1/T2/T3

Script singolo e autonomo: `mouse_control.py`. La Kinect v1 vede un AprilTag **tag36h11**
(`cv2.aruco.DICT_APRILTAG_36h11`) e il cursore segue lo spostamento del tag.

## Flusso matematico

Tutte le trasformazioni sono matrici omogenee 4×4 nel frame camera (x destra, y giù, z avanti).
Il frame del tag è espresso con gli stessi assi (x destra, y giù, z dentro il tag), quindi un tag
rivolto verso la camera ha rotazione ≈ identità.

| Simbolo | Significato |
|---|---|
| `T1` | posa camera → tag, calcolata a ogni frame (`estimatePoseSingleMarkers` / `solvePnP IPPE_SQUARE`) |
| `T2` | posa di riferimento fissa: traslazione `(0, 0, 0.6)`, rotazione identità = centro schermo a 60 cm |
| `T1_ref` | `T1` catturata quando premi **INVIO** |
| `T3 = T2 · inv(T1_ref)` | trasformazione virtuale camera → monitor |

A ogni frame:

```
T_rel  = inv(T1_ref) · T1_corrente      # spostamento del tag rispetto alla referenza
T_virt = T3 · T1_corrente  (= T2 · T_rel)
u, v   = x/z, y/z  di T_virt            # proiezione normalizzata sulla distanza
px     = 1920 ± gain · u                # schermo 3840×2160 (4K 16:9, hardcoded)
py     = 1080 ± gain · v
```

`gain = 1920 / (reach / 0.6)`: con `--reach 0.20` (default) spostare il tag di 20 cm a 60 cm porta
il cursore dal centro al bordo. Il cursore è limitato a `[0, 3839] × [0, 2159]`.

Parametri Kinect v1 fissi nel codice: `fx = fy = 525`, `cx = 319.5`, `cy = 239.5`, 640×480,
distorsione nulla.

## Avvio

```bash
sudo apt install -y libfreenect-dev
pip install -r requirements.txt
python3 mouse_control.py --tag-size 0.05      # lato del quadrato NERO del tag, in metri
```

1. Lo script dice: *"Posiziona il tag al centro dello schermo a ~60 cm. Premi INVIO per calibrare."*
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

## Opzioni

| Flag | Default | Descrizione |
|---|---|---|
| `--tag-size` | `0.05` | lato del tag in metri |
| `--tag-id` | primo visto | usa solo un ID specifico |
| `--reach` | `0.20` | metri (a 60 cm) per andare dal centro al bordo dello schermo |
| `--no-mirror` | off | non specchia l'asse X (se il cursore va al contrario in orizzontale) |
| `--invert-y` | off | inverte l'asse Y |
| `--smooth` | `0.5` | filtro esponenziale (0 = disattivato) |
| `--no-preview` | off | nessuna finestra camera |
| `--index` | `0` | indice device Kinect |
| `--mouse` | `auto` | `uinput` (kernel, Wayland/X11), `pyautogui` (solo X11); auto = uinput se disponibile |

Nota: di default l'asse X è specchiato perché la Kinect guarda l'utente (destra in immagine =
sinistra dell'utente). Se il tag viene tenuto ruotato di 180° gli assi si invertono: usa
`--no-mirror` / `--invert-y`.
