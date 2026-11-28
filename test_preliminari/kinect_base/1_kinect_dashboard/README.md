# 1 — Kinect v1 Dashboard (Flask + Socket.IO)

Webapp locale che testa **tutte** le funzionalità della Kinect v1 da un'unica pagina:

| Funzione | Come | Frequenza |
|----------|------|-----------|
| Stream RGB | MJPEG `/video_rgb` (640×480) | ~30 fps |
| Stream IR | MJPEG `/video_ir` (640×480, 8 bit) | ~30 fps |
| Profondità | MJPEG `/video_depth`, colormap JET + distanza al centro in m | ~30 fps |
| Motore tilt | slider / pulsanti → `POST /api/tilt` | ±27° |
| LED | 6 stati → `POST /api/led` | — |
| Accelerometro | Socket.IO `accel_data`, grafico Chart.js ultimi 5 s | 100 Hz, inviato a 20 Hz |
| Array microfoni | Socket.IO `mic_direction`, GCC-PHAT, freccia SVG | ~31 stime/s a 16 kHz |
| Barra di stato | connessione, fps, tilt, accel, angolo mic, drop USB | 1 Hz |

> **Limite hardware:** la Kinect v1 ha un solo canale video, quindi RGB e IR **non possono
> essere trasmessi insieme**. Il selettore RGB / IR nella dashboard cambia modalità in ~0.5 s.
> La profondità gira sempre in parallelo.

---

## Struttura

```
1_kinect_dashboard/
├── app.py             # server Flask + Socket.IO, API REST, avvio thread
├── kinect_streams.py  # libfreenect_sync via ctypes: video, depth, tilt, LED, accel
├── kinect_audio.py    # cattura 4 canali (pyaudio) + direzione GCC-PHAT
├── noise_filter.py    # filtra spam ALSA/JACK/libfreenect su stderr
├── templates/index.html
├── static/app.js
└── requirements.txt
```

---

## 1 — Setup sistema (una volta sola)

Eseguire come utente normale sul NUC (Ubuntu 24+).

### 1.1 Pacchetti di sistema

```bash
sudo apt update
sudo apt install -y \
    libfreenect-dev freenect \
    portaudio19-dev \
    p7zip-full
```

### 1.2 Firmware audio dei microfoni (kinect-audio-setup)

Il pacchetto `kinect-audio-setup` scarica il firmware USB dal sito Microsoft e verifica il suo
hash MD5. L'hash nel pacchetto non è aggiornato: bisogna calcolarlo dal file reale.

```bash
# 1. installa il pacchetto (può fallire nella fase postinst, va bene)
sudo apt install -y kinect-audio-setup || true

# 2. scarica l'MSI originale per calcolare l'hash attuale
URL=$(grep -o 'http[^" ]*\.msi' /usr/sbin/kinect_fetch_fw | head -1)
wget -q -O /tmp/kinect_sdk.msi "$URL"
file /tmp/kinect_sdk.msi   # deve dire "Composite Document File", non una pagina HTML

# 3. aggiorna l'hash nello script
MD5=$(md5sum /tmp/kinect_sdk.msi | cut -d' ' -f1)
echo "MD5 rilevato: $MD5"
sudo sed -i "s/^SDK_MD5=.*/SDK_MD5=\"$MD5\"/" /usr/sbin/kinect_fetch_fw

# 4. completa l'installazione interrotta
sudo dpkg --configure -a

# 5. verifica che il firmware sia stato estratto
ls /lib/firmware/kinect/
# deve comparire: UACFirmware (e altri file)

rm /tmp/kinect_sdk.msi
```

> **Se `file` riporta "HTML document":** Microsoft ha cambiato il link. Apri il link stampato
> da `echo $URL` nel browser, scarica l'MSI manualmente, copialo in `/tmp/kinect_sdk.msi`
> e riprendi dal punto 3.

### 1.3 Regole udev (accesso Kinect senza sudo)

```bash
sudo tee /etc/udev/rules.d/51-kinect.rules > /dev/null << 'EOF'
# Kinect v1 — camera + motore (045e:02ae) e audio (045e:02ad / 045e:02b0)
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02b0", MODE="0666", GROUP="plugdev"
EOF

sudo adduser $USER plugdev
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**Dopo questo passo: fai logout e login** (o riavvia il NUC) per essere nel gruppo `plugdev`.

### 1.4 Prima connessione della Kinect

Collega la Kinect **direttamente a una porta USB del PC**, senza hub né prolunghe.
Dopo il login (passo 1.3), scollega e ricollega fisicamente la Kinect dalla corrente e dall'USB
per fare caricare il firmware audio (si carica automaticamente via udev).

Verifica che i microfoni siano visibili:

```bash
arecord -l
# deve comparire qualcosa tipo:
# card N: Kinect [Xbox NUI Audio], device 0: USB Audio ...
```

---

## 2 — Setup Python (venv)

```bash
# crea/aggiorna il venv unico dei test preliminari (tutte le dipendenze)
~/Documents/halo_vr_game/test_preliminari/setup_venv.sh

# attiva
source ~/Documents/halo_vr_game/test_preliminari/.venv/bin/activate

# installa le dipendenze della dashboard
pip install -r ~/Documents/halo_vr_game/test_preliminari/kinect_base/1_kinect_dashboard/requirements.txt
```

> Se nel venv hai già `opencv-python` non installare anche `opencv-contrib-python`: usane uno solo.

---

## 3 — Avvio

```bash
source ~/Documents/halo_vr_game/test_preliminari/.venv/bin/activate
cd ~/Documents/halo_vr_game/test_preliminari/kinect_base/1_kinect_dashboard
python app.py
```

Apri **http://localhost:5000** nel browser. Chiudi con **Ctrl+C**.

Accessibile anche dalla rete locale: `http://<ip-del-nuc>:5000`

### Opzioni

| Opzione | Default | Significato |
|---------|---------|-------------|
| `--port` | 5000 | porta HTTP |
| `--host` | 0.0.0.0 | interfaccia di ascolto |
| `--index` | 0 | indice Kinect (0 = prima) |
| `--accel-hz` | 100 | frequenza di polling accelerometro |
| `--no-audio` | — | disattiva i microfoni |
| `--audio-device` | auto | indice PortAudio o stringa nel nome del dispositivo |
| `--noise-gate` | -55 | soglia dBFS sotto la quale la direzione non viene aggiornata |
| `--video-mode` | rgb | stream video iniziale: `rgb` oppure `ir` |
| `--list-audio` | — | stampa i dispositivi audio disponibili ed esce |
| `--verbose` | — | mostra i messaggi grezzi di libfreenect / ALSA / JACK su stderr |

---

## 4 — Cosa verificare

1. Barra in alto: **Socket: connesso** e **Kinect: connessa**; RGB ~30 fps, Depth ~30 fps, Accel ~100 Hz.
2. **RGB/IR**: clicca "IR" → immagine grigia con i puntini del proiettore. Reclicca "RGB".
3. **Profondità**: muovi la mano davanti al sensore → colori da blu (lontano) a rosso (vicino);
   nella card sotto la distanza in metri del punto centrale.
4. **Tilt**: muovi lo slider → la Kinect si inclina fisicamente; "Attuale" segue il target.
5. **LED**: ogni pulsante cambia il LED frontale della Kinect.
6. **Accelerometro**: inclina la Kinect → le curve X/Y/Z cambiano; a riposo |a| ≈ 9.8 m/s².
7. **Microfoni**: batti le mani o parla a sinistra/destra → la freccia ruota (±50°) e si
   allunga quando la confidenza è alta. Se va dal lato opposto usa il toggle **inverti**.

> Testato su NUC13ANHi7 con Kinect v1 (modello 1414) collegata direttamente alla porta USB3 del PC.

---

## 5 — Problemi comuni

| Sintomo | Soluzione |
|---------|-----------|
| `libfreenect_sync not found` | `sudo apt install libfreenect-dev` |
| `Kinect: disconnessa` nella dashboard | alimentatore Kinect collegato + USB direttamente al PC; chiudi `freenect-glview`; `lsusb \| grep 045e` |
| `LIBUSB_ERROR_ACCESS` | fai di nuovo i passi 1.3 e logout/login; poi scollega/ricollega la Kinect |
| Tilt/LED danno errore 503 | sottodispositivo motore non risponde: ricollega la Kinect; sui modelli 1473 (Kinect for Windows) il motore richiede il firmware audio caricato prima |
| `kinect-audio-setup` fallisce con errore MD5 | ripeti il passo 1.2: l'hash nel pacchetto Debian è vecchio, bisogna calcolarlo dal file reale |
| Microfoni non trovati dopo `kinect-audio-setup` | scollega la Kinect dalla corrente e dall'USB, ricollegala (udev carica il firmware al collegamento); poi `arecord -l` |
| Microfoni non trovati: scegliere il dispositivo | `python app.py --list-audio` → prendi il numero accanto a "Kinect" → `python app.py --audio-device <N>` |
| `pyaudio not installed` | `sudo apt install portaudio19-dev && pip install pyaudio` |
| Badge **USB drop N/s** > 0 / immagini a strisce (`[Stream 80] Expected 1908`) | pacchetti video persi: Kinect diretta al PC senza hub né prolunghe; oppure hub USB 2.0 alimentato |
| **RGB fermo** / "no frames from Kinect" (depth e accel OK) | stream video bloccato in libfreenect (tipico su alcuni controller USB3): prova un'altra porta USB; oppure avvia con `--video-mode ir` per verificare l'IR; `freenect-glview` per isolare il problema; per cambiare modalità dopo lo stallo bisogna riavviare l'app |
| La pagina non mostra grafici o microfono | CDN (Bootstrap, Chart.js, Socket.IO): il NUC deve essere online |
| Dashboard lenta | abbassa la qualità JPEG in `kinect_streams.py` (default 85 → 70) |

---

## 6 — API REST

| Metodo | Endpoint | Body JSON | Risposta |
|--------|----------|-----------|----------|
| GET | `/api/status` | — | stato completo (connessione, fps, tilt, led, accel, mic, usb) |
| GET | `/api/tilt` | — | `{tilt, target, status}` |
| POST | `/api/tilt` | `{"angle": 10}` | `{ok, target}` |
| POST | `/api/led` | `{"state": "red"}` | `{ok, state}` — valori: off, green, red, yellow, blink_green, blink_red_yellow |
| POST | `/api/video_mode` | `{"mode": "ir"}` | `{ok, mode}` — valori: rgb, ir |

### Esempio

```bash
curl -X POST http://localhost:5000/api/tilt -H 'Content-Type: application/json' -d '{"angle": 10}'
curl -X POST http://localhost:5000/api/led  -H 'Content-Type: application/json' -d '{"state": "blink_green"}'
```

**Socket.IO** (namespace `/data`): `accel_data`, `mic_direction`, `status`.

---

## 7 — Direzione del suono (GCC-PHAT)

- Posizioni microfoni sulla barra (m): `-0.113, 0.036, 0.076, 0.113` (mic0 … mic3).
- Coppie usate: 0–3 (22.6 cm, baseline più lunga), 0–2 e 1–3; stime pesate per baseline e nitidezza del picco.
- Banda 300–5000 Hz, interpolazione ×16 della correlazione (risoluzione sub-grado).
- `confidence` (0–1) = nitidezza del picco × energia sopra la soglia di rumore.
- Angolo positivo = suono dal lato di mic3. L'ordine dei canali ALSA può variare: usa **inverti** in dashboard.
