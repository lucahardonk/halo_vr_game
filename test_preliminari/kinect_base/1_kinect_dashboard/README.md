# 1 — Kinect v1 Dashboard (Flask + Socket.IO)

Webapp locale che testa **tutte** le funzionalità della Kinect v1 da un'unica pagina:

| Funzione | Come | Frequenza |
|----------|------|-----------|
| Stream RGB | MJPEG `/video_rgb` (640×480) | ~30 fps (hardware) |
| Stream IR | MJPEG `/video_ir` (640×480, 8 bit) | ~30 fps |
| Profondità | MJPEG `/video_depth`, colormap JET + distanza al centro in metri | ~30 fps |
| Motore tilt | slider / pulsanti → `POST /api/tilt` | ±27° |
| LED | 6 stati → `POST /api/led` | — |
| Accelerometro | Socket.IO `accel_data`, grafico Chart.js ultimi 5 s | campionato a 100 Hz, inviato in pacchetti a 20 Hz |
| Array microfoni | Socket.IO `mic_direction`, GCC-PHAT, freccia SVG | ~31 stime/s (blocchi da 512 campioni @ 16 kHz) |
| Barra di stato | connessione, fps di ogni stream, tilt, accel, angolo mic | 1 Hz |

> **Limite hardware:** la Kinect v1 ha **un solo** canale video, quindi RGB e IR **non possono
> essere trasmessi insieme**. Nella card "Camera video" il selettore RGB / IR cambia modalità
> (serve ~0.5 s). La profondità gira sempre in parallelo.

## Struttura

```
1_kinect_dashboard/
├── app.py             # server Flask + Socket.IO, API REST, avvio thread
├── kinect_streams.py  # libfreenect_sync (ctypes): video, depth, tilt, LED, accelerometro
├── kinect_audio.py    # cattura 4 canali (pyaudio) + direzione GCC-PHAT
├── templates/index.html
├── static/app.js
└── requirements.txt
```

Video/depth/tilt/LED/accelerometro usano tutti la **sync API** di libfreenect via ctypes
(come il vecchio `kinect.py`): un solo handle del dispositivo condiviso, quindi nessun conflitto
USB tra camera e motore. Non serve il modulo Python `freenect`.

## Installazione (NUC, Ubuntu)

```bash
sudo apt install freenect libfreenect-dev portaudio19-dev kinect-audio-setup
source ~/.venvs/halo_vr_game/bin/activate
pip install -r ~/Documents/halo_vr_game/test_preliminari/kinect_base/1_kinect_dashboard/requirements.txt
```

- `kinect-audio-setup` scarica e carica il firmware audio: **dopo l'installazione scollega e
  ricollega la Kinect**. Verifica con `arecord -l` → deve comparire un dispositivo "Kinect".
- Se nel venv hai già `opencv-python`, non installare anche `opencv-contrib-python` (usa uno solo).

## Avvio

```bash
source ~/.venvs/halo_vr_game/bin/activate
cd ~/Documents/halo_vr_game/test_preliminari/kinect_base/1_kinect_dashboard
python app.py
```

Apri **http://localhost:5000** nel browser del NUC (oppure `http://<ip-del-nuc>:5000` da un altro PC
della stessa rete). Chiudi con **Ctrl+C**.

Opzioni:

| Opzione | Default | Significato |
|---------|---------|-------------|
| `--port` | 5000 | porta HTTP |
| `--host` | 0.0.0.0 | interfaccia |
| `--index` | 0 | indice Kinect |
| `--accel-hz` | 100 | frequenza di lettura accelerometro |
| `--no-audio` | — | disattiva i microfoni |
| `--audio-device` | auto | indice PortAudio o parte del nome (se l'auto-detect "Kinect" non funziona) |
| `--noise-gate` | -55 | soglia dBFS sotto la quale la direzione non viene aggiornata |

## Cosa verificare

1. Barra in alto: **Socket: connesso** e **Kinect: connessa**, RGB ~30 fps, Depth ~30 fps, Accel ~100 Hz.
2. **RGB/IR**: passa a IR → immagine grigia con i puntini del proiettore.
3. **Profondità**: muovi la mano davanti al sensore → colori da blu (lontano) a rosso (vicino);
   a sinistra la distanza in metri del punto al centro (mirino bianco).
4. **Tilt**: muovi lo slider → la Kinect si inclina; "Attuale" segue il target.
5. **LED**: ogni pulsante cambia il LED frontale.
6. **Accelerometro**: inclina la Kinect / il tavolo → le curve X/Y/Z cambiano; a riposo |a| ≈ 9.8.
7. **Microfoni**: batti le mani o parla a sinistra e a destra del sensore → la freccia ruota
   (±50°) e si allunga quando la confidenza è alta. Se va dal lato opposto attiva **inverti**.

## API

| Metodo | Endpoint | Body JSON | Risposta |
|--------|----------|-----------|----------|
| GET | `/api/status` | — | stato completo |
| GET | `/api/tilt` | — | `{tilt, target, status}` |
| POST | `/api/tilt` | `{"angle": 10}` | `{ok, target}` |
| POST | `/api/led` | `{"state": "red"}` (off, green, red, yellow, blink_green, blink_red_yellow) | `{ok, state}` |
| POST | `/api/video_mode` | `{"mode": "ir"}` (rgb / ir) | `{ok, mode}` |

Eventi Socket.IO (namespace `/data`): `accel_data`, `mic_direction`, `status`.

## Direzione del suono (GCC-PHAT)

- Posizioni microfoni sulla barra (m): `-0.113, 0.036, 0.076, 0.113` (mic0 … mic3).
- Coppie usate: 0–3 (22.6 cm, la più esterna), 0–2 e 1–3; stime pesate per baseline e nitidezza del picco.
- Banda 300–5000 Hz, interpolazione ×16 della correlazione (risoluzione < 1°).
- `confidence` (0–1) = nitidezza del picco × energia sopra la soglia di rumore.
- Angolo positivo = suono dal lato di mic3. L'ordine dei canali ALSA può variare: usa **inverti**.

## Problemi comuni

| Sintomo | Soluzione |
|---------|-----------|
| `libfreenect_sync not found` | `sudo apt install libfreenect-dev` |
| Kinect: disconnessa | alimentatore + USB; chiudi `freenect-glview` e altri script Kinect; `lsusb -d 045e:` |
| `LIBUSB_ERROR_ACCESS` | regole udev: reinstalla `libfreenect-dev`, scollega/ricollega |
| Tilt/LED danno errore 503 | il sottodispositivo motore non risponde: ricollega la Kinect (modelli "Kinect for Windows" 1473 possono richiedere firmware audio caricato) |
| Microfoni non disponibili | `sudo apt install kinect-audio-setup`, ricollega, `arecord -l`; poi `--audio-device Kinect` |
| `pyaudio not installed` | `sudo apt install portaudio19-dev && pip install pyaudio` |
| La pagina non mostra grafici | servono i CDN (Bootstrap, Chart.js, Socket.IO): il NUC deve essere online |
