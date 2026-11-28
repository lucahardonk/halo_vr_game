# bluetooth_test — 7 pulsanti + IMU 9 assi via BLE (Nano 33 BLE Sense Rev2)

Firmware di test che legge **7 pulsanti** e **IMU a 9 assi** e invia tutto via **BLE notify**
(un'unica characteristic, payload binario da 37 byte). Lo script Python `test_receiver.py`
riceve, decodifica e stampa i dati e la frequenza di ricezione (Hz).

PC di riferimento: **Intel NUC 13 con Ubuntu 26 (BlueZ)**.

---

## 1. Collegamenti

> ⚠️ Il Nano 33 BLE Sense lavora a **3,3 V e NON tollera i 5 V** sui pin.
> I pulsanti vanno collegati **solo fra il pin e GND**: non collegare mai 5 V o 3,3 V ai pin D2–D8.

Si usa il pull-up interno (`INPUT_PULLUP`): **non servono resistenze**.
A riposo il pin legge HIGH, quando premi il pulsante legge LOW (= premuto).

| Pin Arduino | Funzione  | Filo da collegare                               | Bit nel payload |
|-------------|-----------|-------------------------------------------------|-----------------|
| D2          | Pulsante 1 | un capo del pulsante 1 (l'altro capo a GND)    | bit 0           |
| D3          | Pulsante 2 | un capo del pulsante 2 (l'altro capo a GND)    | bit 1           |
| D4          | Pulsante 3 | un capo del pulsante 3 (l'altro capo a GND)    | bit 2           |
| D5          | Pulsante 4 | un capo del pulsante 4 (l'altro capo a GND)    | bit 3           |
| D6          | Pulsante 5 | un capo del pulsante 5 (l'altro capo a GND)    | bit 4           |
| D7          | Pulsante 6 | un capo del pulsante 6 (l'altro capo a GND)    | bit 5           |
| D8          | Pulsante 7 | un capo del pulsante 7 (l'altro capo a GND)    | bit 6           |
| GND         | Massa comune | secondo capo di **tutti** i pulsanti          | —               |
| USB         | Alimentazione + upload | cavo micro-USB verso il PC           | —               |

L'IMU è già **integrata sulla scheda** e non richiede collegamenti.

### Schema

```
                 Arduino Nano 33 BLE Sense Rev2
                ┌──────────────────────────────┐
  [BTN1] ───────┤ D2                           │
  [BTN2] ───────┤ D3                           │
  [BTN3] ───────┤ D4      (pull-up interni     │
  [BTN4] ───────┤ D5       attivi su D2..D8)   │
  [BTN5] ───────┤ D6                           │
  [BTN6] ───────┤ D7                           │
  [BTN7] ───────┤ D8                           │
     │          │                              │
     └──────────┤ GND                          │
  (secondo capo │                         USB ─┼──── PC
   di TUTTI i   └──────────────────────────────┘
   pulsanti → GND)

  Ogni pulsante:   Dx ──o/ o── GND
```

---

## 2. Payload BLE (37 byte, little-endian)

| Offset | Tipo        | Campo             | Unità       |
|--------|-------------|-------------------|-------------|
| 0      | uint8       | pulsanti (bit0 = D2 … bit6 = D8, 1 = premuto) | — |
| 1–12   | 3 × float32 | ax, ay, az        | m/s²        |
| 13–24  | 3 × float32 | gx, gy, gz        | °/s         |
| 25–36  | 3 × float32 | mx, my, mz        | µT          |

Python: `struct.unpack("<B9f", data)`.
UUID: servizio `19b10000-e8f2-537e-4f6c-d104768a1214`, characteristic `19b10001-e8f2-537e-4f6c-d104768a1214`.
Nome advertising: `IMU_BTN`. Nello script la stringa `BTN` mostra D2 a sinistra e D8 a destra.

---

## 3. Librerie Arduino

Da Arduino IDE → *Strumenti → Gestione librerie*:

| Libreria | Quando serve |
|----------|--------------|
| **ArduinoBLE** | sempre |
| **Arduino_BMI270_BMM150** | **Rev2** (default dello sketch): la Rev2 monta BMI270 + BMM150 |
| Arduino_LSM9DS1 | solo per la **Rev1**: impostare `#define USE_REV1_LSM9DS1 1` |

> La Rev2 **non** ha l'LSM9DS1 (montato solo sulla Rev1). Con la libreria LSM9DS1, sulla Rev2
> `IMU.begin()` fallisce e il LED lampeggia velocemente.

Board: *Gestione schede* → **Arduino Mbed OS Nano Boards** → scheda "Arduino Nano 33 BLE".

Da riga di comando (alternativa):
```bash
arduino-cli core install arduino:mbed_nano
arduino-cli lib install ArduinoBLE Arduino_BMI270_BMM150
arduino-cli compile --fqbn arduino:mbed_nano:nano33ble bluetooth_test
arduino-cli upload  --fqbn arduino:mbed_nano:nano33ble -p /dev/ttyACM0 bluetooth_test
```

## 4. Caricare il firmware

1. Collega la scheda via USB e apri `bluetooth_test/bluetooth_test.ino`.
2. Scegli la scheda **Arduino Nano 33 BLE** e la porta (`/dev/ttyACM0`).
3. Clicca **Carica**. Se la porta non compare, fai doppio click rapido sul tasto reset
   (modalità bootloader, il LED "pulsa") e riprova.
4. Su Linux, se l'upload fallisce per i permessi: `sudo usermod -aG dialout $USER` e poi rifai il login.
5. (Opzionale) Il Monitor seriale a 115200 mostra l'ODR dei sensori e lo stato della connessione.

LED integrato: **acceso = connesso**, spento = in advertising, lampeggio veloce = errore IMU/BLE.

---

## 5. Script Python (Ubuntu / NUC)

```bash
sudo apt install -y bluez python3-venv     # di solito già presenti
sudo systemctl enable --now bluetooth

# venv unico dei test preliminari (una volta sola, installa anche dbus-fast)
~/Documents/halo_vr_game/test_preliminari/setup_venv.sh

source ~/Documents/halo_vr_game/test_preliminari/.venv/bin/activate
cd ~/Documents/halo_vr_game/test_preliminari/bluetooth_test
python test_receiver.py                 # cerca IMU_BTN per nome
python test_receiver.py --address 18:FE:03:D9:D4:4B   # oppure per indirizzo
```

**La connessione la fai dalla GUI di Ubuntu** (Impostazioni → Bluetooth → `IMU_BTN` → *Connection* ON).
Lo script **non scansiona e non si connette**: parla con BlueZ via D-Bus, trova `IMU_BTN` già
connesso, attiva le notifiche e stampa. Se la scheda non è connessa, lo script aspetta e te lo dice.
Alla chiusura ferma solo le notifiche: la connessione della GUI resta aperta.

Output di esempio:
```
[Hz: 123.4] BTN: 0001010 | Accel: ax=+0.123 ay=-0.456 az=+9.812 | Gyro: gx=+0.01 gy=-0.02 gz=+0.00 | Mag: mx=+12.3 my=-5.6 mz=+8.9
```
Con Ctrl+C lo script si chiude in modo pulito e stampa i frame totali e la frequenza media.
Se la scheda si disconnette, lo script resta in attesa finché non la riconnetti dalla GUI.

Controllo a mano: `bluetoothctl info 18:FE:03:D9:D4:4B` → deve dire `Connected: yes`.

---

## 6. Frequenza (>100 Hz) e limiti BLE

- **Il connection interval lo decide il PC (central).** Il firmware *chiede* 7,5 ms
  (`BLE.setConnectionInterval(6, 6)`), ma BlueZ di default usa 30–50 ms. Con un solo
  notify per connection event si resta così a ~20–33 Hz. Per forzare 7,5 ms su Ubuntu
  (richiede root e valori in unità da 1,25 ms; i valori si azzerano al riavvio):
  ```bash
  sudo mount -t debugfs none /sys/kernel/debug 2>/dev/null
  echo 6 | sudo tee /sys/kernel/debug/bluetooth/hci0/conn_min_interval
  echo 6 | sudo tee /sys/kernel/debug/bluetooth/hci0/conn_max_interval
  ```
  Esegui i comandi **prima di connettere la scheda dalla GUI** (il valore si applica alle
  nuove connessioni: se è già connessa, disattiva e riattiva *Connection*) (se `conn_max_interval` rifiuta il valore,
  scrivi prima `conn_min_interval`, come sopra). Con 7,5 ms il massimo teorico è ~133 Hz
  con 1 pacchetto per evento, di più se controller e stack inviano più pacchetti per evento.
- **MTU:** 37 byte di payload richiedono MTU ≥ 40. BlueZ negozia l'MTU automaticamente.
  ArduinoBLE accetta fino a ~240. Se lo script segnala frame da 20 byte, l'MTU è rimasto
  a 23: aggiorna BlueZ/ArduinoBLE oppure riduci il payload (es. int16 al posto di float).
- **ODR dei sensori:** con la libreria Arduino_BMI270_BMM150, accelerometro e giroscopio
  vanno a **100 Hz** e il magnetometro a **~10 Hz** (i valori esatti compaiono nel Monitor seriale).
  Il firmware invia frame alla massima velocità del link e include sempre *l'ultimo campione
  disponibile*: oltre i 100 Hz alcuni frame ripetono gli stessi valori IMU, mentre i
  pulsanti sono letti a ogni frame. Per limitare la frequenza di invio imposta
  `MIN_SEND_PERIOD_US` (es. `5000` = max 200 Hz).
- La stampa su terminale di ogni frame costa CPU: nel backend reale conviene elaborare i dati
  senza fare `print` a ogni frame.
- Distanza e interferenze Wi-Fi a 2,4 GHz riducono il throughput. Tieni la scheda vicina al NUC
  durante i test.

> Nota: firmware e script sono stati controllati per sintassi e struttura in questo ambiente,
> ma **non sono stati compilati né provati su hardware reale**: le frequenze indicate sono stime teoriche.



### 7. Visualizzatore locale: avvio sul NUC

Estensione del ricevitore esistente: **nessuna modifica a `bluetooth_test.ino`**, nessuna
scansione BLE, nessuna chiamata Connect/Disconnect. Restano le sette letture D2–D8 e la
stampa ordinata di ogni notifica con Hz. Flask serve solo `127.0.0.1`, non la rete locale.

**Sul tuo Intel NUC**, usando il checkout e il venv già esistenti:

```bash
cd ~/Documents/halo_vr_game
git pull --ff-only origin main
source test_preliminari/.venv/bin/activate
python -m pip install -r test_preliminari/bluetooth_test/requirements.txt
cd test_preliminari/bluetooth_test
python test_receiver.py --address 18:FE:03:D9:D4:4B
```

Connetti `IMU_BTN` da **Impostazioni → Bluetooth → Connection ON**, come prima.
Apri **nel browser dello stesso NUC**: **http://127.0.0.1:5000**.
Tieni la scheda ferma per almeno 1,5 s: il riferimento viene acquisito automaticamente.
Il pulsante **Azzera riferimento** prende l'orientamento corrente come nuovo zero.

Varianti (una sola istanza per scheda):

```bash
python test_receiver.py --demo          # dati SINTETICI, nessun accesso a BlueZ
python test_receiver.py --no-web        # solo terminale
python test_receiver.py --port 5050      # se la porta 5000 è occupata
python -m unittest -v test_orientation.py
```

Con `--port 5050`, l'indirizzo sul NUC diventa `http://127.0.0.1:5050`.
Il requirements aggregatore già include questo file: non occorre modificarlo o creare un
secondo venv. Il frontend include Three.js e texture **locali**, senza CDN, font o modelli
remoti; dopo l'installazione non serve internet. Il server non usa debug/reloader.
Ctrl+C o SIGTERM chiudono server e sottoscrizione; la connessione della GUI resta aperta.

### 8. Algoritmo effettivo e motivazione

**Madgwick MARG**: integrazione del giroscopio più correzione a discesa del gradiente
basata su accelerometro e magnetometro. Fallback **Madgwick IMU** (accel+gyro) quando il
magnetometro è nullo o non mostra cambiamenti da 0,5 s; l'interfaccia segnala che lo yaw
può derivare. Il ritorno di valori magnetici diversi riattiva MARG.

Il filtro è implementato in modo esplicito in `orientation.py`, seguendo le equazioni
12, 25/26, 31/32 e 33 del lavoro di Madgwick e l'implementazione di riferimento
[`ahrs` 0.4.0](https://github.com/Mayitzin/ahrs/blob/v0.4.0/ahrs/filters/madgwick.py).
Non è un semplice filtro separato sui tre angoli. La suite confronta gli aggiornamenti
IMU e MARG con `ahrs.filters.Madgwick` su 100 ingressi pseudocasuali ciascuno.

Due protezioni esplicite rispetto a quella versione della libreria: correzione anche con
giroscopio esattamente zero (la libreria restituisce subito il quaternione precedente),
e controllo della norma del gradiente prima della divisione. Nessuna modifica alla
libreria installata. La licenza MIT di riferimento è inclusa in `docs/AHRS-LICENSE.txt`.

**Perché Madgwick:** adatto a questo piccolo strumento 9 assi, costo contenuto, stato in
quaternione e fusione verificabile senza introdurre un EKF e una modellazione del moto
che questo test non richiede. Guadagno `beta=0.06`: compromesso iniziale, non una taratura
sperimentale della tua scheda. `ahrs==0.4.0` resta una dipendenza per i test di equivalenza.

- Accelerazione ricevuta in m/s²; gyro da °/s a **rad/s**; campo magnetico in µT,
  normalizzato dal filtro (la scala uniforme non influisce sulla direzione).
- `dt` misurato con `time.monotonic()` alla ricezione, mai derivato dagli FPS del browser.
  Fusione al massimo a 100 Hz; eventuali passi numerici intermedi non sono nuovi campioni.
- Inizializzazione: almeno 1,5 s di accel prossima a 1 g (±0,8 m/s²), gyro sotto 3 °/s e
  variazione accel tra notifiche sotto 0,3 m/s². Media delle letture, tilt e heading iniziali;
  senza mag si inizializza lo yaw a zero. Questa è una verifica di quiete, **non** stima del
  bias gyro né certificazione della qualità del campo magnetico.
- Quaternione normalizzato a ogni aggiornamento; nessuna doppia integrazione e nessuna
  stima di traslazione: «posizione iniziale» significa esclusivamente orientamento.

### 9. Assi Rev2: fonti e convenzioni riproducibili

Il firmware invia le uscite della libreria, non i registri grezzi del chip. Il profilo
implementato presume Nano 33 BLE Sense **Rev2** e la libreria Arduino_BMI270_BMM150 1.2.1:

1. Nel [codice ufficiale Arduino, tag 1.2.1](https://github.com/arduino-libraries/Arduino_BMI270_BMM150/blob/1.2.1/src/BMI270.cpp),
   `readAcceleration` e `readGyroscope`, sotto `TARGET_ARDUINO_NANO33BLE`, restituiscono
   `x=-raw.y`, `y=-raw.x`, `z=raw.z`. `readMagneticField` restituisce `mag_data.x/y/z`.
   **Non** riapplichiamo al payload la permutazione già fatta dal firmware.
2. [`ReefwingAHRS::setData`, revisione 7a2bf9c](https://github.com/Reefwing-Software/Reefwing-AHRS/blob/7a2bf9c3095d03f949f447e29ed6473adddedb87/src/ReefwingAHRS.cpp)
   applica, per `BMI270_BMM150`, `my=-my` lasciando gli altri canali invariati.
   `rev2_axes()` adotta esplicitamente questo profilo di libreria:
   `a=(ax,ay,az)`, `g=radians(gx,gy,gz)`, `m=(mx,-my,mz)`.

Questa è una verifica delle **fonti software**, non una misura dell'allineamento fisico
sulla tua scheda. Non viene inventata una nuova mappatura sulla base di una schermata,
né affermato che i segni della serigrafia siano stati verificati. Revisioni della libreria,
firmware modificati e diverso montaggio richiedono un controllo sul NUC: scheda ferma,
inclinazioni isolate su ciascun asse, poi rotazione nota di 90°; verificare che accel,
gyro e magnetometro siano coerenti prima di usare l'orientamento in logica applicativa.
Il test matematico verifica il profilo implementato, **non può validare l'hardware**.
Il visualizzatore non è certificato per navigazione o controllo di attuatori.

**Convenzioni:**

- Quaternione Hamilton `[w,x,y,z]`, rotazione attiva corpo→mondo. Il modello matematico
  è destrorso; Z mondiale è la direzione dell'accelerazione di quiete e X è la componente
  orizzontale del campo magnetico, non il nord geografico certificato.
- `q_rel = conjugate(q_ref) ⊗ q_corrente`: orientamento del corpo espresso nella terna
  iniziale. Non sottraiamo roll/pitch/yaw e non usiamo l'ordine inverso del prodotto.
- Euler per i numeri: `R = Rz(yaw) Ry(pitch) Rx(roll)`, gradi; roll/yaw in ±180°,
  pitch in ±90°. Il salto numerico 179°→−179° è normale; vicino a pitch ±90° gli Euler
  hanno singolarità, il quaternione e il velivolo no.
- Aeroplano: muso +X, ala sinistra +Y, dorso +Z. Segni con regola della mano destra;
  non sono la convenzione aeronautica NED (pitch positivo qui abbassa il muso).
- WebGL Y-up: cambio base proprio `B: (x,y,z) → (x,z,−y)`;
  `q_web = B ⊗ q_rel ⊗ B⁻¹`, poi componenti Three.js `[x,y,z,w]`.
  Anche la geometria di base riceve B. Il modello non usa Euler per orientarsi.

### 10. Temporizzazione, freschezza e limiti

Il payload di 37 byte resta invariato: **mancano timestamp del sensore, contatori di
campione e flag di freschezza**. Gli Hz stampati misurano notifiche, non campioni fisici
indipendenti. I dati dei tre sensori non sono necessariamente simultanei; magnetometro
più lento, campioni ripetuti e ritardi BLE possono influire sulla fusione.

La rilevazione di aggiornamento magnetico confronta i valori, non un contatore hardware:
valori identici per 0,5 s sono trattati cautelativamente come non aggiornati. Anche un
sensore fermo ma funzionante può quindi causare fallback IMU; un nuovo valore non prova
che l'intero frame sia fresco. Fra aggiornamenti il mag è mantenuto costante. Il backend
non promette oltre 100 Hz di orientamenti indipendenti né precisione temporale da tracker.

- Oltre 0,5 s senza frame valido: **stale**, aeroplano e numeri nascosti, reset disabilitato.
- Alla disconnessione: **disconnesso**, nessun orientamento valido. Alla ripresa dopo un
  vuoto temporale si richiede nuovamente quiete e si acquisisce un nuovo riferimento;
  nessuna integrazione del gyro attraverso l'intervallo mancante.
- Avvio senza dati: **attesa/inizializzazione**, non un finto orientamento a zero.
- Backend non raggiungibile: interfaccia scollegata e dati nascosti.
- `Azzera riferimento` cambia solo il riferimento: **non calibra** hard/soft iron, bias gyro,
  assi o disturbi da NUC, cavi, magneti e oggetti metallici. Accelerazioni lineari possono
  contaminare la stima della gravità. Prima di misure affidabili serve calibrazione dedicata.

Ricezione e fusione girano nell'event loop asyncio originale; Flask in un thread separato,
snapshot protetti da lock. Il browser legge lo stato circa 20 volte/s e disegna con
`requestAnimationFrame`: nessun polling grafico rallenta intenzionalmente la frequenza BLE.
La stampa di ogni frame è preservata e può comunque costituire un collo di bottiglia.
Il reset usa POST e token per pagina; non è attivabile da GET. Solo loopback, nessun hosting.

### 11. Verifiche e file

- `test_orientation.py`: orientamenti noti, yaw ±180°, riferimento inclinato, cambio base,
  confronto con AHRS, gyro zero, traiettorie sintetiche sui tre assi, mag nullo/bloccato,
  inizializzazione, dati invalidi/stale, Flask/reset e cleanup D-Bus con bus simulato.
- `scripts/verify_browser.py`: integrazione Flask/demo, pulsante reset, conversione nel
  browser, stati senza dati e SIGTERM; necessita Playwright solo per lo sviluppo.
- `docs/VERIFICA.md`: esiti effettivi delle verifiche, incluse immagini GPU.
- `docs/ART_DIRECTION.md`: riferimento del piccolo strumento diagnostico, non di un gioco.

Nessun test eseguito qui legge i tuoi sensori. La demo è esplicitamente marcata e attraversa
lo stesso filtro delle letture reali; i risultati sintetici non certificano accuratezza,
segni fisici, throughput BLE o compatibilità del tuo specifico adattatore BlueZ su Ubuntu 26.
