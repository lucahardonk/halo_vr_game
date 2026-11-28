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

cd test_preliminari/bluetooth_test
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python test_receiver.py
```

Output di esempio:
```
[Hz: 123.4] BTN: 0001010 | Accel: ax=+0.123 ay=-0.456 az=+9.812 | Gyro: gx=+0.01 gy=-0.02 gz=+0.00 | Mag: mx=+12.3 my=-5.6 mz=+8.9
```
Con Ctrl+C lo script si chiude in modo pulito e stampa i frame totali e la frequenza media.
Se la scheda si disconnette, lo script si riconnette da solo.

Per controllare a mano che il dispositivo sia visibile: `bluetoothctl` → `scan on` → cerca `IMU_BTN`.

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
  Esegui i comandi **prima** di avviare lo script (se `conn_max_interval` rifiuta il valore,
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
