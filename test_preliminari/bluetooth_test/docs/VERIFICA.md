### Esito della verifica

**Software verificato con dati sintetici; hardware reale non verificato.** Nessun accesso al NUC o alla sua scheda è avvenuto.

#### Risultati

- Sintassi Python dei moduli e dello script di verifica: superata (`py_compile`).
- Sintassi modulo JavaScript: superata (`node --input-type=module --check`).
- Suite `python -m unittest -v test_orientation.py`: **15 test superati**. Include equivalenza numerica con AHRS 0.4.0, inclinazioni e rotazioni note, yaw vicino a ±180°, normalizzazione, reset relativo, fallback magnetometro, dati scaduti e cleanup D-Bus simulato. Dopo la correzione di una chiusura di file nel test Flask, il test interessato è stato rieseguito con successo.
- Avvio CLI reale in modalità `--demo --no-web`, stampa e arresto SIGINT: confermati, uscita 0.
- Avvio Flask reale con demo, endpoint, frontend e click reset: verificati anche nel browser locale.
- Verifica end-to-end su **NVIDIA L4**, renderer `ANGLE (NVIDIA, Vulkan 1.3.289 (NVIDIA NVIDIA L4 (0x000027B8)), NVIDIA)`: **32 controlli superati**, uscita 0. Rapporto completo `verifica-browser.json`. Include rotazione frontend sui tre assi e angoli ±179/±180°, click reset, sette pulsanti, banner demo, assenza errori browser, dati mancanti e arresto SIGTERM del processo completo.
- Job GPU finale: `49eb5724-9290-4903-8347-0e5cc2a4c9dd`. Eseguito con antialias, PBR e mappe locali, environment, illuminazione e postprocess completi. Non è stato eseguito un benchmark FPS/headroom: nessuna prestazione del NUC è deducibile da questo test. La velocità della vista locale software non è rappresentativa del dispositivo dell'utente.
- Il test browser usa risposte API sintetiche dichiarate per verificare le schermate attesa/stale/disconnesso/inizializzazione. La transizione temporale effettiva del backend è verificata separatamente nei test con clock controllato. Non sono state simulate connessioni BLE spacciandole per connessioni reali.
- Firmware `bluetooth_test.ino` invariato, verificato con `git diff --exit-code`.

#### Verifica visiva

Le immagini `orientamento.png` e `riferimento-azzerato.png` sono state aperte e controllate.
La demo continua a muoversi dopo il click: piccoli angoli non nulli nella seconda immagine sono quindi attesi; l'identità esatta all'istante di reset è controllata dal test matematico.

Confronto con `ART_DIRECTION.md` in `compare_strumento.png`: palette, vista a tre quarti, aeroplano completo, numeri leggibili e banner demo corrispondono. Nella prima iterazione i riflessi sulla coda erano abbaglianti; sono stati corretti aumentando la rugosità e bilanciando le luci, senza rimuovere mappe o postprocess. Il risultato resta volutamente un simbolo diagnostico, con cerchi molto discreti e geometria semplificata, non un prodotto o una scena fotorealistica. La valutazione soggettiva delle sei componenti è in `REALISM_SCORE.json` e si riferisce a questo obiettivo diagnostico.

#### Problemi incontrati, risolti

La preparazione iniziale del container GPU non aveva pip/Playwright e il percorso browser predefinito non era scrivibile. Le dipendenze e il browser di verifica sono stati installati nella directory temporanea del job, senza modificare il software consegnato. Una callback del test Playwright riceveva la richiesta nel parametro destinato al fixture: corretta e verificata nel job finale. Nessuno di questi errori è stato classificato come difetto del BLE o del NUC.

#### Verifiche ancora da fare sul NUC

Compatibilità adattatore/BlueZ su Ubuntu 26, ricezione reale a 37 byte, frequenza effettiva, allineamento fisico degli assi, rumore/bias, calibrazione magnetica e comportamento con interferenze. Il README distingue esplicitamente profilo di libreria verificato nel sorgente e allineamento fisico non misurato. Nessuna promessa di precisione di tracking, >100 Hz di campioni indipendenti o navigazione affidabile.
