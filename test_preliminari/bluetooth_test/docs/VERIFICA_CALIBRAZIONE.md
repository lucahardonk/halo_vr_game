### Verifica della calibrazione guidata — 28 novembre 2026

**Ambito:** estensione del test locale, dati sintetici e backend Flask reale. Nessuna prova sul NUC, sui sensori fisici o sul collegamento radio dell'utente. Firmware, equazioni Madgwick, mapping `rev2_axes`, inizializzazione matematica e scena 3D invariati.

#### Risultati eseguiti

| Verifica | Esito |
|---|---|
| Suite `test_orientation` + `test_calibration` | 27 test superati nella suite completa |
| Aggiunta successiva: errore salvataggio, conservazione profilo, cambio scheda/freschezza e guardia fase errata | Nuovo test superato insieme a 2 regressioni mirate; totale 28 test distinti superati |
| Wizard browser con campioni sintetici e clock controllato, backend reale | 16 controlli superati |
| Avvio `test_receiver.py --demo`, Flask e browser, clock reale | 31 controlli superati, uscita del processo 0 dopo SIGTERM |
| Sintassi Python e JavaScript, `git diff --check` | Superati |
| Confronto AST di `madgwick`, `rev2_axes`, `initial_attitude` con versione precedente | Identici |
| Confronto byte del firmware Arduino tracciato | Identico |

I JSON in `calibrazione/` riportano gli esiti browser. Le immagini mostrano una raccolta planare rifiutata, un wizard completato e la demo in movimento. Sono tutte **simulazioni**, non misure della scheda. Il profilo DEMO generato dal test di avvio è stato rimosso; nessun profilo è incluso nel progetto tracciato.

#### Cosa è stato verificato

- Bias gyro noto non nullo `[2,1; -1,5; 0,7] °/s`: recupero entro la tolleranza di test di 0,01 °/s; movimento, vibrazioni, accelerazione variabile, letture identiche, dati scaduti e timeout non completano la finestra.
- Campo sintetico con offset `[12; -8; 5] µT` e matrice SPD **non diagonale**: offset entro 0,1 µT, coefficienti della correzione entro 0,003 dal valore atteso; errore vettoriale massimo inferiore a 0,2 µT su 600 punti indipendenti, compensando la scala globale non identificabile. Queste sono soglie delle asserzioni superate, non stime di precisione hardware.
- Outlier moderati esclusi; nuvole planari, degeneri, insufficienti, casuali e fortemente mal condizionate rifiutate. Deduplicazione e limite di acquisizione impediscono ai frame BLE ripetuti di gonfiare il conteggio.
- Persistenza validata, separazione reale/demo e per MAC; errore di scrittura, fit rifiutato e annullamento conservano il profilo valido precedente. Profilo presente: nuovo gyro ma nessun obbligo di rifare il mag.
- Correzione applicata una volta sola ai dati grezzi mappati prima del filtro; quaternione normalizzato; reset del filtro dopo calibrazione; nessun riferimento durante le rotazioni, riferimento dopo quiete.
- API protette da token, stati del wizard, riprova, annulla, reset riferimento distinto, attesa, stale e disconnessione.
- Browser: successo e fallimento del fit, istruzioni e progressi, pulsanti reali; aeroplano visibile solo dopo inizializzazione, sette pulsanti digitali, reset, trasformazione assi e arresto del server. Nessun errore console nel percorso verificato.

#### Riproduzione

Dalla directory `test_preliminari/bluetooth_test`, con il venv del progetto attivo:

```bash
python -m unittest -v test_orientation.py test_calibration.py
python scripts/verify_calibration_browser.py --out /tmp/verifica-wizard
python scripts/verify_browser.py --out /tmp/verifica-demo
```

Gli script browser richiedono Playwright e Chromium nell'ambiente di sviluppo: non sono dipendenze dell'operatore NUC. La demo avviata dall'ultimo script può salvare il proprio profilo isolato in `.calibration/demo/SIMULATED.json`; per ripetere il primo avvio completo, eliminare solo quel profilo demo, mai quello reale.

Il primo tentativo dello smoke test demo è terminato in timeout in attesa dell'orientamento. Dopo aver aggiornato il driver per attendere gli stati DOM e completare la raccolta, la seconda esecuzione ha superato tutti i 31 controlli. Nel test wizard la prima asserzione sul conteggio esatto è stata corretta: la deduplicazione elimina legittimamente un punto sintetico, quindi si verifica il numero effettivamente accettato, non quello inviato.

#### Limiti espliciti

La verifica browser locale usa SwiftShader solo per correttezza funzionale; non misura prestazioni sul NUC. Nessuna nuova valutazione GPU della grafica, che non è stata modificata. La calibrazione non elimina disturbi magnetici variabili, deriva termica del gyro o errori di allineamento fisico; il payload non contiene timestamp/contatori per certificare l'età fisica dei campioni. Il miglioramento effettivo del drift resta da misurare sulla scheda reale, senza cambiare Madgwick in questa estensione.
