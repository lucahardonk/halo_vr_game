### Dipendenze e risorse locali

- **Three.js 0.170.0**, MIT, pacchetto npm ufficiale `three@0.170.0`. Inclusi solo modulo base e dipendenze transitive dei pass di rendering/environment utilizzati. Licenza originale in `static/vendor/LICENSE`.
- **Blue Metal Plate**, Poly Haven, CC0: https://polyhaven.com/a/blue_metal_plate . Mappe JPG 1K colore, normale OpenGL e rugosità, scaricate tramite https://api.polyhaven.com/files/blue_metal_plate ; file locali in `static/material/`. La rugosità è rimappata nell'intervallo 160–255 per evitare riflessi abbaglianti nello strumento. Politica di licenza: https://polyhaven.com/license . Nessun caricamento esterno a runtime.
- **Aeroplano diagnostico:** geometria parametrica originale in `static/app.js`, creata appositamente come simbolo di assetto (non riproduzione di un aeromobile reale). Fusoliera lathe, ali e coda estruse con bordi smussati, cabina ellissoidale. Nessun asset artistico o modello di prodotto generato.
- **AHRS 0.4.0**, MIT: https://github.com/Mayitzin/ahrs/tree/v0.4.0 . Equazioni/gradiente di Madgwick in `orientation.py` confrontati con questa implementazione; licenza originale in `docs/AHRS-LICENSE.txt`.
- **Reefwing-AHRS**: fonte consultata per il solo profilo assi BMI270_BMM150, revisione `7a2bf9c3095d03f949f447e29ed6473adddedb87`. Nessun codice della libreria distribuito come dipendenza. Collegamento preciso nel README.

Le immagini in `docs/` sono acquisizioni della modalità **demo sintetica**, non misure della scheda dell'utente.
