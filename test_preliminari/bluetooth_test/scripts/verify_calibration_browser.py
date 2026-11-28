"""Test UI con stato reale e campioni sintetici; clock controllato, nessuna API fixture pubblica.

Esecuzione: python scripts/verify_calibration_browser.py --out /tmp/verifica-calibrazione
Playwright è solo strumento di sviluppo, non dipendenza del NUC.
"""
import argparse
import json
from pathlib import Path
import sys
import tempfile

import numpy as np
from playwright.sync_api import sync_playwright

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from calibration import Calibration, ProfileStore
from orientation import Orientation
from viewer import Viewer


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--out',default='/tmp/verifica-calibrazione')
    args=parser.parse_args()
    out=Path(args.out);out.mkdir(parents=True,exist_ok=True)
    checks=[]
    with tempfile.TemporaryDirectory() as temporary:
        c=Calibration(ProfileStore(temporary),demo=True,device='SIMULATED')
        clock=[0.]
        state=Orientation(demo=True,clock=lambda:clock[0],calibration=c)
        server=Viewer(state,0);server.start()
        rng=np.random.default_rng(172)
        bias=np.array([2.1,-1.5,.7])
        S=np.array([[1.2,.2,.1],[.2,.9,-.1],[.1,-.1,1.1]])
        def feed(count, kind='fermo'):
            for i in range(count):
                clock[0]+=.1 if kind in ('mag','planare') else .025
                g=bias+rng.normal(0,.015,3)
                a=np.array([0,0,9.80665])+rng.normal(0,.003,3)
                if kind=='movimento':g+= [3*(-1)**i,0,0]
                if kind=='mag':
                    v=rng.normal(size=3);v=v/np.linalg.norm(v)*45
                    m=S@v+[12,-8,5]
                elif kind=='planare':m=np.array([40*np.cos(i*.1),40*np.sin(i*.1),1])
                else:m=S@np.array([24,0,39])+[12,-8,5]+rng.normal(0,.001,3)
                state.feed([5,*a,*g,*(m*[1,-1,1])],clock[0])
        def check(name, condition):
            checks.append({'nome':name,'superato':bool(condition)})
            assert condition,name
        try:
            with sync_playwright() as p:
                browser=p.chromium.launch(headless=True,args=['--no-sandbox','--use-gl=angle','--use-angle=swiftshader','--enable-webgl','--ignore-gpu-blocklist'])
                page=browser.new_page(viewport={'width':1400,'height':1100})
                errors=[]
                page.on('pageerror',lambda e:errors.append(str(e)))
                page.on('console',lambda m:errors.append(m.text) if m.type=='error' else None)
                page.goto(f'http://127.0.0.1:{server.server.server_port}/?quality=low')
                page.wait_for_function('window.__IMU_VIEW__?.getState().frames>0',timeout=60000)
                def title(text):
                    page.wait_for_function('t=>document.querySelector("#cal-title").textContent===t',arg=text,timeout=20000)
                page.wait_for_function('document.querySelector("#cal-message").textContent.includes("attesa")')
                check('attesa senza avanzamento',c.elapsed==0 and not state.snapshot()['valid'])
                state.connection(True)
                feed(80);title('1 / 2 · Giroscopio')
                page.wait_for_function('Number(document.querySelector("#cal-progress").value)>0')
                check('countdown da dati validi',0<c.elapsed<4)
                feed(1,'movimento')
                page.wait_for_function('document.querySelector("#cal-message").textContent.includes("Movimento")')
                check('movimento azzera countdown',c.elapsed==0)
                feed(180);title('2 / 2 · Magnetometro')
                check('gyro sintetico stimato',np.max(np.abs(np.degrees(c.bias)-bias))<.02)
                page.locator('[data-cal-action=mag]').click();title('2 / 2 · Raccolta magnetica')
                feed(20,'mag')
                page.locator('[data-cal-action=verifica]').click();title('Raccolta non accettata')
                check('raccolta insufficiente rifiutata',c.profile is None)
                page.locator('[data-cal-action=riprova]').click();title('2 / 2 · Raccolta magnetica')
                feed(260,'planare')
                page.locator('[data-cal-action=verifica]').click();title('Raccolta non accettata')
                check('raccolta planare rifiutata','planare' in c.message and c.profile is None)
                page.screenshot(path=str(out/'raccolta-rifiutata.png'),full_page=True)
                page.locator('[data-cal-action=riprova]').click();title('2 / 2 · Raccolta magnetica')
                feed(500,'mag')
                check('deduplicazione mantiene campioni sufficienti',len(c.samples)>=180)
                page.wait_for_function('n=>document.querySelector("#cal-details").textContent.startsWith(n+" campioni")',arg=len(c.samples))
                page.screenshot(path=str(out/'raccolta-completa.png'),full_page=True)
                page.locator('[data-cal-action=verifica]').click();title('Riferimento iniziale')
                check('successo salva profilo demo',c.profile is not None and c.profile['demo'])
                check('nessun riferimento durante rotazioni',state.q is None and page.locator('#reset').is_disabled())
                feed(70);title('Calibrazione completata')
                page.wait_for_function('window.__IMU_VIEW__.getState().visible')
                check('riferimento acquisito solo dopo quiete',state.snapshot()['valid'])
                page.screenshot(path=str(out/'calibrazione-completata.png'),full_page=True)
                page.locator('#reset').click()
                page.wait_for_function('document.querySelector("#feedback").textContent.includes("azzerato")')
                check('reset distinto e disponibile',state.reset_count==1 and c.phase=='pronto')
                saved=c.profile.copy()
                page.locator('[data-cal-action=mag]').click();title('2 / 2 · Raccolta magnetica')
                page.locator('[data-cal-action=annulla]').click();title('Riferimento iniziale')
                check('annulla conserva profilo',c.profile==saved)
                page.locator('[data-cal-action=gyro]').click();title('1 / 2 · Giroscopio')
                feed(50)
                clock[0]+=1
                page.wait_for_function('document.querySelector("#cal-message").textContent.includes("attesa")')
                check('stale ferma acquisizione',c.elapsed==0)
                state.connection(False)
                page.wait_for_function('document.querySelector("#status").textContent.includes("disconnessa")')
                check('disconnessione visibile',not state.snapshot()['valid'])
                check('demo sempre esplicita','sintetica' in page.locator('#cal-mode').inner_text())
                check('nessun errore browser',not errors)
                browser.close()
        finally:
            server.close()
    (out/'verifica-calibrazione-browser.json').write_text(json.dumps({'ambiente':'Simulazione locale; nessun sensore reale; grafica invariata','checks':checks},indent=2,ensure_ascii=False))
    print(json.dumps({'controlli':len(checks),'tutti_superati':True}))


if __name__=='__main__':main()
