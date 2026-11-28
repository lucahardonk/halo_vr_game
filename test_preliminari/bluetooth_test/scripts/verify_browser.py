"""Verifica integrazione reale Flask/demo e WebGL; Playwright solo per lo sviluppo."""
import argparse
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import urllib.request

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--gpu',action='store_true')
    parser.add_argument('--out',default=os.environ.get('OUT','/tmp/imu-browser-check'))
    args=parser.parse_args();out=Path(args.out);out.mkdir(parents=True,exist_ok=True)
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
    url=f'http://127.0.0.1:{port}'
    log=(out/'server.log').open('w')
    proc=subprocess.Popen([sys.executable,'test_receiver.py','--demo','--port',str(port)],cwd=ROOT,stdout=log,stderr=log)
    checks=[]
    def check(name,condition,detail=''):
        checks.append(dict(name=name,passato=bool(condition),dettaglio=detail))
        assert condition,(name,detail)
    try:
        for _ in range(60):
            try:
                with urllib.request.urlopen(url+'/api/state',timeout=.5) as r: state=json.load(r)
                break
            except OSError:time.sleep(.1)
        else:raise RuntimeError('Avvio Flask fallito')
        check('demo esplicita',state['demo'])
        check('nessun orientamento prima inizializzazione',not state['valid'] and state['quaternion'] is None)
        with sync_playwright() as p:
            flags=['--no-sandbox']+(['--use-angle=vulkan','--enable-features=Vulkan','--disable-vulkan-surface','--ignore-gpu-blocklist','--enable-gpu'] if args.gpu else ['--use-gl=angle','--use-angle=swiftshader','--enable-webgl','--ignore-gpu-blocklist'])
            browser=p.chromium.launch(headless=True,args=flags)
            page=browser.new_page(viewport={'width':1400,'height':960},device_scale_factor=1)
            errors=[]
            page.on('pageerror',lambda e:errors.append(str(e)))
            page.on('console',lambda m:errors.append(m.text) if m.type=='error' else None)
            page.on('response',lambda r:errors.append(f'HTTP {r.status} {r.url}') if r.status>=400 else None)
            page.goto(url+'/?quality='+('high' if args.gpu else 'low'))
            page.wait_for_function('window.__IMU_VIEW__?.getState().frames > 3',timeout=60000)
            page.wait_for_function("['2 / 2 · Magnetometro','Calibrazione completata'].includes(document.querySelector('#cal-title').textContent)",timeout=30000)
            calibration=page.evaluate("fetch('/api/state').then(r=>r.json()).then(s=>s.calibration)")
            if calibration['phase']=='mag_pronto':
                page.locator('[data-cal-action=mag]').click()
                page.wait_for_function("parseInt(document.querySelector('#cal-details').textContent)>=550",timeout=100000)
                page.locator('[data-cal-action=verifica]').click()
            page.wait_for_function('window.__IMU_VIEW__.getState().visible',timeout=30000)
            renderer=page.evaluate("""() => { const gl=document.querySelector('canvas').getContext('webgl2'); const ext=gl.getExtension('WEBGL_debug_renderer_info'); return gl.getParameter(ext.UNMASKED_RENDERER_WEBGL); }""")
            if args.gpu:check('renderer NVIDIA','NVIDIA' in renderer,renderer)
            check('sette pulsanti',page.locator('#buttons span').count()==7)
            check('banner demo','SINTETICI' in page.locator('#source').inner_text())
            page.wait_for_function("Math.abs(parseFloat(document.querySelector('#yaw').textContent)) > 10",timeout=30000)
            page.screenshot(path=str(out/'orientamento.png'))
            before=page.evaluate("fetch('/api/state').then(r=>r.json())")
            page.locator('#reset').click()
            page.wait_for_function("document.querySelector('#feedback').textContent.includes('azzerato')")
            after=page.evaluate("fetch('/api/state').then(r=>r.json())")
            check('reset da pulsante',after['resets']==before['resets']+1)
            check('reset prossimo a identità',abs(after['quaternion'][0])>.98,str(after['rpy']))
            page.screenshot(path=str(out/'riferimento-azzerato.png'))
            for axis in range(3):
                for degrees in [30,-30,179,-179,180,-180]:
                    half=math.radians(degrees)/2;q=[math.cos(half),0,0,0];q[axis+1]=math.sin(half)
                    transformed=page.evaluate('q=>window.__IMU_VIEW__.transform(q)',q)
                    expected=[q[1],q[3],-q[2],q[0]]
                    check(f'base frontend asse {axis} angolo {degrees}',max(abs(a-b) for a,b in zip(transformed,expected))<1e-9)
            # Stato API simulato SOLO per provare tutte le schermate di indisponibilità.
            for status in ['stale','disconnesso','attesa','inizializzazione']:
                payload={**after,'status':status,'valid':False,'rpy':None,'quaternion':None,'buttons':None}
                page.route('**/api/state',lambda route,request,data=payload:route.fulfill(json=data))
                expected={'stale':'Dati non aggiornati','disconnesso':'Scheda disconnessa','attesa':'In attesa di dati','inizializzazione':'Inizializzazione · tieni ferma la scheda'}[status]
                page.wait_for_function("text=>document.querySelector('#status').textContent===text",arg=expected)
                check(f'interfaccia {status}',not page.evaluate('window.__IMU_VIEW__.getState().visible') and page.locator('#reset').is_disabled())
                if status=='stale':page.screenshot(path=str(out/'dati-assenti.png'))
                page.unroute('**/api/state')
                page.wait_for_function('window.__IMU_VIEW__.getState().visible')
            check('nessun errore browser',not errors,str(errors))
            proc.terminate();proc.wait(timeout=8)
            check('SIGTERM arresta il processo',proc.returncode==0)
            page.wait_for_function("document.querySelector('#status').textContent==='Visualizzatore scollegato'",timeout=10000)
            check('server assente nasconde orientamento',not page.evaluate('window.__IMU_VIEW__.getState().visible'))
            browser.close()
            (out/'verifica-browser.json').write_text(json.dumps(dict(renderer=renderer,checks=checks),indent=2,ensure_ascii=False))
            print(json.dumps(dict(renderer=renderer,verifiche=len(checks),tutte_passate=True),ensure_ascii=False))
    except Exception:
        try:
            with urllib.request.urlopen(url+'/api/state',timeout=2) as response:
                (out/'stato-al-fallimento.json').write_text(response.read().decode())
        except OSError:
            pass
        raise
    finally:
        if proc.poll() is None:proc.terminate();proc.wait(timeout=8)
        log.close()


if __name__=='__main__':main()
