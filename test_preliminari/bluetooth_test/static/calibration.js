// Interfaccia del wizard: solo stato backend; nessun timer locale finge acquisizioni.
const el = id => document.getElementById(id);
const labels = {attesa:'Attesa BLE',gyro:'1 / 2 · Giroscopio',mag_pronto:'2 / 2 · Magnetometro',mag:'2 / 2 · Raccolta magnetica',errore:'Raccolta non accettata',annullato:'Procedura annullata',riferimento:'Riferimento iniziale',pronto:'Calibrazione completata'};
let busy = false;
let current = null;
export function updateCalibration(c) {
  current = c;
  if (!c) { el('calibration').hidden=true; return; }
  el('calibration').hidden=false;
  el('cal-title').textContent=labels[c.phase]||'Calibrazione';
  el('cal-message').textContent=c.message;
  el('cal-mode').textContent=c.demo?'DEMO · calibrazione sintetica, non della scheda':`Scheda: ${c.device||'in attesa di identificazione'}`;
  el('cal-notice').textContent=c.notice||'';
  el('cal-profile').textContent=c.profile?'Profilo magnetico locale valido disponibile; una nuova raccolta lo sostituisce solo se accettata.':'Nessun profilo magnetico: completa entrambe le fasi.';
  el('cal-progress').value=c.phase==='gyro'?c.progress:(c.phase==='pronto'?1:Math.min(c.samples/180,1));
  el('cal-details').textContent=c.phase==='gyro'?`Mancano ${c.seconds_remaining.toFixed(1)} s di quiete valida.`:`${c.samples} campioni utili · ${c.sectors}/${c.sectors_total} settori esplorati (stima) · ${c.rejected} fuori limite`;
  el('cal-missing').textContent=['mag','errore'].includes(c.phase)?`Direzioni da esplorare: ${c.missing.length?c.missing.join(', '):'nessuna faccia mancante; serve comunque un fit valido'}. Minimo 180 campioni: il numero da solo non basta.`:'';
  el('cal-bias').textContent=c.bias_dps?`Bias gyro [°/s]: ${c.bias_dps.map(v=>v.toFixed(3)).join(' / ')}`:'';
  for (const button of document.querySelectorAll('[data-cal-action]')) {
    const action=button.dataset.calAction;
    button.hidden = action==='mag'?!(c.bias_dps && ['mag_pronto','pronto','riferimento','annullato'].includes(c.phase)):
      action==='verifica'?!['mag','errore'].includes(c.phase):
      action==='riprova'?!['errore','annullato'].includes(c.phase):
      action==='annulla'?!['gyro','mag','errore','mag_pronto'].includes(c.phase):
      action==='gyro'?!['pronto','riferimento'].includes(c.phase):false;
    button.disabled=busy || (action!=='annulla' && !c.fresh);
  }
}
export function calibrationOffline(){
  el('cal-message').textContent='Server non raggiungibile: acquisizione non confermata. Nessun avanzamento.';
  for(const button of document.querySelectorAll('[data-cal-action]'))button.disabled=true;
}
for(const button of document.querySelectorAll('[data-cal-action]'))button.addEventListener('click',async()=>{
  busy=true;if(current)updateCalibration(current);
  el('cal-error').textContent='';
  try {
    const response=await fetch(`/api/calibration/${button.dataset.calAction}`,{method:'POST',headers:{'X-Reset-Token':document.querySelector('meta[name=reset-token]').content},signal:AbortSignal.timeout(10000)});
    const data=await response.json();
    if(!response.ok)throw Error(data.error||'Operazione non disponibile');
    updateCalibration(data.state.calibration);
  }catch(error){el('cal-error').textContent=error.message;}
  finally{busy=false;if(current)updateCalibration(current);}
});
