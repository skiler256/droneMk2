/* ============================================================================
   volume.js — controle du volume systeme (PulseAudio, via server.js ::
   /api/volume, /api/mute/:state). Widget autonome, aucune dependance sur le
   reste de la page -- meme pattern que le badge de deblocage audio dans
   alarm.js.
   ========================================================================== */

(function (global) {
  'use strict';

  var box = document.createElement('div');
  box.style.cssText = 'position:fixed;left:.6rem;bottom:.6rem;z-index:9999;' +
    'display:flex;align-items:center;gap:.5rem;' +
    'font:11px ui-monospace,monospace;padding:.35rem .6rem;border-radius:6px;' +
    'background:rgba(20,20,25,.85);color:#ddd;border:1px solid #444;';

  var muteBtn = document.createElement('button');
  muteBtn.type = 'button';
  muteBtn.style.cssText = 'cursor:pointer;background:none;border:none;color:inherit;' +
    'font-size:15px;padding:0;line-height:1;';

  var slider = document.createElement('input');
  slider.type = 'range';
  slider.min = '0';
  slider.max = '100';
  slider.value = '70';
  slider.style.cssText = 'width:90px;';

  box.appendChild(muteBtn);
  box.appendChild(slider);

  var muted = false;

  function setIcon() {
    var pct = parseInt(slider.value, 10);
    muteBtn.textContent = muted ? '🔇' : (pct === 0 ? '🔈' : (pct < 50 ? '🔉' : '🔊'));
  }

  function refresh() {
    fetch('/api/volume').then(function (r) { return r.json(); }).then(function (d) {
      if (typeof d.percent === 'number') slider.value = String(d.percent);
      muted = !!d.muted;
      setIcon();
    }).catch(function () { /* pas grave -- pulseaudio pas encore pret, on reessaiera */ });
  }

  slider.addEventListener('input', setIcon);
  slider.addEventListener('change', function () {
    fetch('/api/volume/' + slider.value, { method: 'POST' })
      .catch(function (e) { console.warn('[gcs] volume: reglage echoue', e.message); });
  });

  muteBtn.addEventListener('click', function () {
    muted = !muted;
    setIcon();
    fetch('/api/mute/' + (muted ? '1' : '0'), { method: 'POST' })
      .catch(function (e) { console.warn('[gcs] volume: mute echoue', e.message); });
  });

  document.addEventListener('DOMContentLoaded', function () {
    document.body.appendChild(box);
    refresh();
  });

})(window);
