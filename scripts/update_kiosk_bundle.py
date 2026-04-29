from __future__ import annotations

import json
import re
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parent.parent
SOURCE_HTML = REPO_ROOT / "Cove Matcha Kiosk - standalone.html"
PACKAGE_HTML = REPO_ROOT / "src/cove_kiosk_bridge/web/index.html"
INSTALL_HTML = REPO_ROOT / "install/cove_kiosk_bridge/share/cove_kiosk_bridge/web/index.html"

START_MARKER = "<script>\nconst TWEAK_DEFAULTS = /*EDITMODE-BEGIN*/"
END_MARKER = "\n</script>\n\n\n</body></html>"

NEW_SCRIPT = """<script>
const TWEAK_DEFAULTS = /*EDITMODE-BEGIN*/{
  "accentHue": 140,
  "speed": 3,
  "showDotmatrix": false,
  "headline": "order <em>matcha</em>."
}/*EDITMODE-END*/;

const API_BASE = '';
const TESTNAMES = ['A. Chen', 'M. Yong', 'J. Rodriguez', 'I. Chan', 'J. Cappo', 'K. Park', 'R. Tanaka', 'S. Bauer'];
const STEPS = ['idle', 'received', 'pouring', 'moving', 'arrived'];
const LABELS = {
  idle: { headline: null, sub: null, activity: 'Ready for your order', tag: 'waiting for input', status: 'System - Nominal', dot: 'idle' },
  received: { headline: 'order <em>received.</em>', sub: 'Cove is starting your order now.', activity: 'Starting your order', tag: 'confirming', status: 'System - Order received', dot: 'idle' },
  pouring: { headline: 'pouring <em>your matcha.</em>', sub: 'Your drink is being prepared now.', activity: 'Pouring your drink', tag: 'in progress', status: 'System - Pouring', dot: 'idle' },
  moving: { headline: 'on the <em>way</em>.', sub: 'Cove is moving your order to pickup.', activity: 'Moving your drink to pickup', tag: 'transit', status: 'System - In transit', dot: 'idle' },
  arrived: { headline: 'arrived. <em>enjoy.</em>', sub: 'Your order is ready for pickup.', activity: 'Giving you your drink', tag: 'complete', status: 'System - Delivered', dot: 'idle' },
  error: { headline: 'we need a <em>minute</em>.', sub: 'A fault was detected and the operator panel is holding the queue.', activity: 'System needs attention', tag: 'attention required', status: 'System - Fault', dot: 'err' },
};

let tw = { ...TWEAK_DEFAULTS };
let state = 'idle';
let lastAppliedState = null;
let queue = [];
let currentOrder = null;
let currentMotion = {};
let servedToday = 37;
let remotePhaseProgress = 0;
let soundOn = true;
let apiOnline = false;
let submittingOrder = false;
let lastSnapshot = null;
let lastSpokenOrderId = null;
let activeAnnouncementAudio = null;

function simplifyChrome() {
  const style = document.createElement('style');
  style.textContent = `
    .grid { grid-template-columns: 1fr !important; }
    .hero {
      border-right: none !important;
      padding: 104px 72px 72px !important;
      display: grid !important;
      place-items: center !important;
    }
    .hero > div:first-child,
    .order-meta,
    .rail,
    #oppanel,
    #opBtn,
    #soundBtn,
    #tweaks,
    .botbar,
    .fill-indicator,
    .topbar .meta,
    .logo .divider,
    .logo .robot {
      display: none !important;
    }
    .topbar {
      justify-content: flex-start !important;
      padding: 0 36px !important;
    }
    .topbar .logo img {
      height: 44px !important;
    }
    .order-block {
      position: relative !important;
      z-index: 4 !important;
      display: grid !important;
      grid-template-columns: 1fr 1fr !important;
      align-items: center !important;
      column-gap: 72px !important;
      width: 100% !important;
      max-width: 100% !important;
      margin: 0 !important;
    }
    .order-btn {
      width: 580px !important;
      height: 580px !important;
      flex: 0 0 580px !important;
      font-size: 88px !important;
      gap: 16px !important;
      justify-self: end !important;
    }
    .order-btn .tap {
      margin-top: 12px !important;
      font-size: 18px !important;
    }
    .hero-figure {
      position: relative !important;
      right: auto !important;
      bottom: auto !important;
      width: 700px !important;
      height: 700px !important;
      flex: 0 0 700px !important;
      opacity: 1 !important;
      order: 2 !important;
      margin: 0 !important;
      justify-self: start !important;
    }
    .activity-text {
      position: absolute !important;
      left: 50% !important;
      bottom: 118px !important;
      transform: translateX(-50%) !important;
      z-index: 6 !important;
      min-width: 420px !important;
      padding: 14px 24px !important;
      border: 1px solid rgba(157,218,163,.22) !important;
      background: rgba(15,30,26,.78) !important;
      color: var(--lime) !important;
      text-align: center !important;
      font-family: "Instrument Sans" !important;
      font-weight: 600 !important;
      font-size: 28px !important;
      letter-spacing: -.01em !important;
      box-shadow: 0 18px 48px rgba(0,0,0,.28) !important;
    }
  `;
  document.head.appendChild(style);

  const orderBlock = document.querySelector('.order-block');
  const heroFigure = document.getElementById('heroFigure');
  if (orderBlock && heroFigure) {
    orderBlock.appendChild(heroFigure);
  }

  let activityText = document.getElementById('activityText');
  if (!activityText) {
    activityText = document.createElement('div');
    activityText.id = 'activityText';
    activityText.className = 'activity-text';
    activityText.textContent = 'Ready for your order';
    const hero = document.getElementById('hero');
    if (hero) hero.appendChild(activityText);
  }

  const eyebrow = document.getElementById('eyebrow-label');
  if (eyebrow) eyebrow.textContent = 'cove · r-01 · matcha';

  const orderMetaBig = document.querySelector('.order-meta .big');
  if (orderMetaBig) orderMetaBig.innerHTML = 'fresh <em>matcha</em>';

  const orderMetaNote = document.querySelector('.order-meta .note');
  if (orderMetaNote) orderMetaNote.innerHTML = 'single button · automated pour<br>pickup when ready';

  const specs = document.querySelectorAll('.specs div');
  if (specs.length >= 3) {
    specs[0].innerHTML = 'drink<span>matcha</span>';
    specs[1].innerHTML = 'eta<span>~45s</span>';
    specs[2].innerHTML = 'pickup<span>auto</span>';
  }

  const receiptLines = document.querySelectorAll('.receipt .line');
  receiptLines.forEach((line) => {
    const label = line.querySelector('span');
    const value = line.querySelector('b');
    if (!label || !value) return;
    if (label.textContent.trim() === 'volume') value.textContent = '240ml matcha';
  });
}

function fit() {
  const c = document.getElementById('canvas');
  const w = window.innerWidth || document.documentElement.clientWidth;
  const h = window.innerHeight || document.documentElement.clientHeight;
  if (!w || !h) {
    requestAnimationFrame(fit);
    return;
  }
  const s = Math.min(w / 1920, h / 1080);
  c.style.transform = `scale(${s})`;
}
window.addEventListener('resize', fit);
window.addEventListener('load', fit);
requestAnimationFrame(fit);
fit();

function tick() {
  const d = new Date();
  const hh = String(d.getHours()).padStart(2, '0');
  const mm = String(d.getMinutes()).padStart(2, '0');
  const ss = String(d.getSeconds()).padStart(2, '0');
  document.getElementById('clock').textContent = `${hh}:${mm}:${ss}`;
}
tick();
setInterval(tick, 1000);

function escapeHtml(value) {
  return String(value).replace(/[&<>"']/g, (ch) => (
    { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[ch]
  ));
}

function formatSeconds(totalSeconds) {
  const safe = Math.max(0, Math.round(Number(totalSeconds) || 0));
  const minutes = Math.floor(safe / 60);
  const seconds = String(safe % 60).padStart(2, '0');
  return `${minutes}:${seconds}`;
}

function updateOrderButtonAvailability() {
  const btn = document.getElementById('orderBtn');
  const disabled = !apiOnline || submittingOrder || state !== 'idle';
  btn.disabled = disabled;
  btn.querySelector('.pulse-ring').style.animationPlayState = disabled ? 'paused' : 'running';
}

function renderQueue() {
  const q = document.getElementById('queue');
  const visible = queue.filter((order) => order.status !== 'done' || order.isMe);
  if (visible.length === 0) {
    q.innerHTML = '<div class="queue-empty">no orders in queue - Cove is ready.</div>';
    document.getElementById('queuetag').textContent = 'no orders';
  } else {
    q.innerHTML = visible.slice(0, 5).map((order, index) => {
      const cls = [
        'queue-row',
        order.status === 'active' ? 'active' : '',
        order.isMe ? 'me' : '',
        order.status === 'done' ? 'done' : '',
      ].join(' ');
      const labels = { queued: 'queued', active: 'preparing', done: 'delivered' };
      return `<div class="${cls}"><span class="num">${index + 1}</span><span class="who">order #${order.id} - ${escapeHtml(order.name)}</span><span class="tstamp">${labels[order.status] || order.status}</span></div>`;
    }).join('');
    const queued = queue.filter((order) => order.status === 'queued').length;
    const active = queue.filter((order) => order.status === 'active').length;
    document.getElementById('queuetag').textContent = active ? `${queued} waiting - ${active} active` : (queued ? `${queued} waiting` : 'complete');
  }
  document.getElementById('op-qcount').textContent = queue.filter((order) => order.status !== 'done').length;
}

function renderSlots() {
  const slots = document.querySelectorAll('.slot');
  slots.forEach((slotEl) => {
    const slotNumber = Number(slotEl.dataset.slot);
    const order = queue.find((item) => item.slot === slotNumber && item.status !== 'done');
    const served = queue.find((item) => item.slot === slotNumber && item.status === 'done' && item.isMe);
    slotEl.classList.remove('occupied', 'mine', 'open');
    const label = slotEl.querySelector('.status');
    if (served) {
      slotEl.classList.add('mine', 'open');
      label.textContent = 'open ' + served.name;
    } else if (order) {
      if (order.isMe) {
        slotEl.classList.add('mine');
        label.textContent = order.status === 'active' ? 'reserved' : 'incoming';
      } else {
        slotEl.classList.add('occupied');
        label.textContent = 'occupied';
      }
    } else {
      label.textContent = 'open';
    }
  });
}

function setFillProgress(progress) {
  const pct = Math.max(0, Math.min(1, Number(progress) || 0));
  document.getElementById('fillBar').style.width = `${Math.round(pct * 100)}%`;
  document.getElementById('fillPct').textContent = `${Math.round(pct * 100)}%`;
}

function motionActivityText(fallback) {
  if (!currentMotion) return fallback;
  const step = currentMotion.step || '';
  const message = currentMotion.message || '';
  if (step && message && step !== message) return `${step} - ${message}`;
  return step || message || fallback;
}

function beep(nextState) {
  if (!soundOn) return;
  try {
    const ctx = window.__ac || (window.__ac = new (window.AudioContext || window.webkitAudioContext)());
    const osc = ctx.createOscillator();
    const gain = ctx.createGain();
    const freqs = { received: 660, pouring: 440, moving: 520, arrived: 880, error: 180, idle: 0 };
    if (!freqs[nextState]) return;
    osc.frequency.value = freqs[nextState];
    osc.type = nextState === 'error' ? 'sawtooth' : 'sine';
    gain.gain.setValueAtTime(0.0001, ctx.currentTime);
    gain.gain.linearRampToValueAtTime(0.06, ctx.currentTime + 0.01);
    gain.gain.exponentialRampToValueAtTime(0.0001, ctx.currentTime + 0.3);
    osc.connect(gain).connect(ctx.destination);
    osc.start();
    osc.stop(ctx.currentTime + 0.3);
  } catch (err) {
    console.debug(err);
  }
}

async function announceCompletion(name) {
  if (!name) return;
  try {
    const response = await fetch(API_BASE + '/api/tts', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      cache: 'no-store',
      body: JSON.stringify({ text: `${name}, order completed` }),
    });
    if (!response.ok) {
      let detail = response.statusText;
      try {
        const payload = await response.json();
        detail = payload.error || detail;
      } catch (err) {
        console.debug(err);
      }
      throw new Error(detail || `tts failed: ${response.status}`);
    }

    const audioBlob = await response.blob();
    const audioUrl = URL.createObjectURL(audioBlob);

    if (activeAnnouncementAudio) {
      try {
        activeAnnouncementAudio.pause();
        if (activeAnnouncementAudio.dataset.objectUrl) {
          URL.revokeObjectURL(activeAnnouncementAudio.dataset.objectUrl);
        }
      } catch (err) {
        console.debug(err);
      }
    }

    const audio = new Audio(audioUrl);
    audio.dataset.objectUrl = audioUrl;
    audio.addEventListener('ended', () => URL.revokeObjectURL(audioUrl), { once: true });
    audio.addEventListener('error', () => URL.revokeObjectURL(audioUrl), { once: true });
    activeAnnouncementAudio = audio;
    await audio.play();
  } catch (err) {
    console.warn('Unable to play ElevenLabs announcement.', err);
  }
}

function applyState(nextState, force = false) {
  state = nextState;
  const stateChanged = force || lastAppliedState !== nextState;
  lastAppliedState = nextState;

  const hero = document.getElementById('hero');
  STEPS.forEach((step) => hero.classList.remove('state-' + step));
  hero.classList.remove('state-error');
  hero.classList.add('state-' + nextState);

  const label = LABELS[nextState] || LABELS.idle;
  const isCompletedMine = nextState === 'arrived' && currentOrder && currentOrder.isMe;
  document.getElementById('headline').innerHTML = isCompletedMine
    ? `${escapeHtml(currentOrder.name)} <em>order completed</em>.`
    : (label.headline || tw.headline);
  document.getElementById('sub').innerHTML = isCompletedMine
    ? ''
    : (label.sub || 'Press the button and Cove will prepare your matcha.');
  document.getElementById('statustag').textContent = label.tag;
  document.getElementById('sysstatus').textContent = label.status;
  const activityText = document.getElementById('activityText');
  if (activityText) {
    activityText.textContent = isCompletedMine
      ? `${currentOrder.name} order completed`
      : motionActivityText(label.activity || 'Ready for your order');
  }

  const eyebrow = document.getElementById('eyebrow-label');
  eyebrow.textContent = currentOrder && nextState !== 'idle'
    ? `order #${currentOrder.id} - ${currentOrder.name}`
    : 'cove · r-01 · matcha';

  const sysdot = document.getElementById('sysdot');
  sysdot.classList.remove('idle', 'err');
  sysdot.classList.add(label.dot);

  const timelineSteps = document.querySelectorAll('#timeline .step');
  const idx = STEPS.indexOf(nextState);
  timelineSteps.forEach((el, index) => {
    el.classList.remove('done', 'active', 'err');
    if (nextState === 'error') {
      if (index < 2) el.classList.add('done');
      if (index === 2) el.classList.add('err');
      return;
    }
    if (index < idx) el.classList.add('done');
    else if (index === idx) el.classList.add('active');
  });

  const fig = document.getElementById('heroFigure');
  fig.classList.remove('float', 'working');
  if (nextState === 'idle' || nextState === 'arrived') fig.classList.add('float');
  if (nextState === 'received' || nextState === 'pouring') fig.classList.add('working');

  updateOrderButtonAvailability();
  setFillProgress(nextState === 'pouring' ? remotePhaseProgress : 0);

  const overlay = document.getElementById('overlay');
  overlay.classList.remove('show');

  if (isCompletedMine && stateChanged && currentOrder.id !== lastSpokenOrderId) {
    lastSpokenOrderId = currentOrder.id;
    void announceCompletion(currentOrder.name);
  }
  if (nextState !== 'arrived') {
    lastSpokenOrderId = null;
  }

  renderQueue();
  renderSlots();
  if (stateChanged) beep(nextState);
}

function handleOffline(reason) {
  apiOnline = false;
  updateOrderButtonAvailability();
  document.getElementById('statustag').textContent = 'backend offline';
  document.getElementById('sysstatus').textContent = 'System - Bridge offline';
  document.getElementById('sub').innerHTML = 'Waiting for the kiosk bridge. Start the kiosk server on the Pi and reload this page.';
  console.warn(reason);
}

function applySnapshot(snapshot) {
  apiOnline = true;
  lastSnapshot = snapshot;
  queue = Array.isArray(snapshot.queue) ? snapshot.queue : [];
  currentOrder = snapshot.current_order || null;
  currentMotion = snapshot.motion || {};
  servedToday = Number(snapshot.served_today || 0);
  remotePhaseProgress = Number(currentMotion.progress ?? snapshot.phase_progress ?? 0);

  document.getElementById('r-cups').textContent = String(snapshot.cups_remaining ?? 0);
  document.getElementById('r-served').textContent = String(servedToday);
  document.getElementById('r-uptime').textContent = snapshot.uptime_human || '0d 00:00';
  document.getElementById('op-mode').textContent = snapshot.queue_mode || 'auto-run';
  document.getElementById('op-qcount').textContent = String((snapshot.active_count || 0) + (snapshot.queued_count || 0));

  const operatorLines = document.querySelectorAll('.oppanel .kv .v');
  if (operatorLines.length >= 6) {
    operatorLines[0].textContent = snapshot.uptime_human || operatorLines[0].textContent;
    operatorLines[1].textContent = snapshot.connected
      ? (currentMotion.running ? 'script running' : 'script ready')
      : 'waiting';
    operatorLines[2].textContent = `${snapshot.cups_remaining ?? 0} / ${snapshot.cups_capacity ?? 0}`;
    operatorLines[3].textContent = snapshot.last_calibration || operatorLines[3].textContent;
    operatorLines[4].textContent = `${snapshot.served_today ?? 0} - avg ${formatSeconds(snapshot.average_cycle_seconds || 43)}`;
    operatorLines[5].textContent = snapshot.fault_message ? '1' : '0';
  }

  applyState(snapshot.state || 'idle');

  if (currentMotion.step || currentMotion.message) {
    const activityText = document.getElementById('activityText');
    if (activityText && !(snapshot.state === 'arrived' && currentOrder && currentOrder.isMe)) {
      activityText.textContent = motionActivityText(activityText.textContent);
    }
    document.getElementById('sysstatus').textContent = currentMotion.running
      ? `Arm script - ${currentMotion.step || currentMotion.phase || 'running'}`
      : `Arm script - ${currentMotion.step || 'ready'}`;
  }

  if (!snapshot.connected && snapshot.state === 'idle') {
    document.getElementById('statustag').textContent = 'waiting for script';
    document.getElementById('sysstatus').textContent = 'System - Waiting for arm script';
  }
  if (snapshot.fault_message) {
    document.getElementById('sub').innerHTML = escapeHtml(snapshot.fault_message);
  }
}

async function apiRequest(path, options = {}) {
  const response = await fetch(API_BASE + path, {
    headers: { 'Content-Type': 'application/json' },
    cache: 'no-store',
    ...options,
  });
  if (!response.ok) {
    let detail = response.statusText;
    try {
      const body = await response.json();
      detail = body.error || detail;
    } catch (err) {
      console.debug(err);
    }
    throw new Error(detail || `request failed: ${response.status}`);
  }
  const contentType = response.headers.get('content-type') || '';
  if (!contentType.includes('application/json')) return null;
  return response.json();
}

async function refreshSnapshot() {
  try {
    const snapshot = await apiRequest('/api/state', { method: 'GET' });
    applySnapshot(snapshot);
  } catch (err) {
    handleOffline(err);
  }
}

async function createOrder(name, isMe) {
  const payload = await apiRequest('/api/orders', {
    method: 'POST',
    body: JSON.stringify({ name, isMe }),
  });
  await refreshSnapshot();
  return payload;
}

function openNamePrompt() {
  if (!apiOnline || state !== 'idle') return;
  const overlay = document.getElementById('nameOverlay');
  overlay.classList.add('show');
  nameInput.value = '';
  nameConfirm.disabled = true;
  setTimeout(() => nameInput.focus(), 50);
}

function closeNamePrompt() {
  document.getElementById('nameOverlay').classList.remove('show');
}

const nameInput = document.getElementById('nameInput');
const nameConfirm = document.getElementById('nameConfirm');
nameInput.addEventListener('input', (e) => {
  nameConfirm.disabled = e.target.value.trim().length === 0 || submittingOrder;
});
document.getElementById('nameCancel').addEventListener('click', closeNamePrompt);
document.getElementById('nameConfirm').addEventListener('click', async () => {
  const name = nameInput.value.trim();
  if (!name) return;
  submittingOrder = true;
  updateOrderButtonAvailability();
  nameConfirm.disabled = true;
  try {
    await createOrder(name, true);
    closeNamePrompt();
  } catch (err) {
    alert(err.message || 'Unable to submit order.');
  } finally {
    submittingOrder = false;
    nameConfirm.disabled = nameInput.value.trim().length === 0;
    updateOrderButtonAvailability();
  }
});

const KEYROWS = [...'QWERTYUIOP', ...'ASDFGHJKL', ...'ZXCVBNM'];
const keypad = document.getElementById('keypad');
KEYROWS.forEach((key) => {
  const el = document.createElement('div');
  el.className = 'key';
  el.textContent = key;
  el.addEventListener('click', () => {
    nameInput.value += key;
    nameInput.dispatchEvent(new Event('input'));
  });
  keypad.appendChild(el);
});
const spaceKey = document.createElement('div');
spaceKey.className = 'key space';
spaceKey.textContent = 'space';
spaceKey.addEventListener('click', () => {
  nameInput.value += ' ';
  nameInput.dispatchEvent(new Event('input'));
});
keypad.appendChild(spaceKey);
const backKey = document.createElement('div');
backKey.className = 'key back';
backKey.textContent = 'back';
backKey.addEventListener('click', () => {
  nameInput.value = nameInput.value.slice(0, -1);
  nameInput.dispatchEvent(new Event('input'));
});
keypad.appendChild(backKey);

document.getElementById('orderBtn').addEventListener('click', openNamePrompt);
document.getElementById('closeReceipt').addEventListener('click', () => {
  document.getElementById('overlay').classList.remove('show');
});

document.getElementById('soundBtn').addEventListener('click', () => {
  soundOn = !soundOn;
  const soundBtn = document.getElementById('soundBtn');
  soundBtn.classList.toggle('on', soundOn);
  soundBtn.innerHTML = soundBtn.innerHTML.replace(/sound · (on|off)/, 'sound · ' + (soundOn ? 'on' : 'off'));
});

const op = document.getElementById('oppanel');
document.getElementById('opBtn').addEventListener('click', () => op.classList.toggle('show'));
document.getElementById('errorBtn').addEventListener('click', async () => {
  try {
    await apiRequest('/api/operator/error', { method: 'POST', body: JSON.stringify({}) });
    await refreshSnapshot();
  } catch (err) {
    alert(err.message || 'Unable to trigger fault.');
  }
});
document.getElementById('resetBtn').addEventListener('click', async () => {
  try {
    await apiRequest('/api/operator/reset', { method: 'POST', body: JSON.stringify({ clearQueue: true }) });
    op.classList.remove('show');
    await refreshSnapshot();
  } catch (err) {
    alert(err.message || 'Unable to reset queue.');
  }
});
document.getElementById('addOrderBtn').addEventListener('click', async () => {
  const name = TESTNAMES[Math.floor(Math.random() * TESTNAMES.length)];
  try {
    await createOrder(name, false);
  } catch (err) {
    alert(err.message || 'Unable to add test order.');
  }
});
document.getElementById('addManyBtn').addEventListener('click', async () => {
  try {
    for (let i = 0; i < 3; i += 1) {
      const name = TESTNAMES[Math.floor(Math.random() * TESTNAMES.length)];
      await createOrder(name, false);
    }
  } catch (err) {
    alert(err.message || 'Unable to add test orders.');
  }
});

function applyTweaks(nextTweaks, hueChanged) {
  if (hueChanged) {
    document.documentElement.style.setProperty('--lime', `oklch(0.92 0.14 ${nextTweaks.accentHue})`);
    document.documentElement.style.setProperty('--green', `oklch(0.80 0.14 ${nextTweaks.accentHue})`);
  }
  if (state === 'idle') {
    document.getElementById('headline').innerHTML = nextTweaks.headline;
  }
  document.querySelector('.reticle').style.display = nextTweaks.showDotmatrix ? '' : 'none';
}
applyTweaks(tw, false);

document.getElementById('tw-hue').addEventListener('input', (e) => {
  tw.accentHue = Number(e.target.value);
  applyTweaks(tw, true);
  persist({ accentHue: tw.accentHue });
});
document.getElementById('tw-headline').addEventListener('change', (e) => {
  tw.headline = e.target.value;
  applyTweaks(tw, false);
  persist({ headline: tw.headline });
});
document.getElementById('tw-speed').addEventListener('input', (e) => {
  tw.speed = Number(e.target.value);
  persist({ speed: tw.speed });
});
document.getElementById('tw-bg').addEventListener('click', () => {
  tw.showDotmatrix = !tw.showDotmatrix;
  applyTweaks(tw, false);
  persist({ showDotmatrix: tw.showDotmatrix });
});

window.addEventListener('message', (ev) => {
  if (!ev.data) return;
  if (ev.data.type === '__activate_edit_mode') document.getElementById('tweaks').classList.add('show');
  if (ev.data.type === '__deactivate_edit_mode') document.getElementById('tweaks').classList.remove('show');
});
window.parent.postMessage({ type: '__edit_mode_available' }, '*');
function persist(edits) { window.parent.postMessage({ type: '__edit_mode_set_keys', edits }, '*'); }

simplifyChrome();
applyState('idle', true);
renderQueue();
renderSlots();
refreshSnapshot();
setInterval(refreshSnapshot, 1000);"""


def update_bundle() -> None:
    text = SOURCE_HTML.read_text()
    match = re.search(
        r"(<script type=\"__bundler/template\">\n)(.*?)(\n  </script>)",
        text,
        re.S,
    )
    if not match:
        raise RuntimeError("template payload not found")

    template = json.loads(match.group(2))
    start = template.index(START_MARKER)
    end = template.rindex(END_MARKER)
    updated_template = template[:start] + NEW_SCRIPT + template[end:]
    # Escape closing script tags inside the JSON string so the HTML parser does
    # not terminate the bundle payload early when the page loads.
    encoded_template = json.dumps(updated_template).replace("</script>", "<\\/script>")
    updated_text = text[: match.start(2)] + encoded_template + text[match.end(2) :]
    SOURCE_HTML.write_text(updated_text)
    PACKAGE_HTML.write_text(updated_text)
    if INSTALL_HTML.exists():
        INSTALL_HTML.write_text(updated_text)


if __name__ == "__main__":
    update_bundle()
