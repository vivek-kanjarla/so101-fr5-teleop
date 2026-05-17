#!/usr/bin/env python3
"""
web_tune.py — browser-based interactive parameter tuner.
Run:  python ~/teleop/web_tune.py
Open: http://localhost:7860
"""

import json, os, re, socket, threading
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

HERE           = os.path.dirname(os.path.abspath(__file__))
PRESETS_FILE   = os.path.join(HERE, "presets.json")
CONFIG_PY      = os.path.join(HERE, "config.py")
SINGULARITY_PY = os.path.join(HERE, "singularity.py")
PORT           = 7860

@dataclass
class Param:
    key: str; default: Any; dtype: type; step: float; big: float
    lo: float|None = None; hi: float|None = None; fmt: str = "{:.3f}"
    value: Any = field(init=False)
    def __post_init__(self): self.value = self.default
    def to_json(self): return self.value
    def from_json(self, v): self.value = self.dtype(v)
    def set(self, v):
        v = float(v)
        if self.lo is not None: v = max(float(self.lo), v)
        if self.hi is not None: v = min(float(self.hi), v)
        self.value = int(round(v)) if self.dtype is int else self.dtype(v)

@dataclass
class Section:
    label: str

def build_params():
    P, S = Param, Section
    return [
        S("CONTROL LOOP"),
        P("LOOP_HZ",125,int,1,25,60,1000,"{}"),
        S("JOINT MAPPING — Scale  (+ same direction  − reverse)"),
        P("JOINT_SCALE[shoulder_pan]",-1.0,float,0.05,0.5,-1.0,1.0),
        P("JOINT_SCALE[shoulder_lift]",1.0,float,0.05,0.5,-1.0,1.0),
        P("JOINT_SCALE[elbow_flex]",1.0,float,0.05,0.5,-1.0,1.0),
        P("JOINT_SCALE[wrist_flex]",1.0,float,0.05,0.5,-1.0,1.0),
        P("JOINT_SCALE[wrist_roll]",1.0,float,0.05,0.5,-1.0,1.0),
        S("JOINT MAPPING — Amplification  (1.0 = 1:1)"),
        P("JOINT_AMP[shoulder_pan]",1.5,float,0.1,0.5,0.1,10.0),
        P("JOINT_AMP[shoulder_lift]",2.0,float,0.1,0.5,0.1,10.0),
        P("JOINT_AMP[elbow_flex]",2.0,float,0.1,0.5,0.1,10.0),
        P("JOINT_AMP[wrist_flex]",3.0,float,0.1,0.5,0.1,10.0),
        P("JOINT_AMP[wrist_roll]",1.5,float,0.1,0.5,0.1,10.0),
        P("FR5_J4_FROZEN_DEG",0.0,float,1.0,10.0,-170.0,170.0,"{:.1f}"),
        S("SAFETY — Rate Limits  (deg/cycle × LOOP_HZ = deg/s)"),
        P("MAX_DELTA_DEG_PER_CYCLE",0.08,float,0.005,0.02,0.001,2.0,"{:.4f}"),
        P("MAX_DELTA_PER_JOINT[J1]",0.16,float,0.005,0.02,0.001,2.0,"{:.4f}"),
        P("MAX_DELTA_PER_JOINT[J2]",0.12,float,0.005,0.02,0.001,2.0,"{:.4f}"),
        P("MAX_DELTA_PER_JOINT[J3]",0.12,float,0.005,0.02,0.001,2.0,"{:.4f}"),
        P("MAX_DELTA_PER_JOINT[J4]",0.30,float,0.005,0.05,0.001,2.0,"{:.4f}"),
        P("MAX_DELTA_PER_JOINT[J5]",0.08,float,0.005,0.02,0.001,2.0,"{:.4f}"),
        P("MAX_DELTA_PER_JOINT[J6]",0.20,float,0.005,0.02,0.001,2.0,"{:.4f}"),
        S("SAFETY — ServoJ Parameters"),
        P("FR5_SERVO_VEL",15,int,1,5,1,100,"{}"),
        P("FR5_FILTER_T",0.04,float,0.01,0.05,0.0,1.0),
        S("JOINT LIMITS  (deg)"),
        P("FR5_JOINT_LIMITS[J1].lo",-170.0,float,1.0,10.0,-360.0,0.0,"{:.1f}"),
        P("FR5_JOINT_LIMITS[J1].hi",170.0,float,1.0,10.0,0.0,360.0,"{:.1f}"),
        P("FR5_JOINT_LIMITS[J2].lo",-260.0,float,1.0,10.0,-360.0,0.0,"{:.1f}"),
        P("FR5_JOINT_LIMITS[J2].hi",80.0,float,1.0,10.0,0.0,360.0,"{:.1f}"),
        P("FR5_JOINT_LIMITS[J3].lo",-155.0,float,1.0,10.0,-360.0,0.0,"{:.1f}"),
        P("FR5_JOINT_LIMITS[J3].hi",155.0,float,1.0,10.0,0.0,360.0,"{:.1f}"),
        P("FR5_JOINT_LIMITS[J4].lo",-260.0,float,1.0,10.0,-360.0,0.0,"{:.1f}"),
        P("FR5_JOINT_LIMITS[J4].hi",80.0,float,1.0,10.0,0.0,360.0,"{:.1f}"),
        P("FR5_JOINT_LIMITS[J5].lo",-170.0,float,1.0,10.0,-360.0,0.0,"{:.1f}"),
        P("FR5_JOINT_LIMITS[J5].hi",170.0,float,1.0,10.0,0.0,360.0,"{:.1f}"),
        P("FR5_JOINT_LIMITS[J6].lo",-170.0,float,1.0,10.0,-360.0,0.0,"{:.1f}"),
        P("FR5_JOINT_LIMITS[J6].hi",170.0,float,1.0,10.0,0.0,360.0,"{:.1f}"),
        S("GRIPPER  (DH AG-160-95)"),
        P("GRIPPER_OPEN_PCT",100,int,5,20,0,100,"{}"),
        P("GRIPPER_CLOSE_PCT",0,int,5,20,0,100,"{}"),
        P("GRIPPER_VEL_PCT",50,int,5,20,0,100,"{}"),
        P("GRIPPER_FORCE_PCT",50,int,5,20,0,100,"{}"),
        P("GRIPPER_MAXTIME_MS",5000,int,100,500,100,30000,"{}"),
        S("GRIPPER — SO-101 Normalised Thresholds  (0–1)"),
        P("SO101_GRIPPER_OPEN_THRESHOLD",0.65,float,0.01,0.05,0.01,0.99),
        P("SO101_GRIPPER_CLOSE_THRESHOLD",0.35,float,0.01,0.05,0.01,0.99),
        S("SINGULARITY — Detection Thresholds  (deg)"),
        P("WRIST_WARN_DEG",15.0,float,0.5,5.0,0.0,90.0,"{:.1f}"),
        P("WRIST_DANGER_DEG",5.0,float,0.5,5.0,0.0,90.0,"{:.1f}"),
        P("ELBOW_WARN_DEG",8.0,float,0.5,5.0,0.0,90.0,"{:.1f}"),
        P("ELBOW_DANGER_DEG",3.0,float,0.5,5.0,0.0,90.0,"{:.1f}"),
    ]

def params_to_dict(items):
    return {it.key: it.to_json() for it in items if isinstance(it, Param)}

def dict_to_params(items, d):
    for it in items:
        if isinstance(it, Param) and it.key in d:
            it.from_json(d[it.key])

def load_presets():
    if os.path.exists(PRESETS_FILE):
        with open(PRESETS_FILE) as f: return json.load(f)
    return {}

def save_presets(p):
    with open(PRESETS_FILE, "w") as f: json.dump(p, f, indent=2)

def write_config_py(p):
    # Patch-in-place using regex — preserves every field we don't expose
    # (CAMERA_*, INSTRUCTION_FILE, LOG_STATE_DOWNSAMPLE, etc.).
    with open(CONFIG_PY) as f:
        src = f.read()

    def s(text, name, val):
        return re.sub(rf'^({re.escape(name)}\s*=\s*)[^\n]+',
                      lambda m: f"{m.group(1)}{val}", text, flags=re.MULTILINE)

    lo = lambda j: int(p[f"FR5_JOINT_LIMITS[{j}].lo"])
    hi = lambda j: int(p[f"FR5_JOINT_LIMITS[{j}].hi"])

    src = s(src, 'LOOP_HZ',               p["LOOP_HZ"])
    src = s(src, 'FR5_J4_FROZEN_DEG',     f'{p["FR5_J4_FROZEN_DEG"]:.1f}')
    src = s(src, 'JOINT_SCALE',
            f'[{p["JOINT_SCALE[shoulder_pan]"]:.3f}, {p["JOINT_SCALE[shoulder_lift]"]:.3f}, '
            f'{p["JOINT_SCALE[elbow_flex]"]:.3f}, {p["JOINT_SCALE[wrist_flex]"]:.3f}, {p["JOINT_SCALE[wrist_roll]"]:.3f}]')
    src = s(src, 'JOINT_AMP',
            f'[{p["JOINT_AMP[shoulder_pan]"]:.2f}, {p["JOINT_AMP[shoulder_lift]"]:.2f}, '
            f'{p["JOINT_AMP[elbow_flex]"]:.2f}, {p["JOINT_AMP[wrist_flex]"]:.2f}, {p["JOINT_AMP[wrist_roll]"]:.2f}]')
    src = s(src, 'MAX_DELTA_DEG_PER_CYCLE', f'{p["MAX_DELTA_DEG_PER_CYCLE"]:.4f}')
    src = s(src, 'MAX_DELTA_PER_JOINT',
            f'[{p["MAX_DELTA_PER_JOINT[J1]"]:.4f}, {p["MAX_DELTA_PER_JOINT[J2]"]:.4f}, '
            f'{p["MAX_DELTA_PER_JOINT[J3]"]:.4f}, {p["MAX_DELTA_PER_JOINT[J4]"]:.4f}, '
            f'{p["MAX_DELTA_PER_JOINT[J5]"]:.4f}, {p["MAX_DELTA_PER_JOINT[J6]"]:.4f}]')
    src = s(src, 'FR5_SERVO_VEL',   p["FR5_SERVO_VEL"])
    src = s(src, 'FR5_FILTER_T',    f'{p["FR5_FILTER_T"]:.2f}')

    limits_block = (
        'FR5_JOINT_LIMITS = [\n'
        f'    ({lo("J1")}, {hi("J1")}),   # J1  hardware ±175°\n'
        f'    ({lo("J2")}, {hi("J2")}),   # J2  hardware (-265, 85)\n'
        f'    ({lo("J3")}, {hi("J3")}),   # J3  hardware ±160°\n'
        f'    ({lo("J4")}, {hi("J4")}),   # J4  hardware (-265, 85) — frozen in mapper\n'
        f'    ({lo("J5")}, {hi("J5")}),   # J5  hardware ±175°\n'
        f'    ({lo("J6")}, {hi("J6")}),   # J6  hardware ±175°\n'
        ']'
    )
    src = re.sub(r'FR5_JOINT_LIMITS\s*=\s*\[.*?\]', limits_block, src, flags=re.DOTALL)

    src = s(src, 'GRIPPER_OPEN_PCT',              p["GRIPPER_OPEN_PCT"])
    src = s(src, 'GRIPPER_CLOSE_PCT',             p["GRIPPER_CLOSE_PCT"])
    src = s(src, 'GRIPPER_VEL_PCT',               p["GRIPPER_VEL_PCT"])
    src = s(src, 'GRIPPER_FORCE_PCT',             p["GRIPPER_FORCE_PCT"])
    src = s(src, 'GRIPPER_MAXTIME_MS',            p["GRIPPER_MAXTIME_MS"])
    src = s(src, 'SO101_GRIPPER_OPEN_THRESHOLD',  f'{p["SO101_GRIPPER_OPEN_THRESHOLD"]:.2f}')
    src = s(src, 'SO101_GRIPPER_CLOSE_THRESHOLD', f'{p["SO101_GRIPPER_CLOSE_THRESHOLD"]:.2f}')

    with open(CONFIG_PY, "w") as f:
        f.write(src)

def write_singularity_py(p):
    with open(SINGULARITY_PY) as f: src = f.read()
    for name, val in [("WRIST_WARN_DEG",p["WRIST_WARN_DEG"]),("WRIST_DANGER_DEG",p["WRIST_DANGER_DEG"]),
                      ("ELBOW_WARN_DEG",p["ELBOW_WARN_DEG"]),("ELBOW_DANGER_DEG",p["ELBOW_DANGER_DEG"])]:
        src = re.sub(rf'^({re.escape(name)}\s*=\s*)[\d.]+',
                     lambda m,v=val: m.group(1)+f"{v:.1f}", src, flags=re.MULTILINE)
    with open(SINGULARITY_PY, "w") as f: f.write(src)

def _seed_from_config() -> dict:
    """Read live config.py + singularity.py values so 'main' always reflects
    the actual running configuration, not stale hardcoded defaults."""
    import importlib.util
    out = {}
    def _load(path):
        spec = importlib.util.spec_from_file_location("_tmp", path)
        mod  = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    try:
        c = _load(CONFIG_PY)
        out["LOOP_HZ"]               = c.LOOP_HZ
        out["FR5_J4_FROZEN_DEG"]     = c.FR5_J4_FROZEN_DEG
        out["JOINT_SCALE[shoulder_pan]"]  = c.JOINT_SCALE[0]
        out["JOINT_SCALE[shoulder_lift]"] = c.JOINT_SCALE[1]
        out["JOINT_SCALE[elbow_flex]"]    = c.JOINT_SCALE[2]
        out["JOINT_SCALE[wrist_flex]"]    = c.JOINT_SCALE[3]
        out["JOINT_SCALE[wrist_roll]"]    = c.JOINT_SCALE[4]
        out["JOINT_AMP[shoulder_pan]"]    = c.JOINT_AMP[0]
        out["JOINT_AMP[shoulder_lift]"]   = c.JOINT_AMP[1]
        out["JOINT_AMP[elbow_flex]"]      = c.JOINT_AMP[2]
        out["JOINT_AMP[wrist_flex]"]      = c.JOINT_AMP[3]
        out["JOINT_AMP[wrist_roll]"]      = c.JOINT_AMP[4]
        out["MAX_DELTA_DEG_PER_CYCLE"]    = c.MAX_DELTA_DEG_PER_CYCLE
        for i, j in enumerate(["J1","J2","J3","J4","J5","J6"]):
            out[f"MAX_DELTA_PER_JOINT[{j}]"] = c.MAX_DELTA_PER_JOINT[i]
        out["FR5_SERVO_VEL"]  = c.FR5_SERVO_VEL
        out["FR5_FILTER_T"]   = c.FR5_FILTER_T
        for i, j in enumerate(["J1","J2","J3","J4","J5","J6"]):
            out[f"FR5_JOINT_LIMITS[{j}].lo"] = float(c.FR5_JOINT_LIMITS[i][0])
            out[f"FR5_JOINT_LIMITS[{j}].hi"] = float(c.FR5_JOINT_LIMITS[i][1])
        out["GRIPPER_OPEN_PCT"]              = c.GRIPPER_OPEN_PCT
        out["GRIPPER_CLOSE_PCT"]             = c.GRIPPER_CLOSE_PCT
        out["GRIPPER_VEL_PCT"]               = c.GRIPPER_VEL_PCT
        out["GRIPPER_FORCE_PCT"]             = c.GRIPPER_FORCE_PCT
        out["GRIPPER_MAXTIME_MS"]            = c.GRIPPER_MAXTIME_MS
        out["SO101_GRIPPER_OPEN_THRESHOLD"]  = c.SO101_GRIPPER_OPEN_THRESHOLD
        out["SO101_GRIPPER_CLOSE_THRESHOLD"] = c.SO101_GRIPPER_CLOSE_THRESHOLD
    except Exception as e:
        print(f"[tune] Could not read config.py ({e}) — using defaults")
    try:
        s = _load(SINGULARITY_PY)
        out["WRIST_WARN_DEG"]   = s.WRIST_WARN_DEG
        out["WRIST_DANGER_DEG"] = s.WRIST_DANGER_DEG
        out["ELBOW_WARN_DEG"]   = s.ELBOW_WARN_DEG
        out["ELBOW_DANGER_DEG"] = s.ELBOW_DANGER_DEG
    except Exception:
        pass
    return out


_lock = threading.Lock()
_items = build_params()
_params = [it for it in _items if isinstance(it, Param)]
_presets = load_presets()
_cur_preset = "main"

# Always sync 'main' from the live config files so the tuner reflects
# whatever values teleop.py is actually using right now.
_live = _seed_from_config()
if "main" not in _presets:
    _presets["main"] = params_to_dict(_params)
_presets["main"].update(_live)
save_presets(_presets)
dict_to_params(_items, _presets["main"])

def _get_state():
    names = sorted(_presets.keys(), key=lambda k: (k != "main", k))
    return {
        "items": [{"type":"section","label":it.label} if isinstance(it,Section)
                  else {"type":"param","key":it.key} for it in _items],
        "params": [{"key":p.key,"value":p.value,"default":p.default,
                    "lo":p.lo,"hi":p.hi,"step":p.step,"big":p.big,
                    "dtype":p.dtype.__name__,"fmt":p.fmt} for p in _params],
        "presets": names, "current_preset": _cur_preset,
        "main_values": _presets.get("main", {}),
    }

# ── HTML ──────────────────────────────────────────────────────────────────────

HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>FR5 Teleop Tuner</title>
<style>
:root{
  --bg:#0d1117;--surf:#161b22;--card:#21262d;--acc:#58a6ff;--glow:rgba(88,166,255,.15);
  --warn:#f0883e;--danger:#f85149;--ok:#3fb950;--text:#c9d1d9;--dim:#8b949e;
  --border:#30363d;--changed:rgba(240,136,62,.1);--sec-bg:#0d1117;
}
*{box-sizing:border-box;margin:0;padding:0}
html,body{height:100%;overflow:hidden}
body{background:var(--bg);color:var(--text);font-family:ui-monospace,'Cascadia Code',monospace;font-size:13px;display:flex;flex-direction:column}

/* ── header ── */
header{background:var(--surf);border-bottom:1px solid var(--border);padding:9px 16px;display:flex;align-items:center;gap:10px;flex-shrink:0;flex-wrap:wrap}
h1{font-size:14px;color:var(--acc);letter-spacing:.05em;white-space:nowrap}
.pill{display:flex;align-items:center;gap:6px;background:var(--card);border:1px solid var(--border);border-radius:7px;padding:4px 10px}
.pill label{color:var(--dim);font-size:11px}
select{background:transparent;color:var(--text);border:none;font-family:inherit;font-size:13px;cursor:pointer;outline:none}
select option{background:var(--card)}
button{background:var(--card);color:var(--text);border:1px solid var(--border);padding:5px 11px;border-radius:6px;font-family:inherit;font-size:12px;cursor:pointer;transition:all .12s;white-space:nowrap}
button:hover{border-color:var(--acc);color:var(--acc)}
button.primary{background:var(--acc);color:#0d1117;border-color:var(--acc);font-weight:700}
button.primary:hover{filter:brightness(.88);color:#0d1117}
button.red:hover{background:var(--danger);border-color:var(--danger);color:#fff}
#status{font-size:12px;color:var(--dim);margin-left:auto;transition:color .25s;white-space:nowrap}
#status.ok{color:var(--ok)}#status.err{color:var(--danger)}#status.busy{color:var(--warn)}

/* ── layout ── */
.layout{flex:1;display:grid;grid-template-columns:1fr 340px;overflow:hidden;min-height:0}

/* ── param list (left) ── */
.param-list{overflow-y:auto;overflow-x:hidden}

.sec{background:var(--sec-bg);border-bottom:1px solid var(--border);padding:8px 16px 6px;color:var(--acc);font-size:10.5px;font-weight:700;letter-spacing:.13em;text-transform:uppercase;position:sticky;top:0;z-index:10}

/* param row: key+desc | lo | slider | hi | input | effect */
.row{display:grid;grid-template-columns:230px 38px 1fr 38px 88px 140px;align-items:center;gap:7px;padding:5px 16px;border-bottom:1px solid var(--border);cursor:pointer;transition:background .1s}
.row:hover{background:var(--card)}
.row.selected{background:rgba(88,166,255,.07);border-left:2px solid var(--acc)}
.row.changed{background:var(--changed)}
.row.selected.changed{background:rgba(88,166,255,.1)}

.key-col{min-width:0}
.key-name{font-size:12px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.key-name .sub{color:var(--dim)}
.key-short{font-size:10px;color:var(--dim);white-space:nowrap;overflow:hidden;text-overflow:ellipsis;margin-top:1px}

.bound{color:var(--dim);font-size:10px;text-align:right;user-select:none}
.bound.hi{text-align:left}

/* slider */
input[type=range]{-webkit-appearance:none;appearance:none;width:100%;height:20px;background:transparent;cursor:pointer;outline:none}
input[type=range]::-webkit-slider-runnable-track{height:3px;border-radius:2px;background:linear-gradient(to right,var(--acc) var(--pct,0%),var(--border) var(--pct,0%))}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:12px;height:12px;margin-top:-4.5px;border-radius:50%;background:var(--acc);transition:box-shadow .12s}
input[type=range]:hover::-webkit-slider-thumb{box-shadow:0 0 0 5px var(--glow)}

/* number input */
.val{background:var(--card);color:var(--text);border:1px solid var(--border);padding:3px 7px;border-radius:5px;font-family:inherit;font-size:12px;width:88px;text-align:right;transition:border-color .12s,color .12s}
.val:focus{outline:none;border-color:var(--acc)}
.val.changed{border-color:var(--warn);color:var(--warn)}

/* effect badge */
.effect{font-size:10px;color:var(--dim);white-space:nowrap;overflow:hidden;text-overflow:ellipsis;text-align:left}
.effect.pos{color:#7ee787}.effect.neg{color:var(--warn)}.effect.danger{color:var(--danger)}

/* ── focus panel (right) ── */
.focus{background:var(--surf);border-left:1px solid var(--border);overflow-y:auto;display:flex;flex-direction:column}
.focus-empty{display:flex;flex-direction:column;align-items:center;justify-content:center;flex:1;color:var(--dim);font-size:12px;text-align:center;padding:32px;gap:8px}
.focus-empty .icon{font-size:40px;opacity:.3}
.focus-content{padding:18px 16px;display:flex;flex-direction:column;gap:14px}

.focus-name{font-size:14px;font-weight:700;color:var(--acc);line-height:1.3}
.focus-short{font-size:11px;color:var(--dim)}
.focus-desc{font-size:12px;color:var(--text);line-height:1.7;background:var(--card);border-radius:8px;padding:12px 14px;border-left:3px solid var(--acc)}

.focus-live{background:var(--bg);border:1px solid var(--border);border-radius:8px;padding:12px 14px}
.focus-live-label{font-size:10px;color:var(--dim);letter-spacing:.1em;text-transform:uppercase;margin-bottom:6px}
.focus-live-val{font-size:15px;font-weight:700;color:var(--ok);font-variant-numeric:tabular-nums}

.focus-meta{display:flex;gap:10px;flex-wrap:wrap}
.focus-chip{background:var(--card);border:1px solid var(--border);border-radius:5px;padding:3px 9px;font-size:11px;color:var(--dim)}
.focus-chip span{color:var(--text)}

/* ── mini visualizations ── */
.viz{background:var(--bg);border:1px solid var(--border);border-radius:8px;padding:12px 14px}
.viz-title{font-size:10px;color:var(--dim);letter-spacing:.1em;text-transform:uppercase;margin-bottom:10px}

/* gripper threshold bar */
.grip-bar-wrap{position:relative;margin:4px 0 20px}
.grip-bar{height:22px;border-radius:4px;display:flex;overflow:hidden}
.grip-close{background:rgba(248,81,73,.5);display:flex;align-items:center;justify-content:center;font-size:9px;color:#fff}
.grip-dead{background:var(--border);display:flex;align-items:center;justify-content:center;font-size:9px;color:var(--dim)}
.grip-open{background:rgba(63,185,80,.5);display:flex;align-items:center;justify-content:center;font-size:9px;color:#fff}
.grip-labels{display:flex;justify-content:space-between;margin-top:4px;font-size:10px;color:var(--dim)}

/* rate bars */
.rate-bars{display:flex;flex-direction:column;gap:5px}
.rate-row{display:grid;grid-template-columns:28px 1fr 60px;align-items:center;gap:8px;font-size:11px}
.rate-row .jlabel{color:var(--dim)}
.rate-bar-bg{background:var(--border);border-radius:2px;height:8px;overflow:hidden}
.rate-bar-fill{height:100%;border-radius:2px;background:var(--acc);transition:width .2s}
.rate-bar-fill.selected-j{background:var(--ok)}
.rate-spd{color:var(--text);text-align:right;font-variant-numeric:tabular-nums}

/* direction viz */
.dir-viz{display:flex;align-items:center;justify-content:center;gap:12px;padding:10px 0;font-size:12px}
.dir-arrow{font-size:20px;transition:transform .2s}
.dir-label{color:var(--dim);font-size:11px;text-align:center}

/* amp viz */
.amp-viz{display:flex;align-items:center;justify-content:space-between;padding:8px 0;font-size:12px}
.amp-so101{background:rgba(88,166,255,.15);border:1px solid var(--acc);border-radius:6px;padding:6px 12px;text-align:center}
.amp-arrow{color:var(--acc);font-size:16px;flex:1;text-align:center}
.amp-fr5{border-radius:6px;padding:6px 12px;text-align:center;transition:all .2s}

/* singularity viz */
.sing-bar{position:relative;margin:6px 0 18px}
.sing-track{height:20px;border-radius:4px;display:flex;overflow:hidden}
.sing-danger{background:rgba(248,81,73,.6)}
.sing-warn{background:rgba(240,136,62,.5)}
.sing-clear{background:rgba(63,185,80,.35);flex:1}
.sing-labels{display:flex;font-size:10px;color:var(--dim);margin-top:4px}

/* limit range viz */
.limit-viz{position:relative;height:30px;margin:4px 0 16px}
.limit-track{position:absolute;top:10px;left:0;right:0;height:8px;background:var(--danger);border-radius:2px;opacity:.3}
.limit-active{position:absolute;top:10px;height:8px;background:var(--acc);border-radius:2px}
.limit-label-lo,.limit-label-hi{position:absolute;top:22px;font-size:10px;color:var(--dim);transform:translateX(-50%)}

/* servo filter viz */
.filter-viz{display:flex;align-items:flex-end;gap:2px;height:40px;padding:0 4px}
.filter-bar{flex:1;background:var(--acc);border-radius:2px 2px 0 0;opacity:.6;transition:height .2s}

/* toast */
#toast{position:fixed;bottom:20px;right:20px;background:var(--surf);border:1px solid var(--ok);color:var(--ok);padding:9px 16px;border-radius:8px;font-size:12px;opacity:0;transform:translateY(6px);transition:opacity .25s,transform .25s;pointer-events:none;z-index:999}
#toast.show{opacity:1;transform:none}
</style>
</head>
<body>

<header>
  <h1>⚙ FR5 Teleop Parameter Tuner</h1>
  <div class="pill"><label>Preset</label><select id="presetSel" onchange="loadPreset(this.value)"></select></div>
  <button onclick="savePreset()">Save as…</button>
  <button onclick="overwritePreset()">Overwrite</button>
  <button onclick="resetMain()">Reset</button>
  <button class="red" onclick="deletePreset()">Delete</button>
  <button class="primary" onclick="applyFiles()">▶ Apply to robot</button>
  <span id="status">Loading…</span>
</header>

<div class="layout">
  <div class="param-list" id="paramList"></div>
  <div class="focus" id="focusPanel">
    <div class="focus-empty">
      <div class="icon">⚙</div>
      <div>Click any parameter<br>to see description &amp; live effects</div>
    </div>
  </div>
</div>

<div id="toast"></div>

<script>
'use strict';

// ── Metadata for every parameter ────────────────────────────────────────────
const META = {
  "LOOP_HZ":{
    name:"Control Loop Frequency",
    short:"Commands sent to FR5 per second",
    desc:"How many ServoJ commands per second are sent to the FR5 controller. Must stay between 60–1000 Hz. Higher = smoother motion and lower latency, but more CPU and network load. The default 125 Hz gives an 8 ms cycle time.",
    effect:v=>`Command every ${(1000/+v).toFixed(2)} ms  (${+v} Hz)`,
    group:"timing"
  },
  "JOINT_SCALE[shoulder_pan]":{
    name:"Shoulder Pan — Direction",short:"Flip if FR5 J1 moves the wrong way",
    desc:"Controls whether FR5 J1 (shoulder pan) moves in the same or opposite direction as SO-101. If the robot pans left when you expect right, set this to -1.0.",
    effect:v=>+v>0?"→ Mirrors SO-101 (same direction)":"→ Reversed (opposite direction)",
    group:"scale",joint:"J1"
  },
  "JOINT_SCALE[shoulder_lift]":{
    name:"Shoulder Lift — Direction",short:"Flip if FR5 J2 lifts/lowers backwards",
    desc:"Controls whether FR5 J2 (shoulder lift) tracks SO-101 in the same or opposite direction.",
    effect:v=>+v>0?"→ Mirrors SO-101":"→ Reversed",
    group:"scale",joint:"J2"
  },
  "JOINT_SCALE[elbow_flex]":{
    name:"Elbow Flex — Direction",short:"Flip if FR5 J3 bends the wrong way",
    desc:"Controls whether FR5 J3 (elbow) tracks SO-101 elbow flex in the same or opposite direction.",
    effect:v=>+v>0?"→ Mirrors SO-101":"→ Reversed",
    group:"scale",joint:"J3"
  },
  "JOINT_SCALE[wrist_flex]":{
    name:"Wrist Flex — Direction",short:"Flip if FR5 J4 pitches the wrong way",
    desc:"Controls whether FR5 J4 (wrist flex) tracks SO-101 wrist flex in the same or opposite direction.",
    effect:v=>+v>0?"→ Mirrors SO-101":"→ Reversed",
    group:"scale",joint:"J4"
  },
  "JOINT_SCALE[wrist_roll]":{
    name:"Wrist Roll — Direction",short:"Flip if FR5 J6 rolls the wrong way",
    desc:"Controls whether FR5 J6 (wrist roll) tracks SO-101 wrist roll in the same or opposite direction.",
    effect:v=>+v>0?"→ Mirrors SO-101":"→ Reversed",
    group:"scale",joint:"J6"
  },
  "JOINT_AMP[shoulder_pan]":{
    name:"Shoulder Pan — Amplification",short:"1° SO-101 → N° FR5 J1",
    desc:"Gear ratio for shoulder pan. 1.0 = equal range. Increase if the SO-101 leader arm can't physically reach all the angles needed to drive FR5 J1 to its limits. Too high = small tremors become large robot movements.",
    effect:v=>`1° SO-101 → ${(+v).toFixed(2)}° FR5 J1`,
    group:"amp",joint:"J1"
  },
  "JOINT_AMP[shoulder_lift]":{
    name:"Shoulder Lift — Amplification",short:"1° SO-101 → N° FR5 J2",
    desc:"Gear ratio for shoulder lift. The FR5 J2 has a -265° to +85° range (350° span). A 2× amplification helps cover it with the smaller SO-101 leader.",
    effect:v=>`1° SO-101 → ${(+v).toFixed(2)}° FR5 J2`,
    group:"amp",joint:"J2"
  },
  "JOINT_AMP[elbow_flex]":{
    name:"Elbow Flex — Amplification",short:"1° SO-101 → N° FR5 J3",
    desc:"Gear ratio for elbow flex. 2× is typical because the FR5 elbow has a large ±160° range that the compact SO-101 leader can't fully span.",
    effect:v=>`1° SO-101 → ${(+v).toFixed(2)}° FR5 J3`,
    group:"amp",joint:"J3"
  },
  "JOINT_AMP[wrist_flex]":{
    name:"Wrist Flex — Amplification",short:"1° SO-101 → N° FR5 J4",
    desc:"Gear ratio for wrist flex. Higher amplification here (3×) lets fine wrist motion drive the FR5 wrist through its full range from a smaller SO-101 movement.",
    effect:v=>`1° SO-101 → ${(+v).toFixed(2)}° FR5 J4`,
    group:"amp",joint:"J4"
  },
  "JOINT_AMP[wrist_roll]":{
    name:"Wrist Roll — Amplification",short:"1° SO-101 → N° FR5 J6",
    desc:"Gear ratio for wrist roll (SO-101 motor 5 → FR5 J6).",
    effect:v=>`1° SO-101 → ${(+v).toFixed(2)}° FR5 J6`,
    group:"amp",joint:"J6"
  },
  "FR5_J4_FROZEN_DEG":{
    name:"FR5 J5 Reference Angle",short:"J5 has no SO-101 counterpart — frozen at home",
    desc:"FR5 J5 (forearm roll) has no corresponding SO-101 joint, so it stays frozen at whatever position it was in at startup. This value is stored for reference; the actual frozen angle is captured from the robot at the start of each teleoperation session.",
    effect:v=>`J5 held at home position (captured at startup)`,
    group:"amp"
  },
  "MAX_DELTA_DEG_PER_CYCLE":{
    name:"Global Rate Limit (fallback)",short:"Safety backstop — per-joint limits take priority",
    desc:"Maximum degrees any joint can move per 8 ms control cycle. This is the fallback used by the singularity scaler. Per-joint limits (MAX_DELTA_PER_JOINT) are tighter and always take priority in normal operation.",
    effect:(v,s)=>{const hz=pval(s,'LOOP_HZ')||125;return`Max ${(+v*hz).toFixed(1)}°/s  (${(+v).toFixed(4)} × ${hz} Hz)`},
    group:"ratelimit"
  },
  "MAX_DELTA_PER_JOINT[J1]":{
    name:"J1 Shoulder Pan — Max Speed",short:"Hard rate cap per cycle for FR5 J1",
    desc:"Maximum degrees FR5 J1 can advance per control cycle. This clamps motion regardless of how fast the SO-101 is moved. Reduce for smoother, safer motion; increase only after verifying the robot handles it safely.",
    effect:(v,s)=>{const hz=pval(s,'LOOP_HZ')||125;return`→ max ${(+v*hz).toFixed(1)}°/s`},
    group:"ratelimit",joint:"J1"
  },
  "MAX_DELTA_PER_JOINT[J2]":{
    name:"J2 Shoulder Lift — Max Speed",short:"Hard rate cap per cycle for FR5 J2",
    desc:"Maximum degrees FR5 J2 can advance per control cycle.",
    effect:(v,s)=>{const hz=pval(s,'LOOP_HZ')||125;return`→ max ${(+v*hz).toFixed(1)}°/s`},
    group:"ratelimit",joint:"J2"
  },
  "MAX_DELTA_PER_JOINT[J3]":{
    name:"J3 Elbow Flex — Max Speed",short:"Hard rate cap per cycle for FR5 J3",
    desc:"Maximum degrees FR5 J3 can advance per control cycle.",
    effect:(v,s)=>{const hz=pval(s,'LOOP_HZ')||125;return`→ max ${(+v*hz).toFixed(1)}°/s`},
    group:"ratelimit",joint:"J3"
  },
  "MAX_DELTA_PER_JOINT[J4]":{
    name:"J4 Wrist Flex — Max Speed",short:"Hard rate cap per cycle for FR5 J4",
    desc:"Wrist joints can tolerate higher speeds because they carry less inertia. J4 is set to 25°/s (0.20 × 125Hz) by default — 2–3× faster than shoulder joints.",
    effect:(v,s)=>{const hz=pval(s,'LOOP_HZ')||125;return`→ max ${(+v*hz).toFixed(1)}°/s`},
    group:"ratelimit",joint:"J4"
  },
  "MAX_DELTA_PER_JOINT[J5]":{
    name:"J5 (frozen) — Max Speed",short:"J5 is frozen — this limit is unused during teleoperation",
    desc:"J5 is frozen at its home position during teleoperation, so this limit is never triggered. It is included for completeness and used if J5 ever becomes active.",
    effect:()=>"J5 is frozen — this limit has no effect",
    group:"ratelimit",joint:"J5"
  },
  "MAX_DELTA_PER_JOINT[J6]":{
    name:"J6 Wrist Roll — Max Speed",short:"Hard rate cap per cycle for FR5 J6",
    desc:"Wrist roll can move faster than shoulder joints due to lower inertia. Default is 12.5°/s.",
    effect:(v,s)=>{const hz=pval(s,'LOOP_HZ')||125;return`→ max ${(+v*hz).toFixed(1)}°/s`},
    group:"ratelimit",joint:"J6"
  },
  "FR5_SERVO_VEL":{
    name:"ServoJ Velocity %",short:"% of max velocity for ServoJ trajectory",
    desc:"Velocity percentage passed to the FR5's ServoJ RPC. Controls how aggressively the controller interpolates between consecutive commands. Start at 2 and increase slowly — too high can cause jerky motion or controller errors.",
    effect:v=>`ServoJ velocity set to ${v}% of maximum`,
    group:"servo"
  },
  "FR5_FILTER_T":{
    name:"ServoJ Trajectory Filter",short:"Low-pass smoothing between consecutive commands",
    desc:"A time-constant (seconds) for the FR5's built-in trajectory filter. It smooths the motion between ServoJ calls. 0.12 s = 120 ms window. Too high = the arm visibly lags behind the leader. Too low = may amplify 8 ms noise into visible jitter.",
    effect:v=>`${(+v*1000).toFixed(0)} ms smoothing  (${+v >= 0.2 ? 'laggy' : +v <= 0.05 ? 'may jitter' : 'good'})`,
    group:"servo"
  },
  "FR5_JOINT_LIMITS[J1].lo":{name:"J1 Lower Soft Limit",short:"J1 cannot go below this angle",desc:"Software lower bound for FR5 J1. Commands are clamped here before being sent. Set 5° inside the hardware limit (-175°) for a safety margin.",effect:v=>`J1 clamped to ≥ ${(+v).toFixed(1)}°`,group:"limits",joint:"J1"},
  "FR5_JOINT_LIMITS[J1].hi":{name:"J1 Upper Soft Limit",short:"J1 cannot exceed this angle",desc:"Software upper bound for FR5 J1.",effect:v=>`J1 clamped to ≤ ${(+v).toFixed(1)}°`,group:"limits",joint:"J1"},
  "FR5_JOINT_LIMITS[J2].lo":{name:"J2 Lower Soft Limit",short:"J2 cannot go below this angle",desc:"Software lower bound for FR5 J2. Hardware limit is -265°.",effect:v=>`J2 clamped to ≥ ${(+v).toFixed(1)}°`,group:"limits",joint:"J2"},
  "FR5_JOINT_LIMITS[J2].hi":{name:"J2 Upper Soft Limit",short:"J2 cannot exceed this angle",desc:"Software upper bound for FR5 J2. Hardware limit is +85°.",effect:v=>`J2 clamped to ≤ ${(+v).toFixed(1)}°`,group:"limits",joint:"J2"},
  "FR5_JOINT_LIMITS[J3].lo":{name:"J3 Lower Soft Limit",short:"J3 cannot go below this angle",desc:"Software lower bound for FR5 J3. Hardware limit is -160°.",effect:v=>`J3 clamped to ≥ ${(+v).toFixed(1)}°`,group:"limits",joint:"J3"},
  "FR5_JOINT_LIMITS[J3].hi":{name:"J3 Upper Soft Limit",short:"J3 cannot exceed this angle",desc:"Software upper bound for FR5 J3.",effect:v=>`J3 clamped to ≤ ${(+v).toFixed(1)}°`,group:"limits",joint:"J3"},
  "FR5_JOINT_LIMITS[J4].lo":{name:"J4 Lower Soft Limit",short:"J4 (frozen) cannot go below this",desc:"Software lower bound for FR5 J4 wrist flex. Hardware limit is -265°.",effect:v=>`J4 clamped to ≥ ${(+v).toFixed(1)}°`,group:"limits",joint:"J4"},
  "FR5_JOINT_LIMITS[J4].hi":{name:"J4 Upper Soft Limit",short:"J4 cannot exceed this angle",desc:"Software upper bound for FR5 J4.",effect:v=>`J4 clamped to ≤ ${(+v).toFixed(1)}°`,group:"limits",joint:"J4"},
  "FR5_JOINT_LIMITS[J5].lo":{name:"J5 Lower Soft Limit",short:"J5 cannot go below this angle",desc:"Software lower bound for FR5 J5. J5 is frozen at home during teleoperation.",effect:v=>`J5 clamped to ≥ ${(+v).toFixed(1)}°`,group:"limits",joint:"J5"},
  "FR5_JOINT_LIMITS[J5].hi":{name:"J5 Upper Soft Limit",short:"J5 cannot exceed this angle",desc:"Software upper bound for FR5 J5.",effect:v=>`J5 clamped to ≤ ${(+v).toFixed(1)}°`,group:"limits",joint:"J5"},
  "FR5_JOINT_LIMITS[J6].lo":{name:"J6 Lower Soft Limit",short:"J6 cannot go below this angle",desc:"Software lower bound for FR5 J6 wrist roll. Hardware limit is -175°.",effect:v=>`J6 clamped to ≥ ${(+v).toFixed(1)}°`,group:"limits",joint:"J6"},
  "FR5_JOINT_LIMITS[J6].hi":{name:"J6 Upper Soft Limit",short:"J6 cannot exceed this angle",desc:"Software upper bound for FR5 J6.",effect:v=>`J6 clamped to ≤ ${(+v).toFixed(1)}°`,group:"limits",joint:"J6"},
  "GRIPPER_OPEN_PCT":{
    name:"Gripper Open Position",short:"% travel sent when FR5 gripper opens",
    desc:"Target position % (0–100) sent to the DH AG-160-95 via MoveGripper when an open command is triggered. 100 = fully open. Reduce slightly to avoid hard end-stop contact.",
    effect:v=>`FR5 gripper opens to ${v}% of full travel`,
    group:"gripper"
  },
  "GRIPPER_CLOSE_PCT":{
    name:"Gripper Close Position",short:"% travel sent when FR5 gripper closes",
    desc:"Target position % sent when closing. 0 = fully closed. Increase to reduce closing distance — useful for objects that only need a partial close to be gripped.",
    effect:v=>`FR5 gripper closes to ${v}% of travel`,
    group:"gripper"
  },
  "GRIPPER_VEL_PCT":{
    name:"Gripper Move Speed",short:"% of max gripper speed",
    desc:"How fast the DH gripper travels between open and close positions. 50% is safe. Lower for fragile objects; higher for fast pick-and-place tasks where gripper speed matters.",
    effect:v=>`Gripper moves at ${v}% of max speed`,
    group:"gripper"
  },
  "GRIPPER_FORCE_PCT":{
    name:"Gripper Grip Force",short:"% of max grip force when holding",
    desc:"Force applied by the gripper when holding an object. 50% is a safe default. Increase for heavy or slippery objects; decrease to avoid crushing soft or delicate items.",
    effect:v=>`Applies ${v}% of maximum grip force`,
    group:"gripper"
  },
  "GRIPPER_MAXTIME_MS":{
    name:"Gripper Move Timeout",short:"Max ms before a move command times out",
    desc:"If the gripper doesn't reach its target within this time (e.g., blocked by an object), the move command times out. Prevents the teleoperation loop from hanging if a gripper stall occurs.",
    effect:v=>`Times out after ${(+v/1000).toFixed(1)} s if gripper stalls`,
    group:"gripper"
  },
  "SO101_GRIPPER_OPEN_THRESHOLD":{
    name:"SO-101 Open Trigger",short:"Normalised position that triggers FR5 gripper OPEN",
    desc:"When the SO-101 gripper motor's normalised position (0 = 178°, 1 = 284°) rises above this value, the FR5 gripper opens. The dead band between this and the close threshold prevents rapid toggling during partial grips.",
    effect:v=>`FR5 gripper opens when SO-101 ≥ ${(+v*100).toFixed(0)}%`,
    group:"gripper"
  },
  "SO101_GRIPPER_CLOSE_THRESHOLD":{
    name:"SO-101 Close Trigger",short:"Normalised position that triggers FR5 gripper CLOSE",
    desc:"When the SO-101 gripper motor's normalised position drops below this value, the FR5 gripper closes. The gap between close and open thresholds forms a hysteresis dead band that prevents oscillation.",
    effect:v=>`FR5 gripper closes when SO-101 ≤ ${(+v*100).toFixed(0)}%`,
    group:"gripper"
  },
  "WRIST_WARN_DEG":{
    name:"Wrist Singularity — Warning Zone",short:"Speed starts reducing when |J4| falls below this",
    desc:"As |J4| (wrist flex) drops below this threshold, the teleop loop progressively scales down the rate limit from 100% (at this angle) to 10% (at the danger threshold). This gives the operator a 'slowing down' warning before a full stop.",
    effect:v=>`Motion slows linearly from ${v}° down to danger zone`,
    group:"singularity"
  },
  "WRIST_DANGER_DEG":{
    name:"Wrist Singularity — Full Stop",short:"Motion blocked when |J4| < this angle",
    desc:"When |J4| drops below this angle, teleoperation motion is completely blocked and the FR5 holds position. Move the SO-101 back away from wrist-straight to resume. Prevents entering a configuration where J4 and J6 axes align (infinite IK solutions).",
    effect:v=>`All motion blocked when |J4| < ${v}°`,
    group:"singularity"
  },
  "ELBOW_WARN_DEG":{
    name:"Elbow Singularity — Warning Zone",short:"Speed starts reducing when |J3| falls below this",
    desc:"As |J3| (elbow flex) drops below this threshold, motion slows progressively. This prevents the arm from snapping through a fully-extended or fully-folded configuration.",
    effect:v=>`Motion slows linearly below ${v}°`,
    group:"singularity"
  },
  "ELBOW_DANGER_DEG":{
    name:"Elbow Singularity — Full Stop",short:"Motion blocked when |J3| < this angle",
    desc:"When |J3| drops below this angle, teleoperation motion is fully blocked. The elbow singularity occurs when the arm is nearly straight or nearly folded — the Jacobian approaches a near-zero determinant.",
    effect:v=>`All motion blocked when |J3| < ${v}°`,
    group:"singularity"
  },
};

// ── State ────────────────────────────────────────────────────────────────────
let ST = null, MAIN_VALS = {}, SELECTED = null;

function pval(s, key){ return (s||ST)?.params.find(x=>x.key===key)?.value }

// ── API ──────────────────────────────────────────────────────────────────────
async function api(path, method='GET', body=null){
  const o={method,headers:{'Content-Type':'application/json'}};
  if(body) o.body=JSON.stringify(body);
  const r=await fetch(path,o); return r.json();
}
function setStatus(msg,cls=''){
  const el=document.getElementById('status');
  el.textContent=msg; el.className=cls;
  if(cls==='ok'||cls==='err') setTimeout(()=>{el.textContent='';el.className=''},3500);
}
function toast(msg){
  const t=document.getElementById('toast');
  t.textContent=msg; t.classList.add('show');
  setTimeout(()=>t.classList.remove('show'),2800);
}

// ── Format helpers ───────────────────────────────────────────────────────────
function fmtV(p,v){
  if(p.dtype==='int') return String(Math.round(+v));
  const dp=parseInt((p.fmt.match(/\.(\d+)f/)||['','3'])[1]);
  return (+v).toFixed(dp);
}
function keyHTML(key){
  const m=key.match(/^([^\[\.]+)([\[\.].*)?$/);
  if(!m) return `<span>${key}</span>`;
  return m[2]?`<span>${m[1]}</span><span class="sub">${m[2]}</span>`:`<span>${m[1]}</span>`;
}
function slPct(p,v){ return p.lo==null||p.hi==null?0:((+v-p.lo)/(p.hi-p.lo)*100).toFixed(2) }
function getEffect(key,v){
  const m=META[key]; if(!m||!m.effect) return '';
  try{ return typeof m.effect==='function'?m.effect(v,ST):''; }catch(e){ return ''; }
}
function effectClass(key,v){
  if(key.startsWith('JOINT_SCALE')) return +v>0?'pos':'neg';
  if(key.startsWith('MAX_DELTA')||key.startsWith('FR5_SERVO')) return 'pos';
  if(key.includes('SINGULARITY')||key.includes('_DANGER')) return 'danger';
  if(key.includes('_WARN')) return 'warn';
  return '';
}

// ── Build UI ─────────────────────────────────────────────────────────────────
function buildUI(st){
  ST=st; MAIN_VALS=st.main_values;
  const list=document.getElementById('paramList');
  list.innerHTML='';

  st.items.forEach(item=>{
    if(item.type==='section'){
      const d=document.createElement('div');
      d.className='sec'; d.textContent=item.label;
      list.appendChild(d); return;
    }
    const p=st.params.find(x=>x.key===item.key); if(!p) return;
    const id=p.key.replace(/[\[\].]/g,'_');
    const diff=MAIN_VALS[p.key]!==undefined && Math.abs(p.value-MAIN_VALS[p.key])>1e-9;
    const isSelected=SELECTED===p.key;
    const m=META[p.key]||{};
    const eff=getEffect(p.key,p.value);

    const row=document.createElement('div');
    row.className='row'+(diff?' changed':'')+(isSelected?' selected':'');
    row.id='row_'+id;
    row.onclick=()=>selectParam(p.key);
    row.innerHTML=`
      <div class="key-col">
        <div class="key-name">${keyHTML(p.key)}</div>
        <div class="key-short">${m.short||''}</div>
      </div>
      <span class="bound">${p.lo??''}</span>
      <input type="range" id="sl_${id}"
        min="${p.lo??0}" max="${p.hi??100}" step="${p.dtype==='int'?1:p.step/5}" value="${p.value}"
        style="--pct:${slPct(p,p.value)}%"
        oninput="onSlider('${p.key}',this.value)"
        onclick="e=>e.stopPropagation()">
      <span class="bound hi">${p.hi??''}</span>
      <input type="number" class="val${diff?' changed':''}"
        id="in_${id}" value="${fmtV(p,p.value)}" step="${p.step}"
        ${p.lo!=null?`min="${p.lo}"`:''}
        ${p.hi!=null?`max="${p.hi}"`:''}
        oninput="onNumber('${p.key}',this.value)"
        onchange="commitNumber('${p.key}',this.value)"
        onclick="event.stopPropagation()">
      <div class="effect ${effectClass(p.key,p.value)}" id="eff_${id}">${eff}</div>
    `;
    list.appendChild(row);
  });

  // preset selector
  const sel=document.getElementById('presetSel');
  sel.innerHTML=st.presets.map(n=>`<option value="${n}"${n===st.current_preset?' selected':''}>${n}</option>`).join('');
  setStatus('Ready');
  if(SELECTED) renderFocus(SELECTED);
}

// ── Param interaction ────────────────────────────────────────────────────────
let deb=null;

function onSlider(key,raw){
  const p=ST.params.find(x=>x.key===key);
  const v=p.dtype==='int'?Math.round(+raw):+raw;
  const id=key.replace(/[\[\].]/g,'_');
  const inp=document.getElementById('in_'+id);
  if(inp) inp.value=fmtV(p,v);
  const sl=document.getElementById('sl_'+id);
  if(sl) sl.style.setProperty('--pct',slPct(p,v)+'%');
  const eff=document.getElementById('eff_'+id);
  if(eff){eff.textContent=getEffect(key,v);eff.className='effect '+effectClass(key,v)}
  updateDiff(key,v); p.value=v;
  clearTimeout(deb); deb=setTimeout(()=>sendParam(key,v),60);
  if(SELECTED===key) renderFocus(key);
}

function onNumber(key,raw){
  const p=ST.params.find(x=>x.key===key);
  const v=p.dtype==='int'?parseInt(raw):parseFloat(raw);
  if(isNaN(v)) return;
  const id=key.replace(/[\[\].]/g,'_');
  const sl=document.getElementById('sl_'+id);
  if(sl){sl.value=v;sl.style.setProperty('--pct',slPct(p,v)+'%')}
  const eff=document.getElementById('eff_'+id);
  if(eff){eff.textContent=getEffect(key,v);eff.className='effect '+effectClass(key,v)}
  updateDiff(key,v);
}

function commitNumber(key,raw){
  const p=ST.params.find(x=>x.key===key);
  let v=p.dtype==='int'?parseInt(raw):parseFloat(raw);
  if(isNaN(v)) return;
  if(p.lo!=null&&v<p.lo) v=p.lo;
  if(p.hi!=null&&v>p.hi) v=p.hi;
  const id=key.replace(/[\[\].]/g,'_');
  const inp=document.getElementById('in_'+id);
  if(inp) inp.value=fmtV(p,v);
  const sl=document.getElementById('sl_'+id);
  if(sl){sl.value=v;sl.style.setProperty('--pct',slPct(p,v)+'%')}
  updateDiff(key,v); p.value=v;
  sendParam(key,v);
  if(SELECTED===key) renderFocus(key);
}

function updateDiff(key,v){
  const diff=MAIN_VALS[key]!==undefined&&Math.abs(v-MAIN_VALS[key])>1e-9;
  const id=key.replace(/[\[\].]/g,'_');
  document.getElementById('row_'+id)?.classList.toggle('changed',diff);
  document.getElementById('in_'+id)?.classList.toggle('changed',diff);
}

async function sendParam(key,value){
  try{ await api('/api/param','POST',{key,value}); }
  catch(e){ setStatus('Send error','err'); }
}

// ── Focus panel ───────────────────────────────────────────────────────────────
function selectParam(key){
  if(SELECTED){
    const prev=SELECTED.replace(/[\[\].]/g,'_');
    document.getElementById('row_'+prev)?.classList.remove('selected');
  }
  SELECTED=key;
  document.getElementById('row_'+key.replace(/[\[\].]/g,'_'))?.classList.add('selected');
  renderFocus(key);
}

function renderFocus(key){
  const p=ST.params.find(x=>x.key===key); if(!p) return;
  const m=META[key]||{name:key,short:'',desc:'No description available.',effect:null,group:''};
  const eff=getEffect(key,p.value);
  const panel=document.getElementById('focusPanel');

  panel.innerHTML=`<div class="focus-content">
    <div>
      <div class="focus-name">${m.name||key}</div>
      <div class="focus-short" style="margin-top:3px">${m.short||''}</div>
    </div>
    <div class="focus-desc">${m.desc||''}</div>
    <div class="focus-live">
      <div class="focus-live-label">Live Effect</div>
      <div class="focus-live-val" id="focus-eff">${eff||'—'}</div>
    </div>
    <div class="focus-meta">
      <div class="focus-chip">Default <span>${fmtV(p,p.default)}</span></div>
      <div class="focus-chip">Range <span>${p.lo??'—'} … ${p.hi??'∞'}</span></div>
      <div class="focus-chip">Step <span>${p.step}</span></div>
      ${m.joint?`<div class="focus-chip">FR5 joint <span>${m.joint}</span></div>`:''}
    </div>
    <div id="focus-viz"></div>
  </div>`;

  renderViz(key, p, m);
}

function renderViz(key,p,m){
  const viz=document.getElementById('focus-viz'); if(!viz) return;

  if(m.group==='gripper'&&(key.includes('THRESHOLD')||key.includes('_PCT'))){
    renderGripperViz(viz);
  } else if(m.group==='ratelimit'){
    renderRateBars(viz, m.joint);
  } else if(m.group==='scale'){
    renderDirViz(viz, p.value, m.joint);
  } else if(m.group==='amp'){
    renderAmpViz(viz, p.value, m.joint);
  } else if(m.group==='singularity'){
    renderSingViz(viz);
  } else if(m.group==='limits'){
    renderLimitViz(viz, key, m.joint);
  } else if(m.group==='servo'&&key==='FR5_FILTER_T'){
    renderFilterViz(viz, p.value);
  }
}

function renderGripperViz(container){
  const close=pval(null,'SO101_GRIPPER_CLOSE_THRESHOLD')||0.35;
  const open=pval(null,'SO101_GRIPPER_OPEN_THRESHOLD')||0.65;
  const closePct=(close*100).toFixed(0);
  const deadPct=((open-close)*100).toFixed(0);
  const openPct=((1-open)*100).toFixed(0);
  container.innerHTML=`<div class="viz">
    <div class="viz-title">Gripper Trigger Zones  (SO-101 normalised position)</div>
    <div class="grip-bar-wrap">
      <div class="grip-bar">
        <div class="grip-close" style="width:${closePct}%">CLOSE</div>
        <div class="grip-dead" style="width:${deadPct}%">dead band</div>
        <div class="grip-open" style="width:${openPct}%">OPEN</div>
      </div>
      <div class="grip-labels"><span>0%</span><span>${closePct}% close</span><span>${(open*100).toFixed(0)}% open</span><span>100%</span></div>
    </div>
    <div style="font-size:11px;color:var(--dim)">Gripper state only changes at the thresholds. The dead band prevents oscillation when the SO-101 is partially squeezed.</div>
  </div>`;
}

function renderRateBars(container, selectedJoint){
  const joints=['J1','J2','J3','J4','J5','J6'];
  const hz=pval(null,'LOOP_HZ')||125;
  const speeds=joints.map(j=>({
    j,
    dps:(pval(null,`MAX_DELTA_PER_JOINT[${j}]`)||0)*hz,
    frozen:j==='J5'
  }));
  const maxDps=Math.max(...speeds.map(s=>s.dps));
  const rows=speeds.map(s=>`
    <div class="rate-row">
      <span class="jlabel">${s.j}</span>
      <div class="rate-bar-bg">
        <div class="rate-bar-fill${s.j===selectedJoint?' selected-j':''}" style="width:${s.frozen?0:s.dps/maxDps*100}%"></div>
      </div>
      <span class="rate-spd">${s.frozen?'frozen':s.dps.toFixed(1)+'°/s'}</span>
    </div>`).join('');
  container.innerHTML=`<div class="viz">
    <div class="viz-title">All Joint Speeds at ${hz} Hz</div>
    <div class="rate-bars">${rows}</div>
    <div style="font-size:11px;color:var(--dim);margin-top:8px">Green bar = currently selected joint. Wrist joints are set faster (less inertia).</div>
  </div>`;
}

function renderDirViz(container, v, joint){
  const same=+v>0;
  container.innerHTML=`<div class="viz">
    <div class="viz-title">Direction Mapping  (${joint||'FR5'})</div>
    <div class="dir-viz">
      <div>
        <div style="font-size:20px">🤚</div>
        <div class="dir-label">SO-101</div>
      </div>
      <div style="text-align:center">
        <div style="font-size:22px;color:${same?'var(--ok)':'var(--warn)'}">${same?'→':'↔'}</div>
        <div class="dir-label" style="color:${same?'var(--ok)':'var(--warn)'}">${same?'same direction':'reversed'}</div>
      </div>
      <div>
        <div style="font-size:20px">🤖</div>
        <div class="dir-label">FR5 ${joint||''}</div>
      </div>
    </div>
    <div style="font-size:11px;color:var(--dim)">
      ${same?'When you move the leader arm in one direction, the FR5 follows in the same direction.'
             :'The FR5 moves in the opposite direction to the SO-101. Useful when the kinematic chain is mirrored.'}
    </div>
  </div>`;
}

function renderAmpViz(container, v, joint){
  const amp=+v;
  const color=amp>2.5?'var(--warn)':amp<0.8?'var(--dim)':'var(--ok)';
  const barW=Math.min(100,amp/5*100);
  container.innerHTML=`<div class="viz">
    <div class="viz-title">Amplification  (${joint||''})</div>
    <div class="amp-viz">
      <div class="amp-so101"><div style="font-size:18px">1°</div><div style="font-size:10px;color:var(--dim)">SO-101</div></div>
      <div class="amp-arrow">── ×${amp.toFixed(2)} ──▶</div>
      <div class="amp-fr5" style="background:${color}22;border:1px solid ${color}">
        <div style="font-size:18px;color:${color}">${amp.toFixed(2)}°</div>
        <div style="font-size:10px;color:var(--dim)">FR5 ${joint||''}</div>
      </div>
    </div>
    <div style="margin:6px 0;background:var(--border);border-radius:3px;height:6px;overflow:hidden">
      <div style="width:${barW}%;height:100%;background:${color};border-radius:3px;transition:width .2s"></div>
    </div>
    <div style="font-size:11px;color:var(--dim)">
      ${amp<1?'Reduced: FR5 moves less than SO-101 — precise but limited range.'
       :amp===1?'1:1 mapping — equal range on both arms.'
       :amp<2?'Slightly amplified — small increase in range.'
       :amp<3?'Amplified — covers more FR5 workspace per unit SO-101 movement.'
             :'High amplification — small SO-101 tremors produce large FR5 motion. Ensure rate limits are low enough.'}
    </div>
  </div>`;
}

function renderSingViz(container){
  const wd=pval(null,'WRIST_DANGER_DEG')||5;
  const ww=pval(null,'WRIST_WARN_DEG')||15;
  const ed=pval(null,'ELBOW_DANGER_DEG')||3;
  const ew=pval(null,'ELBOW_WARN_DEG')||8;
  const scale=90; // max display angle
  function bar(danger,warn,label){
    const dp=(danger/scale*100).toFixed(1), wp=(warn/scale*100).toFixed(1);
    return `<div style="margin-bottom:12px">
      <div style="font-size:10px;color:var(--dim);margin-bottom:4px">${label}</div>
      <div style="height:18px;display:flex;border-radius:3px;overflow:hidden">
        <div style="width:${dp}%;background:rgba(248,81,73,.6);display:flex;align-items:center;justify-content:center;font-size:9px;color:#fff">STOP</div>
        <div style="width:${wp-dp}%;background:rgba(240,136,62,.5);display:flex;align-items:center;justify-content:center;font-size:9px;color:#fff">SLOW</div>
        <div style="flex:1;background:rgba(63,185,80,.35);display:flex;align-items:center;justify-content:center;font-size:9px;color:#fff">CLEAR</div>
      </div>
      <div style="display:flex;justify-content:space-between;font-size:9px;color:var(--dim);margin-top:2px">
        <span>0°</span><span>${danger}° stop</span><span>${warn}° slow</span><span>90°+</span>
      </div>
    </div>`;
  }
  container.innerHTML=`<div class="viz">
    <div class="viz-title">Singularity Zones</div>
    ${bar(wd,ww,'Wrist (|J4|)')}
    ${bar(ed,ew,'Elbow (|J3|)')}
    <div style="font-size:11px;color:var(--dim)">Motion is progressively slowed in the SLOW zone and fully blocked in the STOP zone. Move SO-101 away to resume.</div>
  </div>`;
}

function renderLimitViz(container, key, joint){
  if(!joint) return;
  const lo=pval(null,`FR5_JOINT_LIMITS[${joint}].lo`)||0;
  const hi=pval(null,`FR5_JOINT_LIMITS[${joint}].hi`)||0;
  const span=hi-lo, total=360;
  const loPos=((lo+180)/total*100).toFixed(1);
  const w=(span/total*100).toFixed(1);
  container.innerHTML=`<div class="viz">
    <div class="viz-title">${joint} Allowed Range</div>
    <div style="position:relative;margin:6px 0 18px">
      <div style="height:14px;background:rgba(248,81,73,.25);border-radius:3px"></div>
      <div style="position:absolute;top:0;left:${loPos}%;width:${w}%;height:14px;background:var(--acc);border-radius:3px;opacity:.7"></div>
    </div>
    <div style="display:flex;justify-content:space-between;font-size:11px">
      <span style="color:var(--dim)">−180°</span>
      <span style="color:var(--ok)">${lo.toFixed(0)}° to ${hi.toFixed(0)}°  (${span.toFixed(0)}° span)</span>
      <span style="color:var(--dim)">+180°</span>
    </div>
    <div style="font-size:11px;color:var(--dim);margin-top:8px">Blue = allowed zone. Red = hardware limit zone (commands are clamped before reaching here).</div>
  </div>`;
}

function renderFilterViz(container, v){
  const ms=(+v*1000).toFixed(0);
  // show a simple impulse response curve as bars
  const n=12, bars=[];
  for(let i=0;i<n;i++){
    const h=Math.exp(-i/(+v*125+0.01));
    bars.push(`<div class="filter-bar" style="height:${(h*36).toFixed(0)}px"></div>`);
  }
  container.innerHTML=`<div class="viz">
    <div class="viz-title">Filter Impulse Response  (${ms} ms window)</div>
    <div class="filter-viz">${bars.join('')}</div>
    <div style="font-size:11px;color:var(--dim);margin-top:6px">
      Each bar = one 8 ms ServoJ cycle. The filter blends consecutive commands over this window.
      ${+v<0.05?'Very short window — may amplify jitter.'
       :+v>0.25?'Long window — noticeable lag between leader and follower.'
               :'Good balance of smoothness and responsiveness.'}
    </div>
  </div>`;
}

// ── Preset actions ────────────────────────────────────────────────────────────
async function loadPreset(name){
  try{
    const r=await api('/api/preset/load','POST',{name});
    if(r.ok){buildUI(r.state);setStatus(`Loaded '${name}'`,'ok');}
    else setStatus(r.error,'err');
  }catch(e){setStatus(e.message,'err');}
}
async function savePreset(){
  const name=prompt('Save preset as:'); if(!name?.trim()) return;
  try{
    const r=await api('/api/preset/save','POST',{name:name.trim()});
    if(r.ok){buildUI(r.state);toast(`✓ Saved '${name.trim()}'`);}
    else setStatus(r.error,'err');
  }catch(e){setStatus(e.message,'err');}
}
async function overwritePreset(){
  const name=document.getElementById('presetSel').value;
  if(!confirm(`Overwrite preset '${name}'?`)) return;
  try{
    const r=await api('/api/preset/save','POST',{name});
    if(r.ok){buildUI(r.state);toast(`✓ Overwritten '${name}'`);}
    else setStatus(r.error,'err');
  }catch(e){setStatus(e.message,'err');}
}
async function resetMain(){
  try{
    const r=await api('/api/preset/load','POST',{name:'main'});
    if(r.ok){buildUI(r.state);setStatus("Reset to 'main'",'ok');}
  }catch(e){setStatus(e.message,'err');}
}
async function deletePreset(){
  const name=document.getElementById('presetSel').value;
  if(name==='main'){alert("Cannot delete 'main'");return;}
  if(!confirm(`Delete preset '${name}'?`)) return;
  try{
    const r=await api('/api/preset/delete','POST',{name});
    if(r.ok){buildUI(r.state);toast(`Deleted '${name}'`);}
    else setStatus(r.error,'err');
  }catch(e){setStatus(e.message,'err');}
}
async function applyFiles(){
  setStatus('Applying…','busy');
  try{
    const r=await api('/api/apply','POST');
    if(r.ok){setStatus('Applied → config.py + singularity.py','ok');toast('✓ Applied to robot config');}
    else setStatus(r.error,'err');
  }catch(e){setStatus(e.message,'err');}
}

(async()=>{
  try{ const st=await api('/api/state'); buildUI(st); }
  catch(e){ setStatus('Connection failed','err'); }
})();
</script>
</body>
</html>
"""

# ── HTTP handler ──────────────────────────────────────────────────────────────
class Handler(BaseHTTPRequestHandler):
    def log_message(self,*_): pass
    def _json(self,data,code=200):
        b=json.dumps(data).encode()
        self.send_response(code); self.send_header("Content-Type","application/json")
        self.send_header("Content-Length",str(len(b))); self.end_headers(); self.wfile.write(b)
    def _html(self,html):
        b=html.encode(); self.send_response(200)
        self.send_header("Content-Type","text/html; charset=utf-8")
        self.send_header("Content-Length",str(len(b))); self.end_headers(); self.wfile.write(b)
    def _body(self):
        n=int(self.headers.get("Content-Length",0))
        return json.loads(self.rfile.read(n)) if n else {}

    def do_GET(self):
        if self.path in ("/","/index.html"): self._html(HTML)
        elif self.path=="/api/state":
            with _lock: self._json(_get_state())
        else: self.send_error(404)

    def do_POST(self):
        global _cur_preset
        body=self._body()
        if self.path=="/api/param":
            key=body.get("key"); val=body.get("value")
            with _lock:
                p=next((x for x in _params if x.key==key),None)
                if not p: self._json({"ok":False,"error":f"Unknown key: {key}"}); return
                p.set(val); self._json({"ok":True})
        elif self.path=="/api/preset/load":
            name=body.get("name")
            with _lock:
                if name not in _presets: self._json({"ok":False,"error":f"No preset '{name}'"}); return
                dict_to_params(_items,_presets[name]); _cur_preset=name
                self._json({"ok":True,"state":_get_state()})
        elif self.path=="/api/preset/save":
            name=body.get("name","").strip()
            if not name: self._json({"ok":False,"error":"Name required"}); return
            with _lock:
                _presets[name]=params_to_dict(_params); _cur_preset=name
                save_presets(_presets); self._json({"ok":True,"state":_get_state()})
        elif self.path=="/api/preset/delete":
            name=body.get("name")
            if name=="main": self._json({"ok":False,"error":"Cannot delete 'main'"}); return
            with _lock:
                _presets.pop(name,None); save_presets(_presets)
                if _cur_preset==name:
                    _cur_preset="main"; dict_to_params(_items,_presets["main"])
                self._json({"ok":True,"state":_get_state()})
        elif self.path=="/api/apply":
            with _lock: p=params_to_dict(_params)
            try:
                write_config_py(p); write_singularity_py(p)
                self._json({"ok":True})
            except Exception as exc: self._json({"ok":False,"error":str(exc)})
        else: self.send_error(404)

def _ips():
    a={"127.0.0.1"}
    try:
        s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.connect(("8.8.8.8",80))
        a.add(s.getsockname()[0]); s.close()
    except: pass
    return sorted(a)

if __name__=="__main__":
    srv=ThreadingHTTPServer(("0.0.0.0",PORT),Handler)
    print(f"\n  FR5 Teleop Parameter Tuner")
    print(f"  {'─'*38}")
    for ip in _ips(): print(f"  http://{ip}:{PORT}")
    print(f"\n  Ctrl-C to stop\n")
    try: srv.serve_forever()
    except KeyboardInterrupt: print("  Stopped.")
