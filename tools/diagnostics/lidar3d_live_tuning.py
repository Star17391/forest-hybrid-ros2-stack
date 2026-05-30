#!/usr/bin/env python3
"""Live tuning web UI for lidar3d_segmentation_node.

Uses rclpy SyncParametersClient (one batch get/set per request) instead of
spawning `ros2 param` subprocesses per parameter.
"""

from __future__ import annotations

import argparse
import json
import threading
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import rclpy
from rcl_interfaces.msg import Parameter as ParameterMsg
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient


NODE_NAME = "/lidar3d_segmentation_node"

PARAM_SCHEMA: list[dict[str, Any]] = [
    # Preprocessing
    {"name": "voxel_leaf_size_m", "type": "float", "min": 0.02, "max": 0.30, "step": 0.01, "group": "preprocess"},
    {"name": "trunk_voxel_leaf_size_m", "type": "float", "min": 0.0, "max": 0.20, "step": 0.01, "group": "preprocess"},
    {"name": "min_range_m", "type": "float", "min": 0.0, "max": 5.0, "step": 0.05, "group": "preprocess"},
    {"name": "max_range_m", "type": "float", "min": 2.0, "max": 60.0, "step": 0.5, "group": "preprocess"},
    {"name": "min_z_m", "type": "float", "min": -5.0, "max": 2.0, "step": 0.05, "group": "preprocess"},
    {"name": "max_z_m", "type": "float", "min": 1.0, "max": 15.0, "step": 0.1, "group": "preprocess"},
    # Ground / terrain
    {"name": "ground_method", "type": "enum", "values": ["grid", "ransac"], "group": "ground"},
    {"name": "grid_size_x_m", "type": "float", "min": 5.0, "max": 80.0, "step": 1.0, "group": "ground"},
    {"name": "grid_size_y_m", "type": "float", "min": 5.0, "max": 80.0, "step": 1.0, "group": "ground"},
    {"name": "grid_resolution_m", "type": "float", "min": 0.05, "max": 1.0, "step": 0.01, "group": "ground"},
    {"name": "grid_ground_height_thresh_m", "type": "float", "min": 0.02, "max": 1.0, "step": 0.01, "group": "ground"},
    {"name": "grid_hole_depth_m", "type": "float", "min": 0.02, "max": 1.0, "step": 0.01, "group": "ground"},
    {"name": "grid_inpaint_passes", "type": "int", "min": 0, "max": 20, "step": 1, "group": "ground"},
    {"name": "grid_height_percentile", "type": "float", "min": 0.01, "max": 0.50, "step": 0.01, "group": "ground"},
    {"name": "grid_smooth_max_step_m", "type": "float", "min": 0.05, "max": 2.0, "step": 0.01, "group": "ground"},
    {"name": "grid_smooth_clamp_passes", "type": "int", "min": 0, "max": 20, "step": 1, "group": "ground"},
    {"name": "grid_smooth_median_radius_cells", "type": "int", "min": 0, "max": 6, "step": 1, "group": "ground"},
    {"name": "grid_ground_neighbor_cells", "type": "int", "min": 0, "max": 10, "step": 1, "group": "ground"},
    {"name": "ground_connectivity_enable", "type": "bool", "group": "ground"},
    {"name": "ground_connectivity_max_step_m", "type": "float", "min": 0.02, "max": 2.0, "step": 0.01, "group": "ground"},
    {"name": "ground_connectivity_seed_radius_m", "type": "float", "min": 0.2, "max": 20.0, "step": 0.1, "group": "ground"},
    # Trunks / detector
    {"name": "trunk_method", "type": "enum", "values": ["cluster", "column", "slice"], "group": "trunk"},
    {"name": "trunk_cluster_tolerance_m", "type": "float", "min": 0.05, "max": 2.0, "step": 0.01, "group": "trunk"},
    {"name": "trunk_min_cluster_size", "type": "int", "min": 1, "max": 2000, "step": 1, "group": "trunk"},
    {"name": "trunk_max_cluster_size", "type": "int", "min": 5, "max": 20000, "step": 5, "group": "trunk"},
    {"name": "trunk_min_height_m", "type": "float", "min": 0.05, "max": 5.0, "step": 0.01, "group": "trunk"},
    {"name": "trunk_max_radius_m", "type": "float", "min": 0.05, "max": 2.0, "step": 0.01, "group": "trunk"},
    {"name": "trunk_min_verticality", "type": "float", "min": 0.0, "max": 1.0, "step": 0.01, "group": "trunk"},
    {"name": "ndsm_trunk_min_m", "type": "float", "min": 0.0, "max": 5.0, "step": 0.01, "group": "trunk"},
    {"name": "ndsm_trunk_max_m", "type": "float", "min": 0.2, "max": 8.0, "step": 0.01, "group": "trunk"},
    {"name": "slice_min_points_per_cluster", "type": "int", "min": 1, "max": 200, "step": 1, "group": "trunk"},
    {"name": "slice_max_stems_per_frame", "type": "int", "min": 1, "max": 200, "step": 1, "group": "trunk"},
    # Column detector
    {"name": "column_min_points_per_cell", "type": "int", "min": 1, "max": 50, "step": 1, "group": "column"},
    {"name": "column_min_cells_per_column", "type": "int", "min": 1, "max": 100, "step": 1, "group": "column"},
    {"name": "column_min_points_per_column", "type": "int", "min": 1, "max": 1000, "step": 1, "group": "column"},
    {"name": "column_max_points_per_column", "type": "int", "min": 10, "max": 50000, "step": 10, "group": "column"},
    {"name": "column_max_columns_per_frame", "type": "int", "min": 1, "max": 500, "step": 1, "group": "column"},
    # Cylinder fit
    {"name": "cylinder_min_height_m", "type": "float", "min": 0.05, "max": 6.0, "step": 0.01, "group": "cylinder"},
    {"name": "cylinder_max_radius_m", "type": "float", "min": 0.05, "max": 2.0, "step": 0.01, "group": "cylinder"},
    {"name": "cylinder_max_rmse_m", "type": "float", "min": 0.01, "max": 2.0, "step": 0.01, "group": "cylinder"},
    {"name": "cylinder_min_inlier_ratio", "type": "float", "min": 0.0, "max": 1.0, "step": 0.01, "group": "cylinder"},
    {"name": "cylinder_inlier_dist_m", "type": "float", "min": 0.01, "max": 1.0, "step": 0.01, "group": "cylinder"},
    {"name": "cylinder_max_slice_height_m", "type": "float", "min": 0.1, "max": 10.0, "step": 0.05, "group": "cylinder"},
    # Tracking and debug
    {"name": "landmark_assoc_max_xy_m", "type": "float", "min": 0.05, "max": 5.0, "step": 0.01, "group": "tracking"},
    {"name": "landmark_max_misses", "type": "int", "min": 0, "max": 500, "step": 1, "group": "tracking"},
    {"name": "always_publish_segmentation_clouds", "type": "bool", "group": "debug"},
    {"name": "pipeline_log_interval_frames", "type": "int", "min": 1, "max": 2000, "step": 1, "group": "debug"},
    # Restart-only flags (editable but not applied live by node callback)
    {"name": "publish_ndsm_debug", "type": "bool", "group": "restart_only", "requires_restart": True},
    {"name": "publish_slice_pipeline_debug", "type": "bool", "group": "restart_only", "requires_restart": True},
    {"name": "publish_debug_stats", "type": "bool", "group": "restart_only", "requires_restart": True},
    {"name": "publish_terrain_mesh", "type": "bool", "group": "restart_only", "requires_restart": True},
    {"name": "publish_ground_suspended_debug", "type": "bool", "group": "restart_only", "requires_restart": True},
]


@dataclass
class ServerConfig:
    host: str
    port: int
    node_name: str


_ros_lock = threading.Lock()
_ros_node: Node | None = None
_executor: SingleThreadedExecutor | None = None
_param_client: AsyncParameterClient | None = None
_target_node: str = NODE_NAME
_SERVICE_TIMEOUT_SEC = 8.0


class _SyncParamClient:
    """Thin sync wrapper (Jazzy has AsyncParameterClient only)."""

    def __init__(self, node: Node, executor: SingleThreadedExecutor, remote_node_name: str):
        self._executor = executor
        self._async = AsyncParameterClient(node, remote_node_name)
        self.remote_node_name = remote_node_name

    def _spin(self, future: rclpy.task.Future) -> Any:
        self._executor.spin_until_future_complete(future, timeout_sec=_SERVICE_TIMEOUT_SEC)
        if not future.done():
            raise RuntimeError(
                f"parameter service timeout ({_SERVICE_TIMEOUT_SEC}s) on {self.remote_node_name}"
            )
        exc = future.exception()
        if exc is not None:
            raise exc
        return future.result()

    def get_parameters(self, names: list[str]) -> list[Parameter]:
        if not self._async.wait_for_services(timeout_sec=2.0):
            raise RuntimeError(
                f"parameter services not ready for '{self.remote_node_name}' "
                "(is forest up and lidar3d_segmentation_node running?)"
            )
        resp = self._spin(self._async.get_parameters(names))
        return [
            Parameter.from_parameter_msg(ParameterMsg(name=n, value=v))
            for n, v in zip(names, resp.values)
        ]

    def set_parameters(self, parameters: list[Parameter]) -> None:
        if not self._async.wait_for_services(timeout_sec=2.0):
            raise RuntimeError(f"parameter services not ready for '{self.remote_node_name}'")
        resp = self._spin(self._async.set_parameters(parameters))
        for result in resp.results:
            if not result.successful:
                raise RuntimeError(result.reason or "set_parameters failed")


def _remote_node_service_name(node_name: str) -> str:
    """AsyncParameterClient expects name without leading slash."""
    return node_name.strip().lstrip("/")


def _ensure_ros() -> _SyncParamClient:
    global _ros_node, _executor, _param_client, _target_node
    with _ros_lock:
        if _param_client is None:
            if not rclpy.ok():
                rclpy.init()
            _ros_node = rclpy.create_node("lidar3d_live_tuning")
            _executor = SingleThreadedExecutor()
            _executor.add_node(_ros_node)
            remote = _remote_node_service_name(_target_node)
            _param_client = _SyncParamClient(_ros_node, _executor, remote)
        return _param_client


def configure_target_node(node_name: str) -> None:
    global _target_node, _param_client, _executor
    _target_node = node_name if node_name.startswith("/") else f"/{node_name}"
    with _ros_lock:
        _param_client = None


def _param_from_value(name: str, value: Any, ptype: str) -> Parameter:
    if ptype == "bool":
        return Parameter(name, Parameter.Type.BOOL, bool(value))
    if ptype == "int":
        return Parameter(name, Parameter.Type.INTEGER, int(value))
    if ptype == "float":
        return Parameter(name, Parameter.Type.DOUBLE, float(value))
    return Parameter(name, Parameter.Type.STRING, str(value))


def _value_from_param(param: Parameter | None, ptype: str) -> Any:
    if param is None or param.type_ == Parameter.Type.NOT_SET:
        return None
    if ptype == "bool":
        return bool(param.value)
    if ptype == "int":
        return int(param.value)
    if ptype == "float":
        return float(param.value)
    if ptype == "enum":
        return str(param.value)
    return param.value


def get_current_params(node_name: str) -> dict[str, Any]:
    client = _ensure_ros()
    names = [p["name"] for p in PARAM_SCHEMA]
    with _ros_lock:
        results = client.get_parameters(names)
    by_name = {p.name: p for p in results}
    out: dict[str, Any] = {}
    for spec in PARAM_SCHEMA:
        name = spec["name"]
        out[name] = _value_from_param(by_name.get(name), spec["type"])
    return out


def set_param(node_name: str, name: str, value: Any, ptype: str) -> str:
    client = _ensure_ros()
    param = _param_from_value(name, value, ptype)
    with _ros_lock:
        client.set_parameters([param])
    return "ok"


def save_yaml(path: Path, params: dict[str, Any]) -> None:
    lines = [
        "lidar3d_segmentation_node:",
        "  ros__parameters:",
    ]
    for p in PARAM_SCHEMA:
        name = p["name"]
        if name not in params or params[name] is None:
            continue
        v = params[name]
        if isinstance(v, bool):
            val = "true" if v else "false"
        elif isinstance(v, str):
            val = f"\"{v}\""
        else:
            val = str(v)
        lines.append(f"    {name}: {val}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def html_page() -> str:
    return """<!doctype html>
<html><head><meta charset='utf-8'><title>LiDAR3D Live Tuning</title>
<style>
body{font-family:Arial,sans-serif;margin:16px;background:#111;color:#eee}
.grid{display:grid;grid-template-columns:220px 1fr 120px 120px;gap:8px;align-items:center}
.row{padding:6px 0;border-bottom:1px solid #222}.group{margin-top:18px}
input[type=range]{width:100%} button{padding:6px 10px}
.restart{color:#ffb347}.ok{color:#7dd37d}.err{color:#ff6b6b}
</style></head><body>
<h2>LiDAR3D live tuning</h2>
<div>
  <button onclick='refreshAll()'>Refresh</button>
  <button onclick='saveYaml()'>Save YAML</button>
  <input id='savePath' size='60' value='tools/diagnostics/lidar3d_live_overlay.yaml'/>
  <span id='status'></span>
</div>
<div id='app'></div>
<script>
let schema=[], current={};
const groups=["preprocess","ground","trunk","column","cylinder","tracking","debug","restart_only"];
function setStatus(msg, ok=true){const s=document.getElementById('status');s.className=ok?'ok':'err';s.textContent=msg;}
async function api(path, opts){const r=await fetch(path,opts);if(!r.ok) throw new Error(await r.text());return r.json();}
function rowTemplate(p,v){
  const restart=p.requires_restart?'<span class="restart">restart</span>':'';
  if(p.type==="bool"){return `<div class='row grid'><div>${p.name} ${restart}</div><div></div><input type='checkbox' ${v?'checked':''} onchange='applyBool("${p.name}",this.checked)'/><div>${v}</div></div>`;}
  if(p.type==="enum"){const opts=p.values.map(x=>`<option ${x===v?'selected':''}>${x}</option>`).join('');return `<div class='row grid'><div>${p.name} ${restart}</div><div></div><select onchange='applyEnum("${p.name}",this.value)'>${opts}</select><div>${v??''}</div></div>`;}
  return `<div class='row grid'><div>${p.name} ${restart}</div><input type='range' min='${p.min}' max='${p.max}' step='${p.step}' value='${v??p.min}' oninput='updateLabel("${p.name}",this.value)' onchange='applyNum("${p.name}",this.value)'/><input id='num_${p.name}' value='${v??""}' onchange='applyNum("${p.name}",this.value)'/><div id='val_${p.name}'>${v??""}</div></div>`;
}
function updateLabel(name,val){document.getElementById('val_'+name).textContent=val;const n=document.getElementById('num_'+name); if(n) n.value=val;}
function render(){const app=document.getElementById('app'); let html=''; for(const g of groups){const ps=schema.filter(p=>p.group===g); if(!ps.length) continue; html+=`<div class='group'><h3>${g}</h3>`; for(const p of ps){html+=rowTemplate(p,current[p.name]);} html+='</div>'; } app.innerHTML=html;}
async function refreshAll(){try{schema=(await api('/api/schema')).schema; current=(await api('/api/current')).values; render(); setStatus('updated');}catch(e){setStatus(String(e),false);} }
async function applyNum(name,val){try{await api('/api/set',{method:'POST',headers:{'content-type':'application/json'},body:JSON.stringify({name,value:val})}); setStatus(`set ${name}`);}catch(e){setStatus(String(e),false);} }
async function applyBool(name,val){try{await api('/api/set',{method:'POST',headers:{'content-type':'application/json'},body:JSON.stringify({name,value:val})}); setStatus(`set ${name}`);}catch(e){setStatus(String(e),false);} }
async function applyEnum(name,val){try{await api('/api/set',{method:'POST',headers:{'content-type':'application/json'},body:JSON.stringify({name,value:val})}); setStatus(`set ${name}`);}catch(e){setStatus(String(e),false);} }
async function saveYaml(){const path=document.getElementById('savePath').value; try{const r=await api('/api/save',{method:'POST',headers:{'content-type':'application/json'},body:JSON.stringify({path})}); setStatus('saved '+r.path);}catch(e){setStatus(String(e),false);} }
refreshAll();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    config: ServerConfig

    def _send_json(self, obj: Any, status: int = 200) -> None:
        body = json.dumps(obj).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            return {}
        return json.loads(self.rfile.read(length).decode("utf-8"))

    def do_GET(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if path == "/":
            body = html_page().encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if path == "/api/schema":
            self._send_json({"schema": PARAM_SCHEMA})
            return
        if path == "/api/current":
            try:
                self._send_json({"values": get_current_params(self.config.node_name)})
            except Exception as e:
                self._send_json({"error": str(e)}, 500)
            return
        self._send_json({"error": "not found"}, 404)

    def do_POST(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        data = self._read_json()
        if path == "/api/set":
            name = data.get("name")
            value = data.get("value")
            spec = next((p for p in PARAM_SCHEMA if p["name"] == name), None)
            if spec is None:
                self._send_json({"error": f"unknown param {name}"}, 400)
                return
            try:
                out = set_param(self.config.node_name, name, value, spec["type"])
                self._send_json({"ok": True, "out": out})
            except Exception as e:
                self._send_json({"error": str(e)}, 500)
            return
        if path == "/api/save":
            out_path = data.get("path", "tools/diagnostics/lidar3d_live_overlay.yaml")
            try:
                vals = get_current_params(self.config.node_name)
                save_yaml(Path(out_path), vals)
                self._send_json({"ok": True, "path": out_path})
            except Exception as e:
                self._send_json({"error": str(e)}, 500)
            return
        self._send_json({"error": "not found"}, 404)


def main() -> int:
    ap = argparse.ArgumentParser(description="Live tuning sliders for lidar3d segmentation")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--node", default=NODE_NAME)
    args = ap.parse_args()

    node_name = args.node if args.node.startswith("/") else f"/{args.node}"
    configure_target_node(node_name)

    Handler.config = ServerConfig(host=args.host, port=args.port, node_name=node_name)
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"Live tuning server: http://{args.host}:{args.port}")
    print(f"Target node: {node_name} (rclpy AsyncParameterClient, sync wrapper)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        with _ros_lock:
            if _ros_node is not None:
                _ros_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
