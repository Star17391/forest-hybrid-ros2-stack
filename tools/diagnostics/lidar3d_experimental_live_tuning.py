#!/usr/bin/env python3
"""Live tuning web UI for lidar3d_experimental_node (CSF + clustering)."""

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

NODE_NAME = "/lidar3d_experimental_node"

PARAM_SCHEMA: list[dict[str, Any]] = [
    {"name": "enabled", "type": "bool", "group": "pipeline"},
    {"name": "pipeline_sprint", "type": "int", "min": 1, "max": 3, "step": 1, "group": "pipeline"},
    {"name": "voxel_leaf_size_m", "type": "float", "min": 0.02, "max": 0.30, "step": 0.01, "group": "preprocess"},
    {"name": "min_range_m", "type": "float", "min": 0.0, "max": 5.0, "step": 0.05, "group": "preprocess"},
    {"name": "max_range_m", "type": "float", "min": 2.0, "max": 60.0, "step": 0.5, "group": "preprocess"},
    {"name": "min_z_m", "type": "float", "min": -5.0, "max": 2.0, "step": 0.05, "group": "preprocess"},
    {"name": "max_z_m", "type": "float", "min": 1.0, "max": 15.0, "step": 0.1, "group": "preprocess"},
    {"name": "csf.cloth_resolution", "type": "float", "min": 0.1, "max": 2.5, "step": 0.05, "group": "csf"},
    {"name": "csf.rigidness", "type": "int", "min": 1, "max": 5, "step": 1, "group": "csf"},
    {"name": "csf.iterations", "type": "int", "min": 50, "max": 2000, "step": 50, "group": "csf"},
    {"name": "csf.class_threshold", "type": "float", "min": 0.05, "max": 2.0, "step": 0.05, "group": "csf"},
    {"name": "csf.time_step", "type": "float", "min": 0.1, "max": 2.0, "step": 0.05, "group": "csf"},
    {"name": "csf.slope_smooth", "type": "bool", "group": "csf"},
    {"name": "clustering.tolerance", "type": "float", "min": 0.05, "max": 2.0, "step": 0.01, "group": "clustering"},
    {"name": "clustering.min_cluster_size", "type": "int", "min": 2, "max": 500, "step": 1, "group": "clustering"},
    {"name": "clustering.max_cluster_size", "type": "int", "min": 10, "max": 50000, "step": 10, "group": "clustering"},
    {"name": "tree.min_height_m", "type": "float", "min": 0.2, "max": 3.0, "step": 0.05, "group": "tree"},
    {"name": "tree.max_xy_extent_m", "type": "float", "min": 0.3, "max": 3.0, "step": 0.05, "group": "tree"},
    {"name": "tree.min_verticality", "type": "float", "min": 0.2, "max": 0.95, "step": 0.05, "group": "tree"},
    {"name": "tree.min_points", "type": "int", "min": 3, "max": 100, "step": 1, "group": "tree"},
    {"name": "tree.max_candidates_per_frame", "type": "int", "min": 1, "max": 32, "step": 1, "group": "tree"},
]


@dataclass
class ServerConfig:
    host: str
    port: int
    node_name: str


_ros_lock = threading.Lock()
_ros_node: Node | None = None
_executor: SingleThreadedExecutor | None = None
_param_client: Any = None
_target_node: str = NODE_NAME
_SERVICE_TIMEOUT_SEC = 8.0


class _SyncParamClient:
    def __init__(self, node: Node, executor: SingleThreadedExecutor, remote_node_name: str):
        self._executor = executor
        self._async = AsyncParameterClient(node, remote_node_name)

    def _spin(self, future: rclpy.task.Future) -> Any:
        self._executor.spin_until_future_complete(future, timeout_sec=_SERVICE_TIMEOUT_SEC)
        if not future.done():
            raise RuntimeError("parameter service timeout")
        exc = future.exception()
        if exc is not None:
            raise exc
        return future.result()

    def get_parameters(self, names: list[str]) -> list[Parameter]:
        if not self._async.wait_for_services(timeout_sec=2.0):
            raise RuntimeError("parameter services not ready for experimental node")
        resp = self._spin(self._async.get_parameters(names))
        return [
            Parameter.from_parameter_msg(ParameterMsg(name=n, value=v))
            for n, v in zip(names, resp.values)
        ]

    def set_parameters(self, parameters: list[Parameter]) -> None:
        if not self._async.wait_for_services(timeout_sec=2.0):
            raise RuntimeError("parameter services not ready")
        resp = self._spin(self._async.set_parameters(parameters))
        for result in resp.results:
            if not result.successful:
                raise RuntimeError(result.reason or "set_parameters failed")


def _ensure_ros() -> _SyncParamClient:
    global _ros_node, _executor, _param_client, _target_node
    with _ros_lock:
        if _param_client is None:
            if not rclpy.ok():
                rclpy.init()
            _ros_node = rclpy.create_node("lidar3d_experimental_live_tuning")
            _executor = SingleThreadedExecutor()
            _executor.add_node(_ros_node)
            remote = _target_node.strip().lstrip("/")
            _param_client = _SyncParamClient(_ros_node, _executor, remote)
        return _param_client


def configure_target_node(node_name: str) -> None:
    global _target_node, _param_client
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
    return param.value


def get_current_params() -> dict[str, Any]:
    client = _ensure_ros()
    names = [p["name"] for p in PARAM_SCHEMA]
    with _ros_lock:
        results = client.get_parameters(names)
    by_name = {p.name: p for p in results}
    return {
        spec["name"]: _value_from_param(by_name.get(spec["name"]), spec["type"])
        for spec in PARAM_SCHEMA
    }


def set_param(name: str, value: Any, ptype: str) -> None:
    client = _ensure_ros()
    with _ros_lock:
        client.set_parameters([_param_from_value(name, value, ptype)])


def save_yaml(path: Path, params: dict[str, Any]) -> None:
    lines = ["lidar3d_experimental_node:", "  ros__parameters:"]
    for spec in PARAM_SCHEMA:
        name = spec["name"]
        if name not in params or params[name] is None:
            continue
        v = params[name]
        if isinstance(v, bool):
            val = "true" if v else "false"
        elif isinstance(v, str):
            val = f'"{v}"'
        else:
            val = str(v)
        lines.append(f"    {name}: {val}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def html_page() -> str:
    return """<!doctype html>
<html><head><meta charset='utf-8'><title>LiDAR3D Experimental Tuning</title>
<style>
body{font-family:Arial,sans-serif;margin:16px;background:#0d1117;color:#e6edf3}
.grid{display:grid;grid-template-columns:240px 1fr 100px;gap:8px;align-items:center}
.row{padding:6px 0;border-bottom:1px solid #21262d}.group{margin-top:16px}
input[type=range]{width:100%} .ok{color:#3fb950}.err{color:#f85149}
</style></head><body>
<h2>Experimental pipeline (CSF + clustering)</h2>
<p>Target: <code>/lidar3d_experimental_node</code> — legacy node unchanged.</p>
<button onclick='refreshAll()'>Refresh</button>
<button onclick='saveYaml()'>Save YAML</button>
<input id='savePath' size='55' value='tools/diagnostics/lidar3d_experimental_overlay.yaml'/>
<span id='status'></span>
<div id='app'></div>
<script>
let schema=[], current={};
const groups=["pipeline","preprocess","csf","clustering","tree"];
function setStatus(m,ok=true){const s=document.getElementById('status');s.className=ok?'ok':'err';s.textContent=m;}
async function api(p,o){const r=await fetch(p,o);if(!r.ok)throw new Error(await r.text());return r.json();}
function fmt(p,v){
  if(v===null||v===undefined)return '';
  if(p.type==='float')return Number(v).toFixed(3).replace(/\\.?0+$/,'');
  return String(v);
}
function row(p,v){
  const fid='val_'+p.name.replace(/\\./g,'_');
  if(p.type==='bool')return `<div class='row grid'><div>${p.name}</div><input type='checkbox' ${v?'checked':''} onchange='set("${p.name}",this.checked,"bool")'/><div id='${fid}'>${v?'true':'false'}</div></div>`;
  const disp=fmt(p,v??p.min);
  return `<div class='row grid'><div>${p.name}</div><input type='range' min='${p.min}' max='${p.max}' step='${p.step}' value='${v??p.min}' oninput='preview("${p.name}",this.value,"${p.type}")' onchange='set("${p.name}",this.value,"${p.type}")'/><div id='${fid}'>${disp}</div></div>`;
}
function render(){let h='';for(const g of groups){const ps=schema.filter(x=>x.group===g);if(!ps.length)continue;h+=`<h3>${g}</h3>`;for(const p of ps)h+=row(p,current[p.name]);}document.getElementById('app').innerHTML=h;}
function updateLabel(name,val){
  const el=document.getElementById('val_'+name.replace(/\\./g,'_'));
  if(!el)return;
  const p=schema.find(x=>x.name===name);
  el.textContent=p?fmt(p,val):String(val);
}
function preview(n,v,t){current[n]=(t==='int')?parseInt(v,10):parseFloat(v);updateLabel(n,current[n]);}
async function refreshAll(){schema=(await api('/api/schema')).schema;current=(await api('/api/current')).values;render();setStatus('ok — lido do nó');}
async function set(n,v,t){
  try{
    const body={name:n,value:(t==='bool')?!!v:(t==='int')?parseInt(v,10):parseFloat(v)};
    const r=await api('/api/set',{method:'POST',headers:{'content-type':'application/json'},body:JSON.stringify(body)});
    if(r.value!==undefined){current[n]=r.value;updateLabel(n,r.value);}
    setStatus('aplicado '+n+'='+fmt(schema.find(x=>x.name===n)||{type:t},current[n]));
  }catch(e){setStatus(String(e),false);await refreshAll();}
}
async function saveYaml(){const path=document.getElementById('savePath').value;try{const r=await api('/api/save',{method:'POST',headers:{'content-type':'application/json'},body:JSON.stringify({path})});setStatus('saved '+r.path);}catch(e){setStatus(String(e),false);}}
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
        return {} if length <= 0 else json.loads(self.rfile.read(length).decode("utf-8"))

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
                self._send_json({"values": get_current_params()})
            except Exception as e:
                self._send_json({"error": str(e)}, 500)
            return
        self._send_json({"error": "not found"}, 404)

    def do_POST(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        data = self._read_json()
        if path == "/api/set":
            spec = next((p for p in PARAM_SCHEMA if p["name"] == data.get("name")), None)
            if spec is None:
                self._send_json({"error": "unknown param"}, 400)
                return
            try:
                raw = data["value"]
                if spec["type"] == "bool":
                    raw = raw in (True, "true", "1", 1)
                elif spec["type"] == "int":
                    raw = int(raw)
                elif spec["type"] == "float":
                    raw = float(raw)
                set_param(data["name"], raw, spec["type"])
                # Read back from node so UI shows the value ROS actually holds.
                confirmed = get_current_params().get(data["name"])
                self._send_json({"ok": True, "value": confirmed})
            except Exception as e:
                self._send_json({"error": str(e)}, 500)
            return
        if path == "/api/save":
            try:
                save_yaml(Path(data.get("path", "tools/diagnostics/lidar3d_experimental_overlay.yaml")),
                          get_current_params())
                self._send_json({"ok": True, "path": data.get("path")})
            except Exception as e:
                self._send_json({"error": str(e)}, 500)
            return
        self._send_json({"error": "not found"}, 404)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8766)
    ap.add_argument("--node", default=NODE_NAME)
    args = ap.parse_args()
    node_name = args.node if args.node.startswith("/") else f"/{args.node}"
    configure_target_node(node_name)
    Handler.config = ServerConfig(args.host, args.port, node_name)
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"Experimental tuning: http://{args.host}:{args.port} -> {node_name}")
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
