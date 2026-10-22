"""Read-only local viewer: compile the current action builders and serve their output."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import threading
from datetime import datetime, timedelta, timezone
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parent
BUILD = REPO / "build" / "action_viewer"
SOURCES = [ROOT / "export_actions.c"] + [REPO / "firmware" / name for name in (
    "mission_actions.h", "raw_pick_action.h", "placement_heights.h",
    "mechanism_action.h", "turret_motion.h", "ring_pose.h", "main.c",
)]
LOCK = threading.Lock()
cached_hash = None
cached_data = None


def action_data():
    global cached_hash, cached_data
    with LOCK:
        hashes = [{"path": p.relative_to(REPO).as_posix(),
                   "sha256": hashlib.sha256(p.read_bytes()).hexdigest()} for p in SOURCES
                  if p.name != "placement_heights.h" or p.is_file()]
        digest = hashlib.sha256(json.dumps(hashes).encode()).hexdigest()
        if digest == cached_hash:
            return cached_data
        compiler = shutil.which("gcc")
        if not compiler:
            candidate = Path("E:/Qt/Tools/mingw1310_64/bin/gcc.exe")
            if candidate.is_file():
                compiler = str(candidate)
        if not compiler:
            raise RuntimeError("未找到 GCC，请将已有编译器加入 PATH；无需安装新库。")
        BUILD.mkdir(parents=True, exist_ok=True)
        compiler_env = os.environ.copy()
        compiler_env["PATH"] = str(Path(compiler).parent) + os.pathsep + compiler_env.get("PATH", "")
        executable = BUILD / ("export_actions.exe" if os.name == "nt" else "export_actions")
        result = subprocess.run([compiler, "-std=c99", "-O2", str(ROOT / "export_actions.c"),
                                 "-o", str(executable), "-lm"],
                                cwd=REPO, env=compiler_env, capture_output=True, text=True, timeout=45)
        if result.returncode:
            raise RuntimeError("动作导出编译失败：\n" + result.stderr[-3000:])
        result = subprocess.run([str(executable)], env=compiler_env, capture_output=True, text=True, check=True, timeout=10)
        data = json.loads(result.stdout)
        data["generatedAt"] = datetime.now(timezone(timedelta(hours=8))).isoformat(timespec="seconds")
        data["sourceHash"] = digest[:12]
        data["sources"] = hashes
        cached_data = json.dumps(data, ensure_ascii=False).encode("utf-8")
        cached_hash = digest
        return cached_data


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT), **kwargs)

    def do_GET(self):
        if urlsplit(self.path).path == "/api/actions":
            try:
                data = action_data()
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(data)
            except Exception as error:
                data = json.dumps({"error": str(error)}, ensure_ascii=False).encode("utf-8")
                self.send_response(503)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            return
        super().do_GET()

    def end_headers(self):
        self.send_header("Cache-Control", "no-cache")
        super().end_headers()

    def log_message(self, fmt, *args):
        if args and "200" not in str(args[1:2]):
            super().log_message(fmt, *args)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="动作组本地俯视回放，只读固件源码，不连接硬件")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--export-only", action="store_true")
    args = parser.parse_args()
    if not (ROOT / "node_modules/three/build/three.module.js").is_file():
        parser.error("缺少 Three.js，请使用 tools/start-action-viewer.ps1 启动以安装到 E 盘工程目录。")
    action_data()
    if args.export_only:
        print(action_data().decode("utf-8"))
    else:
        server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
        print(f"动作组回放：http://127.0.0.1:{args.port}  （Ctrl+C 停止，只读源码）", flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            server.server_close()
