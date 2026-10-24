"""Regenerate a browser-local snapshot using the current firmware planner."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
from datetime import datetime, timezone, timedelta

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parent
SOURCES = [ROOT / "export_routes.c", REPO / "firmware/preset_route.h",
           REPO / "firmware/route_candidates.h",
           REPO / "firmware/route_plan.h", REPO / "firmware/arc_route_blend.h",
           REPO / "shared/obstacle_regions.h"]


def export():
    compiler = shutil.which("gcc") or "E:/Qt/Tools/mingw1310_64/bin/gcc.exe"
    if not Path(compiler).is_file():
        raise RuntimeError("未找到已有 GCC，请将其加入 PATH；本工具不安装依赖。")
    build = REPO / ".deps/route-viewer"
    build.mkdir(parents=True, exist_ok=True)
    executable = build / ("export_routes.exe" if os.name == "nt" else "export_routes")
    env = os.environ.copy()
    env["PATH"] = str(Path(compiler).parent) + os.pathsep + env.get("PATH", "")
    subprocess.run([compiler, "-std=c99", "-O2", str(ROOT / "export_routes.c"),
                    "-o", str(executable), "-lm"], env=env, check=True, timeout=60)
    result = subprocess.run([str(executable)], env=env, check=True,
                            capture_output=True, text=True, timeout=120)
    data = json.loads(result.stdout)
    data["sources"] = [{"path": p.relative_to(REPO).as_posix(),
                        "sha256": hashlib.sha256(p.read_bytes()).hexdigest()} for p in SOURCES]
    data["sourceHash"] = hashlib.sha256(json.dumps(data["sources"]).encode()).hexdigest()[:12]
    data["generatedAt"] = datetime.now(timezone(timedelta(hours=8))).isoformat(timespec="seconds")
    target = ROOT / "routes-data.js"
    temporary = ROOT / "routes-data.js.tmp"
    temporary.write_text("window.ROUTE_DATA=" + json.dumps(data, ensure_ascii=False, separators=(",", ":")) + ";\n", encoding="utf-8")
    temporary.replace(target)
    valid = sum(r is not None for r in data["routes"].values())
    print(f"已导出 {len(data['routes'])} 组路线，{valid} 组可行；源码 {data['sourceHash']}")


if __name__ == "__main__":
    export()
