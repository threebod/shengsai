$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$viewer = Join-Path $repo 'route_viewer'
$env:PYTHONIOENCODING = 'utf-8'
& python "$viewer/export_routes.py"
if ($LASTEXITCODE -ne 0) { throw 'Route export failed; no firmware or hardware was changed.' }
Start-Process (Join-Path $viewer 'index.html')
