param([int]$Port=8765)
$ErrorActionPreference='Stop'
$env:PYTHONIOENCODING='utf-8'
$repo=Split-Path -Parent $PSScriptRoot
$viewer=Join-Path $repo 'action_viewer'
if(!(Test-Path -LiteralPath "$viewer/node_modules/three/build/three.module.js")) {
    $env:npm_config_cache=Join-Path $repo '.deps/npm-cache'
    & npm install --prefix $viewer --no-audit --no-fund
    if($LASTEXITCODE -ne 0){throw 'Three.js installation failed; dependencies stay inside the E-drive project.'}
}
& python "$viewer/server.py" --port $Port
if($LASTEXITCODE -ne 0){throw 'Action viewer failed to start'}
