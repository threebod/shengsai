param([string]$QtRoot='E:/Qt/6.11.1/mingw_64',[string]$QtSource='E:/Qt/6.11.1/Src/qtbase',[string]$QtTools='E:/Qt/Tools')
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$package="$root/dist/windows"
New-Item -ItemType Directory -Force -Path $package | Out-Null
Copy-Item -LiteralPath "$root/build/upper_computer/shengsai_tuner.exe" -Destination $package
$env:Path="$QtRoot/bin;$QtTools/mingw1310_64/bin;$env:Path"
& "$QtRoot/bin/windeployqt.exe" --release --no-translations --qmldir "$root/upper_computer" "$package/shengsai_tuner.exe"
if($LASTEXITCODE -ne 0){throw 'Qt deployment failed'}
Copy-Item -LiteralPath "$root/docs/release-notes.md" -Destination "$package/README.md"
New-Item -ItemType Directory -Force -Path "$package/licenses" | Out-Null
foreach($name in 'LGPL-3.0-only.txt','GPL-3.0-only.txt','Qt-GPL-exception-1.0.txt') {
    if(Test-Path -LiteralPath "$QtSource/LICENSES/$name") {Copy-Item -LiteralPath "$QtSource/LICENSES/$name" -Destination "$package/licenses"}
}
Compress-Archive -LiteralPath $package -DestinationPath "$root/dist/shengsai-v0.1.0-preview.1-windows-x64.zip" -Force
