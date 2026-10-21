param([string]$QtRoot='E:/Qt/6.11.1/mingw_64',[string]$QtTools='E:/Qt/Tools')
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$env:Path="$QtRoot/bin;$QtTools/mingw1310_64/bin;$env:Path"
& "$QtTools/CMake_64/bin/cmake.exe" -S $root -B "$root/build" -G Ninja "-DCMAKE_MAKE_PROGRAM=$QtTools/Ninja/ninja.exe" "-DCMAKE_PREFIX_PATH=$QtRoot" -DCMAKE_BUILD_TYPE=Release
if($LASTEXITCODE -ne 0){throw 'CMake configure failed'}
& "$QtTools/CMake_64/bin/cmake.exe" --build "$root/build" --parallel 6
if($LASTEXITCODE -ne 0){throw 'Build failed'}
& "$QtTools/CMake_64/bin/ctest.exe" --test-dir "$root/build" --output-on-failure
if($LASTEXITCODE -ne 0){throw 'CTest failed'}
