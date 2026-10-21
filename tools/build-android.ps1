param([string]$QtHost='E:/Qt/6.11.1/mingw_64',[string]$JavaHome='E:/Program Files/Android/Android Studio/jbr',[string]$QtTools='E:/Qt/Tools')
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$env:Path="$(Convert-Path "$QtHost/bin");$(Convert-Path "$QtTools/mingw1310_64/bin");$env:Path"
$env:JAVA_HOME=$JavaHome
$env:ANDROID_SDK_ROOT="$root/.deps/android-sdk"
$env:ANDROID_NDK_ROOT="$root/.deps/android-ndk-r27c"
$env:GRADLE_USER_HOME="$root/.deps/gradle"
$env:ANDROID_USER_HOME="$root/.deps/android-user"
$env:TEMP="$root/.deps/tmp"
$env:TMP=$env:TEMP
New-Item -ItemType Directory -Force -Path $env:TEMP | Out-Null
& "$QtTools/CMake_64/bin/cmake.exe" -S $root -B "$root/build-android" -G Ninja "-DCMAKE_MAKE_PROGRAM=$QtTools/Ninja/ninja.exe" "-DCMAKE_TOOLCHAIN_FILE=$root/.deps/qt-android/lib/cmake/Qt6/qt.toolchain.cmake" "-DQT_HOST_PATH=$QtHost" "-DANDROID_SDK_ROOT=$env:ANDROID_SDK_ROOT" "-DANDROID_NDK_ROOT=$env:ANDROID_NDK_ROOT" -DANDROID_ABI=arm64-v8a -DANDROID_PLATFORM=android-28 -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
if($LASTEXITCODE -ne 0){throw 'Android configure failed'}
& "$QtTools/CMake_64/bin/cmake.exe" --build "$root/build-android" --target apk --parallel 6
if($LASTEXITCODE -ne 0){throw 'Android build failed'}
