# 构建与发布

## Windows

使用 Qt 6.11.1 MinGW x64、Quick、QuickControls2、SerialPort、Test 和 CMake/Ninja。

```powershell
./tools/build-windows.ps1 -QtRoot E:/Qt/6.11.1/mingw_64 -QtTools E:/Qt/Tools
./tools/package-windows.ps1
```

脚本默认路径对应本机环境，可传入自己的Qt安装路径。CMake源码和固件不依赖旧项目目录。

## Android

使用同版本Qt Android arm64、NDK r27c、JDK21、Android API36、Gradle9.3.1，最低运行版本Android9，targetSdk35。Qt要求API36编译，但不要求手机必须运行Android16。

本次所有新增工具、SDK、NDK、Gradle缓存及临时文件均位于E盘。`.deps/`不会提交，已有JDK从E盘Android Studio复用。

Qt官方6.11.1归档已拆分为`all_os/android/...`，旧aqt安装器不能直接定位，因此提供`tools/fetch_android_qt.py`读取官方清单、核对官方归档校验并提取所需库。该脚本需要py7zr。`fetch_ndk.py`下载Google NDK；`fetch_android_platform.py`读取Google官方包清单下载API36，并复用现有E盘SDK工具和许可证到工程私有目录。

```powershell
./tools/build-android.ps1
python tools/sign_android.py
```

构建下载目录或依赖工具的位置需按自己的环境准备；不要将`.deps`或本机绝对路径生成文件提交。

签名脚本首次生成`.deps/signing/shengsai-release.jks`及私有配置，后续复用相同签名。请私下备份整个签名目录；不要上传密钥或其配置。发布APK前执行apksigner验证及16KB zipalign校验。

## 固件

Keil MDK：`firmware/MecanumJog.uvprojx`，ARMCC 5.06 update7。

```powershell
& E:/keil_v5/UV4/UV4.exe -r firmware/MecanumJog.uvprojx -o firmware/build.log
```

检查构建日志中的实际错误数。静态调用图在`firmware/Objects/MecanumJog.htm`；新工程栈为4KB。固件构建不等于已经烧录或完成硬件验证。

## GitHub

`tools/publish_release.py`仅用于用户明确授权的`threebod/shengsai`及`v0.1.0-preview.1`，通过已有Git凭据管理器认证，凭据仅在内存中使用，不输出或保存。代码先本地提交、普通推送；发布脚本只上传`dist/`根目录的发布文件，不上传依赖或签名材料。
