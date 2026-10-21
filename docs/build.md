# 构建与发布

## Windows

使用 Qt 6.11.1 MinGW x64、Quick、QuickControls2、SerialPort、Test 和 CMake/Ninja。

```powershell
./tools/build-windows.ps1 -QtRoot E:/Qt/6.11.1/mingw_64 -QtTools E:/Qt/Tools
./tools/package-windows.ps1
```

脚本默认路径对应本机环境，可传入自己的Qt安装路径。CMake源码和固件不依赖旧项目目录。

## Android

使用 Qt 6.11.1 Android arm64、同版本桌面 Qt 构建工具、NDK r27c（27.2.12479018）、JDK21、Android API36、SDK Build Tools37.0.0、Gradle9.3.1。最低运行版本Android9，targetSdk35；API36是编译要求，不要求手机必须运行Android16。只有桌面版 Qt 的 MinGW Kit 不能生成安卓 APK。

本次所有新增工具、SDK、NDK、Gradle缓存及临时文件均位于E盘。`.deps/`不会提交，已有JDK从E盘Android Studio复用。

Qt官方6.11.1归档已拆分为`all_os/android/...`，旧aqt安装器不能直接定位，因此提供`tools/fetch_android_qt.py`读取官方清单、核对官方归档校验并提取所需库。该脚本需要py7zr。`fetch_ndk.py`下载Google NDK；`fetch_android_platform.py`读取Google官方包清单下载API36，并复用现有E盘SDK工具和许可证到工程私有目录。

本机依赖已准备好，在 PowerShell 中执行：

```powershell
Set-Location E:/Download/project/gongchuang/shengsai
./tools/build-android.ps1
python ./tools/sign_android.py
```

第一步脚本使用项目内 `.deps/qt-android`、`.deps/android-sdk` 和 `.deps/android-ndk-r27c`，配置 Release / arm64-v8a 并构建 `apk` 目标。Qt 的 `androiddeployqt` 收集 Qt/QML 依赖，再由 Gradle 生成未签名安装包；只编译 `shengsai_tuner` 原生目标不会更新 APK。第二步进行 16KB 对齐、签名、签名验证和对齐验证，并生成 SHA256 校验文件。

当前输出：

- 可直接安装：`dist/shengsai-v0.1.0-preview.2-android-arm64.apk`。
- 校验值：同目录同名 `.apk.sha256` 文件。
- 中间未签名文件：`build-android/upper_computer/android-build/shengsai_tuner.apk`，不要将它作为最终安装包分发。

Qt/JDK 不在默认位置时，用脚本参数指定；签名也需使用同一 JDK：

```powershell
$env:JAVA_HOME = 'E:/Program Files/Android/Android Studio/jbr'
./tools/build-android.ps1 -QtHost E:/Qt/6.11.1/mingw_64 -QtTools E:/Qt/Tools -JavaHome $env:JAVA_HOME
python ./tools/sign_android.py
```

每次正式分发新构建前，修改 `upper_computer/CMakeLists.txt` 中的 `QT_ANDROID_VERSION_NAME`，并递增 `QT_ANDROID_VERSION_CODE`；当前分别为 `0.1.0-preview.2`、`2`。签名脚本从生成后的 Manifest 读取版本号命名输出，不会覆盖旧版本包。

构建下载目录或依赖工具的位置需按自己的环境准备；不要将`.deps`或本机绝对路径生成文件提交。

签名脚本首次生成`.deps/signing/shengsai-release.jks`及私有配置，后续复用相同签名。请私下备份整个签名目录；不要上传密钥或其配置。换电脑或重新克隆仓库时，依赖和签名材料都不会从 Git 恢复：需另行准备工具链，并恢复原签名目录，才能继续覆盖安装同一个应用。不要通过删除签名目录来解决构建问题。

将最终签名 APK 传到手机，点开并允许对应来源安装即可；与旧版签名一致时可覆盖安装，无须先卸载。也可以在启用 USB 调试并授权电脑后手动安装：

```powershell
./.deps/android-sdk/platform-tools/adb.exe install -r ./dist/shengsai-v0.1.0-preview.2-android-arm64.apk
```

若使用 Qt Creator，先通过 Qt 安装器安装并配置 Qt 6.11.1 Android arm64 Kit，再打开本仓库根目录 `CMakeLists.txt`，选择该 Kit 和 Release 配置，在“项目 → 构建设置 → Build Android APK”中构建 APK。SDK/JDK/NDK 使用上述 E 盘路径。本仓库 `.deps/qt-android` 为命令行构建准备的归档，未验证其 Qt Creator 自动注册；当前机器直接使用脚本最方便。最终对外分发仍推荐使用本仓库脚本，以保持版本、签名和校验步骤一致。参考 [Qt APK 打包说明](https://doc.qt.io/qtcreator/creator-how-to-package-android-apps.html)及 [androiddeployqt](https://doc.qt.io/qt-6/android-deploy-qt-tool.html)。

## 固件

Keil MDK：`firmware/MecanumJog.uvprojx`，ARMCC 5.06 update7。

```powershell
& E:/keil_v5/UV4/UV4.exe -r firmware/MecanumJog.uvprojx -o firmware/build.log
```

检查构建日志中的实际错误数。静态调用图在`firmware/Objects/MecanumJog.htm`；新工程栈为4KB。固件构建不等于已经烧录或完成硬件验证。

## GitHub

`tools/publish_release.py`仅用于用户明确授权的`threebod/shengsai`及`v0.1.0-preview.1`，通过已有Git凭据管理器认证，凭据仅在内存中使用，不输出或保存。代码先本地提交、普通推送；发布脚本只上传`dist/`根目录的发布文件，不上传依赖或签名材料。
