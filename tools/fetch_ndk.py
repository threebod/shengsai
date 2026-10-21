from pathlib import Path
import urllib.request,zipfile
root=Path(__file__).resolve().parents[1]/'.deps'
archive=root/'downloads/android-ndk-r27c-windows.zip'
archive.parent.mkdir(parents=True,exist_ok=True)
if not archive.exists():
    print('Downloading official Android NDK r27c to E-drive workspace',flush=True)
    with urllib.request.urlopen('https://dl.google.com/android/repository/android-ndk-r27c-windows.zip',timeout=120) as r,archive.open('wb') as f:
        while block:=r.read(4*1024*1024):f.write(block)
with zipfile.ZipFile(archive) as z:
    for member in z.namelist():
        if not (root/member).resolve().is_relative_to(root.resolve()):raise RuntimeError('Unsafe archive member')
    print('Extracting Android NDK',archive.stat().st_size,flush=True)
    z.extractall(root)
print('NDK ready:',root/'android-ndk-r27c',flush=True)
