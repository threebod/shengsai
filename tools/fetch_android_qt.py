"""Download the exact required Qt 6.11.1 Android libraries from the official split repository."""
from pathlib import Path
import concurrent.futures, hashlib, urllib.request, xml.etree.ElementTree as ET
import py7zr

ROOT=Path(__file__).resolve().parents[1]
BASE='https://download.qt.io/online/qtsdkrepository/all_os/android/qt6_6111/qt6_6111_arm64_v8a/'
DEST=ROOT/'.deps/qt-android'
CACHE=ROOT/'.deps/downloads'
DEST.mkdir(parents=True,exist_ok=True);CACHE.mkdir(parents=True,exist_ok=True)
metadata=urllib.request.urlopen(BASE+'Updates.xml',timeout=60).read()
(CACHE/'qt-android-Updates.xml').write_bytes(metadata)
jobs=[]
for package in ET.fromstring(metadata).findall('PackageUpdate'):
    name=package.findtext('Name','')
    if name not in ('qt.qt6.6111.android_arm64_v8a','qt.qt6.6111.addons.qtconnectivity.android_arm64_v8a','qt.qt6.6111.addons.qtshadertools.android_arm64_v8a'):continue
    for archive in package.findtext('DownloadableArchives','').split(','):
        archive=archive.strip()
        if not archive or archive.startswith(('qtdoc-','qttranslations-')):continue
        jobs.append((BASE+name+'/'+package.findtext('Version')+archive,CACHE/archive))

def download(job):
    url,path=job
    digest=urllib.request.urlopen(url+'.sha1',timeout=60).read().decode().strip().split()[0]
    if not path.exists() or hashlib.sha1(path.read_bytes()).hexdigest()!=digest:
        print('Downloading',path.name,flush=True)
        with urllib.request.urlopen(url,timeout=120) as response,path.open('wb') as out:
            while chunk:=response.read(1024*1024):out.write(chunk)
        if hashlib.sha1(path.read_bytes()).hexdigest()!=digest:raise RuntimeError('Qt archive checksum mismatch: '+path.name)
    print('Verified',path.name,path.stat().st_size,flush=True)
    return path

with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
    archives=list(pool.map(download,jobs))
for archive in archives:
    print('Extracting',archive.name,flush=True)
    with py7zr.SevenZipFile(archive) as z:
        for member in z.getnames():
            target=(DEST/member).resolve()
            if not target.is_relative_to(DEST.resolve()):raise RuntimeError('Unsafe archive member')
        z.extractall(DEST)
print('Qt Android installed under',DEST,flush=True)
