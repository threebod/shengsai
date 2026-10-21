from pathlib import Path
import urllib.request,xml.etree.ElementTree as ET,hashlib,zipfile,shutil
root=Path(__file__).resolve().parents[1]/'.deps'
sdk=root/'android-sdk';sdk.mkdir(parents=True,exist_ok=True)
old=Path(r'E:/Android/sdk')
for folder in ['licenses','platform-tools','build-tools/36.1.0']:
    shutil.copytree(old/folder,sdk/folder,dirs_exist_ok=True)
url='https://dl.google.com/android/repository/'
metadata=urllib.request.urlopen(url+'repository2-3.xml',timeout=60).read()
tree=ET.fromstring(metadata)
for element in tree.iter():element.tag=element.tag.split('}')[-1]
package=next(p for p in tree.iter('remotePackage') if p.get('path')=='platforms;android-36')
complete=package.find('archives/archive/complete')
name=complete.findtext('url');digest=complete.findtext('checksum')
archive=root/'downloads'/name
if not archive.exists() or hashlib.sha1(archive.read_bytes()).hexdigest()!=digest:
    print('Downloading Android 36:',name,flush=True)
    with urllib.request.urlopen(url+name,timeout=120) as response,archive.open('wb') as out:
        while chunk:=response.read(1024*1024):out.write(chunk)
if hashlib.sha1(archive.read_bytes()).hexdigest()!=digest:raise RuntimeError('Platform checksum mismatch')
target=sdk/'platforms/android-36';target.mkdir(parents=True,exist_ok=True)
with zipfile.ZipFile(archive) as z:
    for member in z.infolist():
        parts=Path(member.filename).parts
        if len(parts)<2:continue
        path=(target/Path(*parts[1:])).resolve()
        if not path.is_relative_to(target.resolve()):raise RuntimeError('Unsafe archive member')
        if member.is_dir():path.mkdir(parents=True,exist_ok=True)
        else:
            path.parent.mkdir(parents=True,exist_ok=True)
            with z.open(member) as src,path.open('wb') as dst:shutil.copyfileobj(src,dst)
print('SDK ready:',sdk,flush=True)
