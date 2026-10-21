"""Sign the internal preview APK; keep the reusable release key outside Git."""
from pathlib import Path
import os,json,secrets,subprocess,hashlib
import xml.etree.ElementTree as ET
root=Path(__file__).resolve().parents[1]
signing=root/'.deps/signing';signing.mkdir(parents=True,exist_ok=True)
metadata=signing/'signing.json';keystore=signing/'shengsai-release.jks'
java=Path(os.environ.get('JAVA_HOME',r'E:/Program Files/Android/Android Studio/jbr'))
sdk=root/'.deps/android-sdk/build-tools/37.0.0'
env=os.environ.copy();env['JAVA_HOME']=str(java)
if not metadata.exists():
    if keystore.exists():raise RuntimeError('Existing key found without its saved configuration')
    metadata.write_text(json.dumps({'password':secrets.token_urlsafe(32),'alias':'shengsai'}),encoding='utf-8')
config=json.loads(metadata.read_text(encoding='utf-8'));env['SS_KEYSTORE_PASSWORD']=config['password']
if not keystore.exists():
    subprocess.run([str(java/'bin/keytool.exe'),'-genkeypair','-keystore',str(keystore),'-storetype','JKS','-alias',config['alias'],'-keyalg','RSA','-keysize','3072','-validity','10000','-dname','CN=Shengsai, OU=Robotics','-storepass:env','SS_KEYSTORE_PASSWORD','-keypass:env','SS_KEYSTORE_PASSWORD'],env=env,check=True,capture_output=True)
dist=root/'dist';dist.mkdir(exist_ok=True)
source=root/'build-android/upper_computer/android-build/shengsai_tuner.apk'
aligned=root/'build-android/upper_computer/android-build/shengsai-aligned.apk'
manifest=ET.parse(source.parent/'AndroidManifest.xml').getroot()
version=manifest.attrib['{http://schemas.android.com/apk/res/android}versionName']
destination=dist/f'shengsai-v{version}-android-arm64.apk'
subprocess.run([str(sdk/'zipalign.exe'),'-P','16','-f','4',str(source),str(aligned)],check=True)
sign=[str(java/'bin/java.exe'),'-jar',str(sdk/'lib/apksigner.jar'),'sign','--ks',str(keystore),'--ks-key-alias',config['alias'],'--ks-pass','env:SS_KEYSTORE_PASSWORD','--key-pass','env:SS_KEYSTORE_PASSWORD','--v4-signing-enabled','false','--out',str(destination),str(aligned)]
subprocess.run(sign,env=env,check=True)
destination.with_suffix('.apk.idsig').unlink(missing_ok=True)
subprocess.run([str(java/'bin/java.exe'),'-jar',str(sdk/'lib/apksigner.jar'),'verify','--verbose','--print-certs',str(destination)],check=True)
subprocess.run([str(sdk/'zipalign.exe'),'-c','-P','16','4',str(destination)],check=True)
destination.with_suffix('.apk.sha256').write_text(hashlib.sha256(destination.read_bytes()).hexdigest()+'  '+destination.name+'\n',encoding='utf-8')
print('Signed APK:',destination)
print('Reusable signing material stays in the ignored .deps/signing directory; back it up privately.')
