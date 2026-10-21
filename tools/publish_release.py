"""Publish the explicitly authorized repository/release using the user's Git credential manager.

Credentials are kept in memory and sent only to api.github.com/uploads.github.com.
No credential values are written to files or printed.
"""
from pathlib import Path
import json,subprocess,urllib.request,urllib.error,urllib.parse,argparse,hashlib
ROOT=Path(__file__).resolve().parents[1]
REPO='threebod/shengsai'
parser=argparse.ArgumentParser()
parser.add_argument('mode',choices=['check','create','release'])
parser.add_argument('--tag',choices=['v0.1.0-preview.1','v0.1.0-preview.2'],default='v0.1.0-preview.1')
args=parser.parse_args()
TAG=args.tag

def credential():
    result=subprocess.run(['git','credential','fill'],input='protocol=https\nhost=github.com\n\n',text=True,capture_output=True,check=True)
    fields=dict(line.split('=',1) for line in result.stdout.splitlines() if '=' in line)
    if not fields.get('password'):raise RuntimeError('No existing GitHub credential')
    return fields['password']

def api(path,data=None,method=None,content_type='application/json'):
    url=path if path.startswith('https://') else 'https://api.github.com'+path
    if urllib.parse.urlparse(url).hostname not in ('api.github.com','uploads.github.com'):raise RuntimeError('Unexpected API host')
    body=data if isinstance(data,bytes) else json.dumps(data).encode() if data is not None else None
    request=urllib.request.Request(url,data=body,method=method,headers={'Authorization':'Bearer '+TOKEN,'Accept':'application/vnd.github+json','X-GitHub-Api-Version':'2022-11-28','User-Agent':'Shengsai-release','Content-Type':content_type})
    with urllib.request.urlopen(request,timeout=180) as response:return json.load(response)

TOKEN=credential()
mode=args.mode
if mode=='check':
    user=api('/user');print('Authenticated GitHub user:',user['login'])
    if user['login']!='threebod':raise RuntimeError('Unexpected GitHub account')
elif mode=='create':
    user=api('/user')
    if user['login']!='threebod':raise RuntimeError('Unexpected GitHub account')
    result=api('/user/repos',{'name':'shengsai','description':'省赛 STM32F407 比赛固件与 Qt Quick Windows/Android 控制台','private':False,'auto_init':False})
    print('Created',result['html_url'])
elif mode=='release':
    notes='release-notes.md' if TAG=='v0.1.0-preview.1' else 'release-notes-v0.1.0-preview.2.md'
    assets=ROOT/'dist' if TAG=='v0.1.0-preview.1' else ROOT/'dist'/TAG
    paths=sorted(path for path in assets.iterdir() if path.is_file())
    if TAG=='v0.1.0-preview.2':
        expected={'shengsai-v0.1.0-preview.2-android-arm64.apk','SHA256SUMS.txt','VALIDATION.md','mobile-safe-area.png','landscape-safe-area.png'}
        if {path.name for path in paths}!=expected:raise RuntimeError('Unexpected preview.2 release assets')
    body=(ROOT/'docs'/notes).read_text(encoding='utf-8')
    result=api('/repos/'+REPO+'/releases',{'tag_name':TAG,'name':'省赛控制台 '+TAG,'body':body,'draft':True,'prerelease':True})
    upload=result['upload_url'].split('{')[0]
    for path in paths:
        content_type={'.apk':'application/vnd.android.package-archive','.zip':'application/zip','.png':'image/png','.md':'text/markdown'}.get(path.suffix,'text/plain')
        data=path.read_bytes()
        asset=api(upload+'?name='+urllib.parse.quote(path.name),data,'POST',content_type)
        if asset['size']!=len(data) or asset.get('digest') not in (None,'sha256:'+hashlib.sha256(data).hexdigest()):
            raise RuntimeError('Uploaded asset verification failed: '+path.name)
        print('Uploaded',asset['name'],asset['size'])
    result=api('/repos/'+REPO+'/releases/'+str(result['id']),{'draft':False},'PATCH')
    print('Release',result['html_url'])
else:raise RuntimeError('Expected check/create/release')
