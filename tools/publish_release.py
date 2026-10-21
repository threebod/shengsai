"""Publish the explicitly authorized repository/release using the user's Git credential manager.

Credentials are kept in memory and sent only to api.github.com/uploads.github.com.
No credential values are written to files or printed.
"""
from pathlib import Path
import json,subprocess,urllib.request,urllib.error,urllib.parse,sys
ROOT=Path(__file__).resolve().parents[1]
REPO='threebod/shengsai'
TAG='v0.1.0-preview.1'

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
mode=sys.argv[1]
if mode=='check':
    user=api('/user');print('Authenticated GitHub user:',user['login'])
    if user['login']!='threebod':raise RuntimeError('Unexpected GitHub account')
elif mode=='create':
    user=api('/user')
    if user['login']!='threebod':raise RuntimeError('Unexpected GitHub account')
    result=api('/user/repos',{'name':'shengsai','description':'省赛 STM32F407 比赛固件与 Qt Quick Windows/Android 控制台','private':False,'auto_init':False})
    print('Created',result['html_url'])
elif mode=='release':
    body=(ROOT/'docs/release-notes.md').read_text(encoding='utf-8')
    result=api('/repos/'+REPO+'/releases',{'tag_name':TAG,'name':'省赛控制台 '+TAG,'body':body,'draft':True,'prerelease':True})
    upload=result['upload_url'].split('{')[0]
    for path in sorted((ROOT/'dist').iterdir()):
        if path.is_file():
            content_type={'.apk':'application/vnd.android.package-archive','.zip':'application/zip','.png':'image/png','.md':'text/markdown'}.get(path.suffix,'text/plain')
            asset=api(upload+'?name='+urllib.parse.quote(path.name),path.read_bytes(),'POST',content_type)
            print('Uploaded',asset['name'],asset['size'])
    result=api('/repos/'+REPO+'/releases/'+str(result['id']),{'draft':False},'PATCH')
    print('Release',result['html_url'])
else:raise RuntimeError('Expected check/create/release')
