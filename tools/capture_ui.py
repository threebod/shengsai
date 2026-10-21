from pathlib import Path
import os,subprocess
root=Path(__file__).resolve().parents[1]
env=os.environ.copy();env['PATH']=r'E:/Qt/6.11.1/mingw_64/bin;E:/Qt/Tools/mingw1310_64/bin;'+env['PATH']
env['QT_FORCE_STDERR_LOGGING']='1'
exe=root/'build/upper_computer/shengsai_tuner.exe'
for mobile in [False,True]:
    for page in range(4):
        output=root/'docs/design'/f'{"mobile" if mobile else "desktop"}-page-{page+1}.png'
        command=[str(exe),'--page',str(page),'--capture',str(output)]
        if mobile:command.append('--mobile')
        result=subprocess.run(command,env=env,capture_output=True,text=True,timeout=20)
        if result.returncode or result.stderr:raise RuntimeError(f'UI capture failed: {result.returncode} {result.stderr}')
        print('Captured',output.name,flush=True)
