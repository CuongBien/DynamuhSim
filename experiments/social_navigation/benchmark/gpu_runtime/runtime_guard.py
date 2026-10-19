"""Scoped GPU preflight and ownership-based cleanup for experimental runners."""
import hashlib,json,os,signal,subprocess,time,uuid
from pathlib import Path
HERE=Path(__file__).resolve().parent
TOKEN='DYNAMUHSIM_EXPERIMENT_SESSION'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def gpu_preflight(runtime):
    runtime=Path(runtime);generated=HERE/'generated';generated.mkdir(exist_ok=True)
    source=HERE/'gl_probe.cpp';binary=generated/'gl_probe';digest=generated/'gl_probe.sha256'
    if not binary.exists() or not digest.exists() or digest.read_text()!=sha(source):
        subprocess.run(['g++','-O2','-std=c++17',str(source),'-lGL','-lX11','-o',str(binary)],check=True,timeout=30)
        digest.write_text(sha(source))
    keys=['DISPLAY','WAYLAND_DISPLAY','LIBGL_ALWAYS_SOFTWARE','LIBGL_ALWAYS_INDIRECT','GALLIUM_DRIVER','MESA_D3D12_DEFAULT_ADAPTER_NAME','MESA_LOADER_DRIVER_OVERRIDE']
    before={k:os.environ[k] for k in keys if k in os.environ}
    if Path('/dev/dxg').exists():
        # This project's WSL launch already chooses NVIDIA. Make the host preflight
        # use the same adapter and remove old software-render troubleshooting flags.
        os.environ['GALLIUM_DRIVER']='d3d12'
        os.environ['MESA_D3D12_DEFAULT_ADAPTER_NAME']='NVIDIA'
        for key in ['LIBGL_ALWAYS_SOFTWARE','LIBGL_ALWAYS_INDIRECT']:os.environ.pop(key,None)
        if os.environ.get('MESA_LOADER_DRIVER_OVERRIDE','').lower() in ['swrast','llvmpipe','softpipe']:os.environ.pop('MESA_LOADER_DRIVER_OVERRIDE')
    result=subprocess.run([str(binary)],capture_output=True,text=True,timeout=15)
    if result.returncode:raise RuntimeError('GPU preflight failed: '+result.stderr.strip())
    info=json.loads(result.stdout);renderer=info['renderer'].lower()
    info.update(environment_before=before,environment_used={k:os.environ[k] for k in keys if k in os.environ},probe_source_sha256=sha(source),probe_binary_sha256=sha(binary))
    info['hardware_verified']=not any(k in renderer for k in ['llvmpipe','softpipe','swrast','software rasterizer'])
    if Path('/dev/dxg').exists():info['hardware_verified'] &= 'nvidia' in renderer
    (runtime/'rendering.json').write_text(json.dumps(info,indent=2))
    if not info['hardware_verified']:raise RuntimeError('Hardware renderer required; detected '+info['renderer'])
    print('GPU renderer: '+info['renderer'],flush=True)
    return info

class RuntimeGuard:
    def __init__(self,runtime):
        self.runtime=Path(runtime);self.token=uuid.uuid4().hex;self.previous=os.environ.get(TOKEN);self.start=time.time()
        os.environ[TOKEN]=self.token
    def members(self):
        found=[]
        for proc in Path('/proc').iterdir():
            if not proc.name.isdigit() or int(proc.name)==os.getpid():continue
            try:
                if proc.stat().st_uid!=os.getuid():continue
                args=(proc/'cmdline').read_bytes()
                if not args:continue
                env=dict(x.split(b'=',1) for x in (proc/'environ').read_bytes().split(b'\0') if b'=' in x)
                if env.get(TOKEN.encode())!=self.token.encode():continue
                found.append(dict(pid=int(proc.name),command=args.replace(b'\0',b' ').decode(errors='replace')))
            except (FileNotFoundError,ProcessLookupError,PermissionError):continue
        return found
    def cleanup(self):
        seen={};signals=[]
        for sig,seconds in [(signal.SIGINT,2.),(signal.SIGTERM,2.),(signal.SIGKILL,1.)]:
            live=self.members()
            if not live:break
            for item in live:
                seen[item['pid']]=item
                # Re-read identity immediately before signalling; no process-name kill.
                if any(p['pid']==item['pid'] for p in self.members()):
                    try:os.kill(item['pid'],sig);signals.append(dict(pid=item['pid'],signal=sig.name))
                    except ProcessLookupError:pass
            deadline=time.monotonic()+seconds
            while time.monotonic()<deadline and self.members():time.sleep(.05)
        remaining=self.members()
        if self.previous is None:os.environ.pop(TOKEN,None)
        else:os.environ[TOKEN]=self.previous
        report=dict(session=self.token,reaped_members=list(seen.values()),signals=signals,remaining=remaining,status='PASS' if not remaining else 'ERROR')
        (self.runtime/'process_cleanup.json').write_text(json.dumps(report,indent=2))
        log=Path.home()/'.gz/rendering/ogre2.log'
        if log.exists() and log.stat().st_mtime>=self.start:
            target=self.runtime/'ogre2_session.log';target.write_bytes(log.read_bytes())
            report['ogre_log_sha256']=sha(target)
            report['ogre_renderer_lines']=[l.strip() for l in target.read_text(errors='replace').splitlines() if 'GL_RENDERER =' in l]
            (self.runtime/'process_cleanup.json').write_text(json.dumps(report,indent=2))
        return report
