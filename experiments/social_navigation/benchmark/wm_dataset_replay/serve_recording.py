#!/usr/bin/env python3
"""Serve real recorded camera video locally with browser playback controls."""
from http.server import ThreadingHTTPServer,SimpleHTTPRequestHandler
from functools import partial
from pathlib import Path
import argparse,json,shutil,os
p=Path(__file__).resolve().parent
args=argparse.ArgumentParser();args.add_argument('--port',type=int,default=0);a=args.parse_args()
web=p/'generated/web';web.mkdir(parents=True,exist_ok=True)
shutil.copy2(p/'viewer.html',web/'index.html');shutil.copy2(p/'reports/recorded_camera_4x.webm',web/'recorded_camera_4x.webm')
server=ThreadingHTTPServer(('127.0.0.1',a.port),partial(SimpleHTTPRequestHandler,directory=str(web)))
info={'url':f'http://localhost:{server.server_port}/','pid':os.getpid(),'binding':'127.0.0.1','directory':str(web)};(p/'generated/web_session.json').write_text(json.dumps(info,indent=2));print(json.dumps(info),flush=True)
try:server.serve_forever()
except KeyboardInterrupt:pass
finally:server.server_close()
