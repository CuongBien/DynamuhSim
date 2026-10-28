"""Public author checkpoint download to experiment-local storage, with content hash."""
from pathlib import Path
import hashlib,urllib.request
HERE=Path(__file__).resolve().parent
p=HERE/'generated/ckpt_jrdb_ann_dr_spaam_e20.pth';p.parent.mkdir(exist_ok=True)
expected='1b62e50d9bcb954975bda28610e09921e8bde94216b0a291a704a579690b6324'
if not p.exists():
 with urllib.request.urlopen('https://drive.usercontent.google.com/download?id=1gH_pdxUC8diIFUe3_Ky8QiVVCyiR3500&export=download&confirm=t',timeout=60) as response,p.open('xb') as output:
  while chunk:=response.read(1024*1024):output.write(chunk)
assert hashlib.sha256(p.read_bytes()).hexdigest()==expected,'Checkpoint mismatch; preserve file and inspect'
print('Author checkpoint verified',p)
