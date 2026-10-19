#!/usr/bin/env python3
"""Fetch SHA256-pinned official AMCL 1.3.13 reference files, only into generated/."""
import hashlib,json,urllib.request
from pathlib import Path
HERE=Path(__file__).resolve().parent;DEST=HERE/'generated/reference'
BASE='https://raw.githubusercontent.com/ros-navigation/navigation2/1.3.13/nav2_amcl/'
REFERENCES=[{'relative_source': 'src/sensors/laser/likelihood_field_model.cpp', 'file': 'likelihood_field_model.cpp', 'sha256': '84a4542b6891cd665139fcc75dc6b71fa0d2175f167de5c11df463d2f6fe101d'}, {'relative_source': 'src/amcl_node.cpp', 'file': 'amcl_node.cpp', 'sha256': '15afd544e5cbbcb16c09090bb41ebff65cbaa47793d6deb1033a4e1ec61771b7'}, {'relative_source': 'include/nav2_amcl/map/map.hpp', 'file': 'map.hpp', 'sha256': '4a52678c51e088b4ccccc32c1176f6698dbc584ee2e789fb227533955a375d7d'}, {'relative_source': 'src/map/map_cspace.cpp', 'file': 'map_cspace.cpp', 'sha256': 'e77abdeb1f20750de8d0a09027fe71479dba164970570773249bea41292a48ee'}, {'relative_source': 'src/pf/pf.c', 'file': 'pf.c', 'sha256': '6747b6f528bd1e3727ebab79eece7ea52379737127ad4431276901ac88d82b6e'}, {'relative_source': 'include/nav2_amcl/sensors/laser/laser.hpp', 'file': 'include/nav2_amcl/sensors/laser/laser.hpp', 'sha256': '602056f75249a084d3e0f26040114e74351bbacfdc2bcfb424339663be07a0db'}, {'relative_source': 'include/nav2_amcl/pf/pf.hpp', 'file': 'include/nav2_amcl/pf/pf.hpp', 'sha256': 'e3a5f7c84e1ae290faa56ff386ffc151c5c071d52421279bebdec34f1296659a'}, {'relative_source': 'include/nav2_amcl/pf/pf_vector.hpp', 'file': 'include/nav2_amcl/pf/pf_vector.hpp', 'sha256': '1cafc26520ff368848614d6291094c55d83be9f7832516a23cea25d1cd45539f'}, {'relative_source': 'include/nav2_amcl/pf/pf_pdf.hpp', 'file': 'include/nav2_amcl/pf/pf_pdf.hpp', 'sha256': '228e4a9bb6fe5ba3bd59879106110bc3721fdec3cab6a5f0e2776f60cb80542c'}, {'relative_source': 'include/nav2_amcl/pf/pf_kdtree.hpp', 'file': 'include/nav2_amcl/pf/pf_kdtree.hpp', 'sha256': '2cd9157e9570afe56522f1565757d40df813db39f8fb548c5175a3795c624fbd'}]
DEST.mkdir(parents=True,exist_ok=True);manifest=[]
for item in REFERENCES:
    target=DEST/item['file'];url=BASE+item['relative_source']
    data=target.read_bytes() if target.exists() else urllib.request.urlopen(url,timeout=30).read()
    assert hashlib.sha256(data).hexdigest()==item['sha256'],url+' SHA256 mismatch'
    if not target.exists():target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
    manifest.append(dict(url=url,file=str(target),sha256=item['sha256']))
(DEST/'manifest.json').write_text(json.dumps(manifest,indent=2));print('Verified',len(manifest),'official reference files')
