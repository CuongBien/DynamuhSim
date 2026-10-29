"""Axis-aligned bottleneck geometry and matching map for isolated planner/controller trials."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

WIDTHS={'wide':.8,'narrow':.5270765814495917}

def fixture(case):
    if case not in WIDTHS:raise ValueError('Unknown declared gap case')
    gap=WIDTHS[case];inner=3.35;length=inner-gap/2
    boxes=[dict(name='west',center=[-30.,0.,1.],size=[.3,7.3,2.]),dict(name='east',center=[-18.,0.,1.],size=[.3,7.3,2.]),dict(name='north',center=[-24.,3.5,1.],size=[12.,.3,2.]),dict(name='south',center=[-24.,-3.5,1.],size=[12.,.3,2.]),dict(name='gate_top',center=[-24.,gap/2+length/2,.75],size=[.4,length,1.5]),dict(name='gate_bottom',center=[-24.,-gap/2-length/2,.75],size=[.4,length,1.5])]
    return dict(schema='static_gap_fixture/1',case=case,gap_width_m=gap,gate_x=-24.,gate_thickness_m=.4,start=[-28.,0.,0.],goal=[-20.,0.,0.],boxes=boxes,map=dict(origin=[-31.,-4.,0.],resolution=.05,width=280,height=160),humans=0)

def map_array(spec):
    meta=spec['map'];res=meta['resolution'];x=meta['origin'][0]+(np.arange(meta['width'])+.5)*res;y=meta['origin'][1]+(np.arange(meta['height'])+.5)*res;grid=np.full((meta['height'],meta['width']),254,dtype='u1')
    for box in spec['boxes']:
        cx,cy,_=box['center'];sx,sy,_=box['size'];inside=(abs(x[None,:]-cx)<=sx/2+1e-10)&(abs(y[:,None]-cy)<=sy/2+1e-10);grid[inside]=0
    return grid

def write_assets(out,spec):
    out=Path(out);grid=map_array(spec);meta=spec['map']
    (out/'gap.pgm').write_bytes(f"P5\n{meta['width']} {meta['height']}\n255\n".encode()+np.flipud(grid).tobytes())
    (out/'gap.yaml').write_text('image: gap.pgm\nmode: trinary\nresolution: 0.05\norigin: [-31.0, -4.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    (out/'gap_fixture.json').write_text(json.dumps(spec,indent=2)+'\n')

def configure_world(tree,spec):
    world=tree.getroot().find('world')
    for child in list(world):
        if child.tag=='actor' or child.tag=='model' and child.get('name')!='common_observer':world.remove(child)
    def box(name,center,size,color):
        model=ET.SubElement(world,'model',name=name);ET.SubElement(model,'static').text='true';ET.SubElement(model,'pose').text=' '.join(map(str,[*center,0,0,0]));link=ET.SubElement(model,'link',name='body')
        for tag in ['collision','visual']:
            shape=ET.SubElement(link,tag,name='body_'+tag);geometry=ET.SubElement(shape,'geometry');ET.SubElement(ET.SubElement(geometry,'box'),'size').text=' '.join(map(str,size))
            if tag=='visual':
                material=ET.SubElement(shape,'material');ET.SubElement(material,'ambient').text=color;ET.SubElement(material,'diffuse').text=color
    box('floor',[-24.,0.,-.05],[14.,8.,.1],'.6 .62 .65 1')
    for b in spec['boxes']:box(b['name'],b['center'],b['size'],'.85 .48 .12 1' if b['name'].startswith('gate_') else '.38 .42 .48 1')
    observer=world.find("model[@name='common_observer']");observer.find('pose').text='-24 -6 8 0 .927295218 1.57079632679'
    for plugin in world.findall('gui/plugin'):
        if plugin.get('filename')=='MinimalScene':plugin.find('camera_pose').text='-29 -5 6 0 .65 .8'
    if world.findall('actor') or world.find("plugin[@name='response_experiment::Adapter']") is not None:raise ValueError('Human adapter leaked into isolated fixture')
    return tree
