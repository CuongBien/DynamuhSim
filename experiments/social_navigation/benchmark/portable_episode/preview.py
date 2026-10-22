#!/usr/bin/env python3
"""Regenerate an honest data figure: measurement time differs from sample anchor."""
import argparse,json
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from portable_loader import Episode

def preview(root,out):
    e=Episode(root);x=e.observation(0);y=e.supervision(0);fields=x['observations'];anchor=int(fields['anchor_ns']);depth_stamp=int(fields['depth_stamp_ns'][-1])
    board=Image.new('RGB',(1220,590),'white');draw=ImageDraw.Draw(board);font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',16);title=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',21)
    draw.text((15,10),f'Actual recorded sample0: anchor {anchor/1e9:.1f}s, latest available depth {depth_stamp/1e9:.1f}s',font=title,fill='black')
    draw.text((15,45),'20 history points at10Hz | labels30steps /3s after anchor | no model trained',font=font,fill='black')
    d=x['depth_history'][-1];gray=((np.clip(np.nan_to_num(d,nan=5.,posinf=5.,neginf=.28),.28,5)-.28)/(5-.28)*220+15).astype(np.uint8);rgb=np.repeat(gray[:,:,None],3,axis=2);rgb[~np.isfinite(d)]=[110,80,135]
    draw.text((15,90),'INPUT: measured depth, age100ms',font=font,fill='black');board.paste(Image.fromarray(rgb),(15,125))
    b=x['current_bev'];colors=np.empty((64,64,3),dtype=np.uint8);colors[b==-1]=[80,80,80];colors[b==0]=[235,235,235];colors[b==1]=[220,70,40]
    draw.text((470,90),'INPUT: BEV from that depth',font=font,fill='black');board.paste(Image.fromarray(colors[::-1,::-1]).resize((320,320),Image.Resampling.NEAREST),(470,125))
    draw.text((825,90),'LABEL ONLY: human GT3s',font=font,fill='black');draw.rectangle((825,125,1195,445),outline='#aaaaaa')
    palette=['#c73737','#308c48','#395bba','#a96617','#913cb5','#158b9b'];points=y['human_xy_future_m']
    for j,ident in enumerate(y['human_ids']):
        line=[]
        for point in points[:,j,:]:
            px=825+(3.2-point[1])/6.4*370;py=445-point[0]/6.4*320
            if 825<=px<=1195 and 125<=py<=445:line.append((px,py))
        if line:
            if len(line)>1:draw.line(line,fill=palette[j],width=3)
            px,py=line[0];draw.ellipse((px-4,py-4,px+4,py+4),fill=palette[j]);draw.text((px+5,py),str(ident),font=font,fill=palette[j])
    draw.text((15,465),'BEV is local at depth measurement time, without ego-motion compensation to anchor.',font=font,fill='black')
    draw.text((15,495),'Future XY is simulator supervision, not a prediction and not part of observations.',font=font,fill='black')
    draw.text((15,525),'BEV: unknown gray / free white / occupied red. Purple depth pixels are invalid.',font=font,fill='black')
    draw.text((15,555),'H1-H5 unknown; association/future BEV supervision disabled. Pilot only, not train-ready.',font=font,fill='black');board.save(out)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();preview(a.bundle,a.output)
