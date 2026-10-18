"""Count full throat traversals; lateral bypasses are recorded separately."""
class PassageCounter:
    def __init__(self,states,passage):
        self.p=passage;self.sides={h['id']:self.side(h['x']) for h in states}
        self.inside={h['id']:False for h in states};self.bypass={h['id']:False for h in states};self.events=[]
    def side(self,x):
        return -1 if x<=self.p['left_confirmation_x'] else 1 if x>=self.p['right_confirmation_x'] else 0
    def update(self,time,states):
        added=[]
        for h in states:
            hid=h['id'];side=self.side(h['x']);old=self.sides[hid]
            if self.p['x_min']<=h['x']<=self.p['x_max']:
                self.inside[hid]=True
                if abs(h['y'])>self.p['half_width']+1e-6:self.bypass[hid]=True
            if side:
                if old and side!=old:
                    e=dict(time=time,id=hid,from_side=old,to_side=side,through_channel=self.inside[hid] and not self.bypass[hid],x=h['x'],y=h['y'])
                    self.events.append(e);added.append(e)
                self.sides[hid]=side;self.inside[hid]=False;self.bypass[hid]=False
        return added
    def counts(self):
        return {str(hid):sum(e['id']==hid and e['through_channel'] for e in self.events) for hid in sorted(self.sides)}
