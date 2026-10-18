"""Centerline-crossing counter with a +/-0.2 m confirmation band."""
class CrossingCounter:
    def __init__(self,humans,band=.2):
        self.band=band;self.sides={h['id']:self.side(h['y']) for h in humans};self.events=[]
    def side(self,y):return 1 if y>=self.band else -1 if y<=-self.band else 0
    def update(self,time,humans):
        added=[]
        for h in humans:
            side=self.side(h['y']);previous=self.sides.get(h['id'],0)
            if side and previous and side!=previous:
                event=dict(time=time,id=h['id'],from_side=previous,to_side=side,x=h['x'],y=h['y'])
                self.events.append(event);added.append(event)
            if side:self.sides[h['id']]=side
        return added
    def counts(self):return {str(hid):sum(e['id']==hid for e in self.events) for hid in sorted(self.sides)}
