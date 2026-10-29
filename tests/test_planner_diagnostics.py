import base64,json,sys,tempfile,unittest,zlib
from collections import Counter
from pathlib import Path
from types import SimpleNamespace as NS
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.runtime.planner_diagnostics import costmap_row,identity,verify_rows,TOPICS
class NativeCostmapTests(unittest.TestCase):
 def message(self):
  stamp=NS(sec=12,nanosec=345);pose=NS(position=NS(x=-31.,y=-21.,z=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))
  return NS(header=NS(stamp=stamp,frame_id='map'),metadata=NS(size_x=5,size_y=1,resolution=.05,origin=pose,update_time=stamp,layer='master'),data=bytes([0,252,253,254,255]))
 def test_native_payload_keeps_all_byte_values_and_dimensions(self):
  row=costmap_row(self.message());self.assertEqual(zlib.decompress(base64.b64decode(row['data'])),bytes([0,252,253,254,255]));self.assertEqual(row['stamp_ns'],12000000345)
  msg=self.message();msg.metadata.size_y=2
  with self.assertRaises(ValueError):costmap_row(msg)
 def test_bag_identity_detects_changed_origin_and_modified_bytes(self):
  row=costmap_row(self.message());counts=Counter({identity(t,row):1 for t in TOPICS})
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'rows.jsonl';rows=[dict(row,topic=t) for t in TOPICS]
   write=lambda:p.write_text(''.join(json.dumps(v)+'\n' for v in rows))
   write();self.assertTrue(verify_rows(p,counts))
   rows[0]['data']=base64.b64encode(zlib.compress(bytes(5))).decode();write();self.assertFalse(verify_rows(p,counts))
   rows=[dict(costmap_row(self.message()),topic=t) for t in TOPICS];rows[0]['origin']['x']=9.;write();self.assertFalse(verify_rows(p,counts))
if __name__=='__main__':unittest.main()
