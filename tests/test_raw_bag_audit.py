"""Independent raw order, missing-data semantics and interval calculations."""
from pathlib import Path
import struct
import sys
import tempfile
import unittest
import zlib

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.analysis.mcap_records import MAGIC,McapFile,McapError
from dynamuhsim.analysis.raw_bag import timing,intersection,merge,duration,velocities


def record(code,payload):return struct.pack('<BQ',code,len(payload))+payload
def string(s):return struct.pack('<I',len(s))+s.encode()


def synthetic(corrupt_crc=False):
    schema=record(3,struct.pack('<H',1)+string('std_msgs/msg/String')+string('ros2msg')+struct.pack('<I',11)+b'string data')
    channel=record(4,struct.pack('<HH',1,1)+string('/test')+string('cdr')+struct.pack('<I',0))
    def message(t):return record(5,struct.pack('<HIQQ',1,0,t,1_700_000_000_000_000_000+t)+b'payload')
    content=schema+channel+message(20)+message(10)
    crc=zlib.crc32(content) ^ int(corrupt_crc)
    chunk=record(6,struct.pack('<QQQI',10,20,len(content),crc)+string('')+struct.pack('<Q',len(content))+content)
    return MAGIC+record(1,string('ros2')+string('test'))+chunk+record(15,struct.pack('<I',0))+record(2,struct.pack('<QQI',0,0,0))+MAGIC


class AuditTests(unittest.TestCase):
    def test_preserves_write_order_and_both_time_domains(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'test.mcap';path.write_bytes(synthetic());reader=McapFile(path);rows=list(reader.messages())
            self.assertEqual([r['record_ns'] for r in rows],[20,10])
            self.assertGreater(rows[0]['publish_ns'],10**17)
            self.assertEqual(reader.integrity['chunk_crc_verified'],1)
            self.assertEqual(timing([r['record_ns'] for r in rows])['regressions'],1)

    def test_corrupt_crc_and_truncation_are_failures(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'test.mcap';path.write_bytes(synthetic(True))
            with self.assertRaisesRegex(McapError,'CRC mismatch'):list(McapFile(path).messages())
            path.write_bytes(synthetic()[:-5])
            with self.assertRaises(McapError):list(McapFile(path).messages())

    def test_missing_is_not_zero_and_duplicates_not_infinite_hz(self):
        self.assertEqual(timing([]),dict(status='NOT_MEASURED'))
        result=timing([0,100000000,100000000,400000000],.1)
        self.assertEqual(result['duplicate_values'],1)
        self.assertEqual(result['inferred_missing_from_period'],2)
        self.assertAlmostEqual(result['frequency_hz']['max'],10.)
        self.assertEqual(result['actual_missing_messages'],'NOT_MEASURED')

    def test_history_future_erosion_and_gap_union(self):
        coverage=intersection([[0,10],[12,20]],[[2,18]])
        self.assertEqual(coverage,[[2.,10.],[12.,18.]])
        centers=[[a+2,b-3] for a,b in coverage if b-a>=5]
        self.assertEqual(duration(centers),4.)

    def test_velocity_uses_elapsed_source_time_and_deduplicates(self):
        rows=[dict(time=t,x=2*t,y=0.) for t in [0,.1,.2,.2,.3,.4]]
        speed,unique=velocities(rows)
        self.assertEqual(len(unique),5)
        self.assertTrue(all(abs(v['speed']-2)<1e-8 for v in speed))


if __name__=='__main__':unittest.main()
