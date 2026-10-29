"""A reader must never observe a partial worker handshake or missing metadata."""
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch
from dynamuhsim.runtime import worker_readiness as ready


class WorkerReadinessTests(unittest.TestCase):
    def test_only_complete_closed_payload_becomes_visible(self):
        payload=dict(humans=[dict(id=i) for i in range(1,7)],human_motion_profile='group_cohesion_v2',encounter=dict(profile='crossing'),goal_clock=dict(profile='sim_v1'),large='x'*100000)
        with tempfile.TemporaryDirectory() as directory:
            runtime=Path(directory);target=runtime/'ready.json';at_publication=threading.Event();allow=threading.Event();errors=[];link=ready.os.link
            def held_link(source,destination):
                self.assertEqual(json.loads(Path(source).read_text()),payload)
                at_publication.set()
                if not allow.wait(3):raise TimeoutError('test publication gate')
                link(source,destination)
            def producer():
                try:ready.publish_ready(runtime,payload)
                except BaseException as error:errors.append(error)
            with patch.object(ready.os,'link',side_effect=held_link):
                worker=threading.Thread(target=producer);worker.start()
                try:
                    self.assertTrue(at_publication.wait(3));self.assertFalse(target.exists())
                finally:allow.set();worker.join(3)
            self.assertFalse(worker.is_alive());self.assertEqual(errors,[])
            self.assertEqual(json.loads(target.read_text()),payload)
            self.assertEqual([p.name for p in runtime.iterdir()],['ready.json'])

    def test_second_publisher_cannot_overwrite_handshake(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime=Path(directory);ready.publish_ready(runtime,dict(seed=48))
            with self.assertRaises(FileExistsError):ready.publish_ready(runtime,dict(seed=49))
            self.assertEqual(json.loads((runtime/'ready.json').read_text()),dict(seed=48))
            self.assertEqual(len(list(runtime.iterdir())),1)

    def test_serialization_failure_is_never_published(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):ready.publish_ready(directory,dict(value=float('nan')))
            self.assertEqual(list(Path(directory).iterdir()),[])
