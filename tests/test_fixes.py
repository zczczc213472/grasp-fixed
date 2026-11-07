import io
import json
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from grasp_client import Channel, grip_result, call

class FixTests(unittest.TestCase):
    def test_small_block_contact(self):
        samples = [dict(width=.0218, effort=39, moving=False)] * 3
        self.assertEqual(grip_result(samples, .012), 'contact')

    def test_motion_is_not_grasp(self):
        self.assertIsNone(grip_result([dict(width=.04, effort=39, moving=True)] * 3, .012))

    def test_empty_and_open(self):
        self.assertEqual(grip_result([dict(width=.012, effort=9.5, moving=False)] * 3, .012), 'empty')
        self.assertEqual(grip_result([dict(width=.105, effort=3, moving=False)] * 3, .105, False), 'reached')

    def test_transient_contact(self):
        self.assertIsNone(grip_result([dict(width=w, effort=39, moving=False) for w in [.03,.025,.022]], .012))

    def test_logs_and_wrong_ids(self):
        p = SimpleNamespace(stdout=io.StringIO('SDK log\n{}\n'+json.dumps(dict(id=99,ok=True))+'\n'+json.dumps(dict(id=1,ok=True))+'\n'), stdin=io.StringIO())
        self.assertTrue(Channel(p).request({'cmd':'state'})['ok'])

    def test_read_timeout(self):
        channel = Channel(SimpleNamespace(stdout=io.StringIO(''), stdin=io.StringIO()))
        time.sleep(.01)
        while not channel.lines.empty():
            channel.lines.get()
        started = time.monotonic()
        with self.assertRaises(TimeoutError):
            channel.request({'cmd':'state'}, timeout=.03)
        self.assertLess(time.monotonic()-started, .5)
        self.assertTrue(channel.failed)

    def test_motion_failure_aborts(self):
        p=SimpleNamespace(stdout=io.StringIO('{"id":1,"ok":false,"status":"TIMEOUT"}\n'), stdin=io.StringIO())
        channel=Channel(p)
        with self.assertRaises(RuntimeError):
            channel.request({'cmd':'cart'})
        with self.assertRaises(RuntimeError):
            channel.request({'cmd':'grip'})
        self.assertEqual(len(p.stdin.getvalue().splitlines()),1)

    def test_grip_waits_for_stable_contact(self):
        class FakeChannel:
            def __init__(self): self.requests=[]; self.i=0
            def request(self, req, timeout=40):
                self.requests.append(req)
                if req['cmd']=='grip': return {'ok':True}
                self.i+=1
                return {'ok':True,'grip':dict(width=.0218 if self.i>1 else .06,effort=39,moving=self.i==1)}
        ch=FakeChannel()
        _, result=call(SimpleNamespace(channel=ch), {'cmd':'grip','w':.012,'blocking':True})
        self.assertTrue(result['grasped'])
        self.assertEqual(ch.i,4)
        self.assertFalse(ch.requests[0]['blocking'])

if __name__=='__main__': unittest.main()
