#!/usr/bin/env python3
"""Unit tests for the openpilot -> aapi OPNAV encoder. No car, no panda."""
import types
import unittest

from opendbc.car.honda.values import HondaFlags
from openpilot.selfdrive.navcan import navcan as N


def stub_cp(fp="HONDA_CIVIC_BOSCH", long=False):
  return types.SimpleNamespace(carFingerprint=fp, safetyConfigs=[None],
                               flags=HondaFlags.BOSCH,
                               openpilotLongitudinalControl=long)


def ins(valid=True, mtype="turn", mod="right", dist=250.0, text=""):
  return types.SimpleNamespace(valid=valid, maneuverType=mtype, maneuverModifier=mod,
                               maneuverDistance=dist, maneuverPrimaryText=text)


class NavCanTest(unittest.TestCase):
  def setUp(self):
    self.n = N.NavCan(stub_cp())
    self.assertTrue(self.n)

  def frames(self, instruction=None, frames=1):
    out = []
    for i in range(frames):
      out += self.n.update(instruction or ins(), i)
    return out

  def status_frames(self, instruction=None, frames=20):
    return [f for f in self.frames(instruction, frames) if f.address == N.OPNAV_STATUS]

  def test_bus_and_packer(self):
    self.assertEqual(self.n.bus, 0)  # Bus.body == CAN 0 for Honda Bosch
    self.assertTrue(self.n.packer)

  def test_non_honda_is_disabled(self):
    self.assertFalse(N.NavCan(stub_cp(fp="TOYOTA_COROLLA_TSS2")))

  def test_status_layout(self):
    f = self.status_frames()[0]
    self.assertEqual(f.address, N.OPNAV_STATUS)
    self.assertEqual(len(f.dat), 8)
    b = f.dat
    self.assertEqual(b[0], 0)                       # counter
    self.assertEqual(b[1] >> 4, N.VERSION)
    self.assertEqual(b[1] & 0xF, N.FLAG_VALID | N.FLAG_ROUTE_ACTIVE)
    self.assertEqual(b[2], N.TURN)                  # maneuver
    self.assertEqual(b[3] >> 4, N.RIGHT)            # turn side
    self.assertEqual(b[3] & 0xF, N.UNIT_METRES)
    self.assertEqual((b[4] << 8) | b[5], 250)       # distance
    self.assertEqual(b[6], 0)                       # eta unknown
    self.assertEqual(b[7], N.TTL_DS)
    # Golden vector the aapi-side decoder test also asserts, so the two ends cannot
    # drift apart without one of them failing.
    self.assertEqual(b.hex(), "0015042000fa001e")

  def test_status_cadence(self):
    got = self.frames(frames=N.STATUS_PERIOD)
    statuses = [f for f in got if f.address == N.OPNAV_STATUS]
    self.assertEqual(len(statuses), 1)  # frame 0 only, next at 50
    got = self.n.update(ins(), N.STATUS_PERIOD + 2)
    self.assertIn(N.OPNAV_STATUS, [f.address for f in got])

  def test_invalid_clears(self):
    f = self.status_frames(ins(valid=False))[0]
    self.assertEqual(f.dat[1] & 0xF, 0)
    self.assertEqual(f.dat[2], N.UNKNOWN)

  def test_arrived(self):
    f = self.status_frames(ins(mtype="arrive", mod="", dist=12.0))[0]
    self.assertEqual(f.dat[1] & 0xF, N.FLAG_VALID | N.FLAG_ARRIVED | N.FLAG_ROUTE_ACTIVE)
    self.assertEqual(f.dat[2], N.DESTINATION)
    self.assertEqual((f.dat[4] << 8) | f.dat[5], 0)

  def test_text_is_chunked(self):
    got = []
    for i in range(10):
      got += self.n.update(ins(text="x" * N.MAX_TEXT), i)
    text = [f for f in got if f.address in (N.OPNAV_TEXT_CTL, N.OPNAV_TEXT_CHUNK)]
    expected = N.MAX_TEXT // N.CHUNK_BYTES
    self.assertEqual(len(text), expected + 1)
    ctl = text[0]
    self.assertEqual(ctl.address, N.OPNAV_TEXT_CTL)
    self.assertEqual(ctl.dat[1], self.n.text_id)
    self.assertEqual(ctl.dat[2], N.MAX_TEXT)         # length
    self.assertEqual(ctl.dat[3], expected)           # chunks
    self.assertEqual(text[1].address, N.OPNAV_TEXT_CHUNK)
    self.assertEqual(text[1].dat[0], self.n.text_id)
    self.assertEqual(text[1].dat[1], 0)              # chunk index

  def test_text_only_on_change(self):
    self.frames(frames=N.STATUS_PERIOD * 2)

    def addrs(frames):
      return [f.address for f in frames]

    self.assertNotIn(N.OPNAV_TEXT_CTL, addrs(self.n.update(ins(), 99)))
    self.assertIn(N.OPNAV_TEXT_CTL, addrs(self.n.update(ins(text="New road"), 100)))

  def test_counter_wraps(self):
    self.n.counter = 255
    f = self.status_frames(ins(), 3)[0]
    self.assertEqual(f.dat[0], 255)
    self.n.last_status_frame = -N.STATUS_PERIOD
    f = self.n.update(ins(), 1)[0]
    self.assertEqual(f.dat[0], 0)


if __name__ == "__main__":
  unittest.main(verbosity=2)
