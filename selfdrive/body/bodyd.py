#!/bin/python3
from common.params import Params
# from time import sleep
# from openioc.honda import HondaInputOutputController
from selfdrive.boardd.boardd import can_list_to_can_capnp
import cereal.messaging as messaging
from common.basedir import BASEDIR
from common.hardware import PC
# import subprocess
# import yaml
import time
import os
from selfdrive.body.lib.bodyd_helpers import load_car, allowed, offroad, panda_connected
# TODO: abstract this
from selfdrive.car.honda.bodyinterface import BodyInterface

class BodyD(object):
  """Class to communicate with the car's body functions. Can only send commands when controlsd is dead."""

  def __init__(self):
    super(BodyD, self).__init__()
    self.p = Params()
    self.cache = None
    self.read_only = False

    self.cache = self.p.get("LiveParameters")
    # self.cache = 'HONDA ACCORD HATCHBACK 2017 SEDAN/COUPE 2019'

    # Wait until the next loop and try again
    if self.cache is None:
      return

    self.makes, self.make, self.carFingerprint, self.body = load_car(self.cache)
    if self.makes is None:
      raise Exception("bodyd: No body files found in selfdrive/car/*.")
    elif self.make is None:
      raise Exception("bodyd: Unsupported make. Supported makes are", self.makes)
    elif self.body is None:
      raise Exception("bodyd: The body file in selfdrive/car/", self.make, "failed tos load")
    elif self.carFingerprint is None:
      self.read_only = True
      print("bodyd: No compatible car found. Read-only mode based on CIVIC_BOSCH")

    # we have a supported car at this point

    # setup sockets
    can_timeout = None if os.environ.get('NO_CAN_TIMEOUT', False) else 100
    self.can_sock = messaging.sub_sock('can', timeout=can_timeout)
    # self.pm = messaging.PubMaster(['bodyState', 'bodyControl'])
    self.pm = messaging.PubMaster(['bodyState'])
    self.BI = BodyInterface(self)

    time.sleep(2)

    return

  def DEPRECATED_send(self, command):
    """Send, if we can. If so, set d and pass that to the send function."""
    if self.read_only:
      return "Denied. Read-Only"
    else:
      @staticmethod
      def _send_msg(msg):
        # TODO: move this to a published events list and check the car for the desired response
        """Send the actual frame to the car."""
        sendcan = messaging.pub_sock('sendcan')
        time.sleep(2)
        sendcan.send(can_list_to_can_capnp([msg], msgtype='sendcan'))
        return "Success"

      # TODO: handle a sleeping bus (track the bus state and do the wakeup)
        if allowed():
          if 'HONDA' in self.carFingerprint:
            if command == "unlock":
              d = [0x0ef81218, 0, b"\x02\x00", self.bus]
              return self._send_msg(d)
            elif command == "lock":
              d = [0x0ef81218, 0, b"\x01\x00", self.bus]
              return self._send_msg(d)
            else:
              return "Unrecognized command"
          else:
            return "Unsupported car"
        else:
          return "Not permitted. Vehicle is running"
        # read from can
        # last_can =
        # timeout = 250.0  # ms
        return

  def data_sample(self):
    """Receive data from sockets and update bodyState"""

    # Update bodyState from CAN
    bcan_strs = messaging.drain_sock_raw(self.can_sock, wait_for_one=True)
    BS = self.BI.update(bcan_strs)
    if BS is not None:
      self.pm.send('bodyState', BS)
      # exit()
    # self.sm.update(0)

    # # Check for CAN timeout
    # if not can_strs:
    #   self.can_error_counter += 1
    #   self.can_rcv_error = True
    # else:
    #   self.can_rcv_error = False

  def bodyd_thread(self):
    while 1:
      # TODO: use Ratekeeper
      off = offroad(self.p)
      panda = panda_connected(self.p)

      if panda or PC:
        if off:
          # slow mode
          # print('bodyd: tick')
          self.data_sample()
          # pull from socks and update BodyState
          time.sleep(1)
        else:
          # same same, but faster
          # print('bodyd: boom')
          self.data_sample()
          # fastest frame on b-can is 10hz
          time.sleep(0.1)
      else:
        print('bodyd: Board not connected. Waiting')
        time.sleep(5)

def main():
  while 1:
    while 1:
      b = BodyD()

      # in case we have cleared params
      if b.cache is None:
        print("bodyd: Unrecognized car. Start vehicle and wait 15 seconds")
        time.sleep(15)

      else:
        if b is not None:
          print("bodyd: running thread")
          b.bodyd_thread()

      print("bodyd: resetting")
      break



if __name__ == "__main__":
    main()
