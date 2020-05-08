from selfdrive.body.lib.bodyd_helpers import dbc_dict


# supported cars here. Mimic car/make/values.py
# this might not work out long term as some body messages could change between model years where ADAS doesn't
class CAR:
  CIVIC_BOSCH = 'HONDA CIVIC HATCHBACK 2017 SEDAN/COUPE 2019'

# id, time (0), data, bus
# CAN_COMMANDS = {
#   CAR.CIVIC_BOSCH: {
#     'UNLOCK': '[0x0ef81218, 0, b"\x02\x00", 0]',  # keyfob
#     'LOCK': '[0x0ef81218, 0, b"\x01\x00", 0]',  # keyfob
#   }
# }

DBC = {
  CAR.CIVIC_BOSCH: dbc_dict('honda_civic_hatchback_2017_body'),
}
