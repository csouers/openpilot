from selfdrive.body.lib.bodyd_helpers import dbc_dict


# supported cars here. Mimic car/make/values.py. Must have verified signals against a test route.
# this might not work out long term as some body messages could change between model years where ADAS doesn't
class CAR:
  ACCORD = "HONDA ACCORD 2018 SPORT 2T"
  CIVIC_BOSCH = 'HONDA CIVIC HATCHBACK 2017 SEDAN/COUPE 2019'
  CIVIC_BOSCH_DIESEL = "HONDA CIVIC SEDAN 1.6 DIESEL"


# id, time (0), data, bus
# CAN_COMMANDS = {
#   CAR.CIVIC_BOSCH: {
#     'UNLOCK': '[0x0ef81218, 0, b"\x02\x00", 0]',  # keyfob
#     'LOCK': '[0x0ef81218, 0, b"\x01\x00", 0]',  # keyfob
#   }
# }


DBC = {
  CAR.ACCORD: dbc_dict('honda_accord_s2t_2018_body_generated'),
  CAR.CIVIC_BOSCH: dbc_dict('honda_civic_hatchback_ex_2017_body_generated'),
  CAR.CIVIC_BOSCH_DIESEL: dbc_dict('honda_civic_sedan_16_diesel_2019_body_generated'),
}
