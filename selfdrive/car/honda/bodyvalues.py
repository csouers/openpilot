from selfdrive.body.lib.bodyd_helpers import dbc_dict


# Supported cars here. Must have verified signals against a test route.
# See > https://github.com/csouers/bodyharness/wiki/Data-Collection#window-lock-and-door-capture---instructions

class CAR:
  ACCORD = "HONDA ACCORD 2018 SPORT 2T"
  ACCORD_15 = "HONDA ACCORD 2018 LX 1.5T"
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
  CAR.ACCORD_15: dbc_dict('honda_accord_lx15t_2018_body_generated'),
  CAR.CIVIC_BOSCH: dbc_dict('honda_civic_hatchback_ex_2017_body_generated'),
  CAR.CIVIC_BOSCH_DIESEL: dbc_dict('honda_civic_sedan_16_diesel_2019_body_generated'),
}
