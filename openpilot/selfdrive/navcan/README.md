# navcan — openpilot nav instruction → Honda B-CAN → aapi

`card.py` appends OPNAV frames to the existing `sendcan` publish. The aapi companion box
reads them with a white panda on the 125 kbit/s body bus and turns them into Android Auto
nav events for the instrument cluster. Arrow + road name + distance only: the cluster has
no turn-angle display.

## Wire format
Defined in `opendbc_repo/opendbc/dbc/generator/honda/_body_2017.dbc` (generated into
`honda_body_2017_generated.dbc`).

| id | message | cadence |
|---|---|---|
| `0x1A5A0001` | `OPNAV_Status` — counter, version/flags, maneuver, side/unit, distance, eta, ttl | 2 Hz |
| `0x1A5A0002` | `OPNAV_TextCtl` — text id, length, chunks, ttl | on change (+1 repeat) |
| `0x1A5A0003` | `OPNAV_TextChunk` — 6 ASCII bytes per frame, up to 36 bytes | on change (+1 repeat) |

`MANEUVER` is the Android Auto `NextTurnEnum` numbering, so the aapi maps it 1:1.
`TTL_DS` is the frame lifetime in 100 ms units; the aapi clears the cluster when it expires
or when `FLAGS.VALID` is 0, so openpilot stopping does not leave a stale arrow.

Ids are private and distinct from the head unit's own write-only nav family
(`0x12F9595x`), which the aapi ignores.

## Safety
The frames go out on `CanBus(CP).bcm` (bus 0, where the Honda body bus is forwarded) and
must be in `HONDA_BOSCH_TX_MSGS` / `HONDA_BOSCH_LONG_TX_MSGS` with
`.check_relay = false`. They carry no actuation and the Honda `tx_hook` has no payload
opinion on them.

## Testing
- `python3 openpilot/selfdrive/navcan/test_navcan.py` — 9 cases, no car/panda needed.
  Includes a golden byte vector shared with the aapi decoder test.
- `pytest opendbc/safety/tests/test_honda.py` — the fork's 9 expected failures
  (`test_fwd_hook` ×8 + `TestHondaBoschLongSafety::test_relay_malfunction`) are unchanged.

## Not yet verified on car
1. The adapter forwards B-CAN *to* openpilot's CAN 0; nothing proves it forwards CAN 0
   *back out* to the physical 125 kbit bus. First test: send one `OPNAV_Status` and watch
   the aapi's `navcan.py --dry-run`.
2. That the aapi's white panda enumerates at all (the Pi is currently in a USB
   over-current storm; see the aapi repo's `tools/navcan/README.md`).

## Steering-angle dev source

`tools/nav/steering_nav.py` publishes a `navInstruction` from `carState.steeringAngleDeg`
(threshold + hysteresis, LEFT/RIGHT/STRAIGHT). It does **not** write CAN itself: msgq has a
single write pointer per ring, so a second `sendcan` publisher would corrupt card's stream.
card.py is the only sender; this script just feeds it. It refuses to run when
`NavigationEnabled` is set, because navd would be the second publisher on `navInstruction`.
