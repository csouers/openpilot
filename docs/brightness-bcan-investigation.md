# Honda display day/night B-CAN investigation

Requirement: the car decides display day/night; in day mode use comma's stock ambient brightness, in night mode follow the dash dimmer. Parking lights alone are not a night-mode indication. Do not use Android Auto or comma camera exposure to decide the mode.

## Existing definitions (not yet verified as display-mode sources)

`opendbc_repo/opendbc/dbc/generator/honda/_body_2017.dbc`:

- `0x0AF81118` / `af811_Wipers_Lighting_Output`: `PARKING_LIGHTS` is byte 0 bit 6; separate `LIGHTING_ON` is byte 1 bit 6 (Motorola start 14, length 1). Its name does not prove it controls screen mode.
- `0x0EF87372` / `Solar_ef873`: `AUTO_LIGHT_DISENGAGED` (start 2/length 1), `AUTO_LIGHT_ENGAGED_1` (start 7/length 5), `PHOTO_CELL_LUX` (start 14/length 17). Scaling and relationship to the screen-mode decision remain unverified. A raw lux threshold would be another guess, not necessarily the car's decision.
- `0x12F85450` / `12f854_Lighting`: dimmer in byte 0 low six bits, parking lights in bit 6; byte 5 echoes dimmer. Other bits may need investigation.

## Saved-capture evidence, 2026-10-29

Inspected `/Users/csouers/honda-android-auto/canbus/baseline_bus1.jsonl` and eight JSONL captures under `/Users/csouers/honda-android-auto/logs/2026-10-07/can/`.

All these captures keep meter payload `16360000b0160000` constant. `af811` is either `080000000000` or `000000000000`, constant within each capture; parking lights and LIGHTING_ON are both off. Solar frame varies (`df...` or `db...`). Thus these recordings establish that the solar frame is present but cannot identify the screen-mode trigger: no parking-on daytime/nighttime transition is recorded in these candidate signals.

The local `hunt3.log` dimmer sweep contains `af811` byte-1 values 00, 80 and 40 independently of parking lights. That makes the separate LIGHTING_ON bit worth testing, but there are no screen-mode labels proving its meaning. No `0x0EF87372` entries were found in that sweep.

## Next verification

Find existing labeled captures or firmware decode for day/night. Otherwise record B-CAN while keeping parking lights ON and the dimmer fixed below top, then change the car's ambient sensor illumination until its screens actually switch. Compare `af811` byte 1, solar flags, and all bytes of `12f854`; include the manual full-bright override. Do not deploy an inferred bit as verified.

Any new convenience parser inputs must be registered ignore-alive/NaN so missing optional B-CAN frames cannot invalidate vehicle CAN. Include missing/stale-source fallback in implementation.

## Live verification and implementation

User-assisted parked-car capture on 2026-10-29, saved on four at `/data/openpilot/work/display-mode/capture-20261029.jsonl`, labels in `markers.jsonl`:

- User confirmed bright/day screens in sunlight; dimmer adjustment unavailable. Meter `d5360000b0160000`.
- Covered ambient sensor; user reported the screens dimmed. Meter changed to `55360000b0550000` at epoch 1793292375.310551.
- Uncovered sensor; user confirmed return to day mode. Meter returned to `d5360000b0160000` at epoch 1793292399.4546316.
- `af811` remained `484000000000` across the transition, so its separate LIGHTING_ON bit did NOT identify this display transition.

**Verified observed day/night override: `12F85450` byte 0 bit 7**, set for day. Parking bit 6 and dimmer low-six-bit value (21) stayed unchanged. Thus previous low-six-bit dimmer parsing discarded the crucial override. Byte 5 was 22 in day and 85 in night; it is not simply an unconditional raw-dimmer echo.

Implemented `DAYLIGHT_OVERRIDE` in the existing optional meter DBC message; `daylightOverride @5` in CarBodyState; forwarded through card; UI uses this car bit instead of the camera >=90 threshold. Day mode uses stock camera-based brightness/filter, night retains the existing dash/sensor blend. Five extracted-method UI checks, raw-bit checks, syntax, and diff whitespace checks passed locally. The subsequent ignition-independent UI listener (`honda_illumination.py`) passively decodes the same meter frame on buses 0/2, retains its last state when the bus sleeps, and rejects TX echoes and malformed frames. This removes the UI's dependency on card (which stops offroad). Normal screen wake/sleep remains unchanged. Three decoder tests and four extracted brightness-method checks passed.

Deployment/rebuild task `b73c7c0e6` completed successfully; UI running. Post-deployment live carBodyState verification with user: uncovered/day => daylightOverride=True, parkingLightsOn=True, illuminationStep=21; covered/night => daylightOverride=False, parkingLightsOn=True, illuminationStep=21. This verifies DBC -> carstate -> card -> cereal publication in both modes. User visually confirmed comma dimmed along with car screens.
