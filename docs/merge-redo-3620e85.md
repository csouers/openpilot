# Reconstruction of merge 3620e85e94

## Scope and safety

- Original reference: `e8882b4bd` on `hondabody_c4`.
- Isolated branch: `audit/redo-3620e85`; worktree: `.pi/merge-redo` under the original checkout.
- Replacement merge: `8f7c829dd`, parents `d67bd687f` and `b1a662033` (same as the original merge).
- No push, deployment, reset, or rewrite of `hondabody_c4`.
- This is a fresh Git merge followed by reuse of reviewed original resolutions and selective repair patches, NOT an independent rewrite of every resolution. Inner submodule merges are reused, not redone.
- Operator direction: lane lines, road edges, and path drawing must follow upstream. Ghost/solid lead cars and other fork UI behavior remain separate. Subsequent explicit direction restores fixed-red road-edge colors `(255, 0, 21)` only; upstream geometry, widths, confidence opacity, lane/path drawing and disengagement fade remain.

## Behavior audit

The pre-merge fork changes 88 paths from the merge base. Of these, 69 do not overlap upstream changes: all 69 blobs are identical in the replacement merge. This is a useful guard, not a substitute for the following overlapping-file review.

| Overlapping path/group | Reconciliation |
| --- | --- |
| `.gitattributes` | Keep fork SVG plain-blob rule; accept upstream NPZ LFS rule. |
| `openpilot/cereal/log.capnp` | Accept upstream audible-alert enum split; retain fork schema additions. |
| `controls/lib/longitudinal_planner.py` | Retain fork throttle allowance, unfiltered cruise acceleration and candidate stack; accept upstream `CurveSpeed(dt)` and curve offset. |
| `controls/plannerd.py` | Retain vehicleParameters subscription; upstream ordering is semantically unchanged. |
| `test/longitudinal_maneuvers/plant.py` | Retain vehicleParameters test plumbing; resolve to the same tested result as original merge. |
| `selfdrived/alerts_offroad.json`, `events.py` | Accept upstream wording and user/system sound split; retain fork jetlink alerts/events. |
| `mici/layouts/home.py` | Fold repairs and final title/commit-hash layout into merge, retain upstream wake animation. No intermediate static-screen/revert history. |
| `mici/onroad/alert_renderer.py` | Retain fork icon guard; retain upstream wider default text. Replay later edge inset separately. |
| `mici/onroad/augmented_road_view.py` | Keep status rail, turn signals, BSM; fold API migration and empty-left-panel/layout repairs. Keep driver renderer object for geometry but never draw its DMoji. Do not replay fork alert-driven model visibility hook. |
| `mici/onroad/confidence_ball.py` | Retain fork status-rail geometry with upstream fade colors. Ball remains unused by normal fork onroad view. |
| `mici/onroad/hud_renderer.py` | Keep fork chestnut row, aligned orange icon and persistent set speed, including while disengaged. Preserve upstream wheel machinery without drawing it. |
| `mici/onroad/model_renderer.py` | Use upstream geometry/projection/gradient pipeline, road colors, widths, draw methods and disengagement fade. Remove fork steering-angle path entirely. Keep ghost/solid lead icons, replay-valid lead initialization, four-field lead tuples, and upstream stop bar. |
| `mici/onroad/torque_bar.py` | Retain fork scale with upstream layout/fade fixes. |
| `ui/onroad/hud_renderer.py`, `ui_state.py` | Complete public `started_frame` migration to `received_since_started`; retain chestnut state handling and dash-detent brightness with upstream dim/backlight clamp. |
| `system/hardware/hardwared.py` | Retain fork chestnut startup gate, accept removal of upstream boot-complete startup condition. |
| `panda`, `opendbc_repo` | Reuse `d68f2ec7` / `7c68585e`; confirm both original parents are ancestors. Final pointers exactly match original HEAD. See validation limitations below. |

### Why the road renderer differs from original HEAD

The original fork copied base projection/gradient methods into its mici subclass and added narrower red road edges and a steering-angle overlay. Later repairs added camera-height projection offsets and forced road graphics visible while disengaged. Those fork changes are intentionally absent here.

The base renderer is byte-identical to `b1a662033`. Mici `_draw_offset`, `_get_ll_color`, and `_draw_path`, plus the lane-line loop of `_draw_lane_lines`, have identical ASTs to that upstream commit. The road-edge loop intentionally uses fixed red per subsequent operator direction. The road visibility assignment/conditional also matches upstream. Geometry methods are inherited, with regression tests preventing fork overrides from quietly returning.

## Disposition of every post-merge commit

| Original commit | Disposition |
| --- | --- |
| `95342c402` | Fold home missing-label repair into replacement merge. |
| `9899122c6` | Fold chestnut `started_frame` migration into merge. |
| `7a4a946da` | Fold hidden DMoji, aligned orange chestnut and persistent-speed repairs into merge. |
| `b2201b052` | Fold empty-left-panel behavior into merge. |
| `a56810a36` | Omit temporary static-home change; canceled by next commit. |
| `c429c7133` | Omit canceling revert; preserve final animated fork home instead. |
| `f6c4e7bc3` | Fold disengaged persistent-speed repair into merge. |
| `da2384808` | Fold final fork home look with wake animation into merge. |
| `d97a9237b` | Fold remaining `started_frame` migrations into merge. |
| `42b6dda24` | Fold four-field lead tuple and reset fixes into merge. |
| `c934c7228` | Fold lead regression test into merge. |
| `6c04cc05e` | Replay as `fe1730a9b`: alert text edge inset. |
| `a2c038338` | Fold removal of empty reserved left-column width into merge. |
| `948fd3963` | Replay as `33746e9a6`: UI test tool. |
| `3c5f9572c` | Replay as `3faedfd0b`: test tool GPU/compass. |
| `f1efda5fd` | Replay as `9cce3acef`: turn-signal placement. |
| `23cf60a80` | Replay as `c6535dbfa`: lead-style test options. |
| `d65538f86` | Omit fork projection override; upstream geometry is required. |
| `7ab2adc9a` | Replay as `d96928791`: local fake-bus/UI rig. |
| `e554225d4` | Replay sender-only changes as `ee3ef09f2`; omit fork road visibility/alert-hook changes. |
| `e8882b4bd` | Omit overlay-hiding patch: entire fork steering-angle overlay is already removed. |

## Final comparison

Relative to original `e8882b4bd`, only these five paths differ:

1. `openpilot/selfdrive/ui/mici/onroad/model_renderer.py`: upstream road graphics, retained fork leads.
2. `openpilot/selfdrive/ui/mici/onroad/augmented_road_view.py`: remove the fork-only alert visibility setter.
3. `openpilot/selfdrive/ui/tests/test_mici_upstream_geometry.py`: new regression contracts.
4. `scripts/audit_redo_3620e85.py`: reproducible pinned-reference checks.
5. This report.

Every other tracked file and every submodule pointer matches the recovered original HEAD exactly. This establishes preservation of the known recovered state; it cannot prove there were no undiscovered losses in the original merge.

## Validation

- Initial reconstruction: 41 passed. Fixed-red follow-up: 43 passed, adding low/high lane-confidence edge-color regressions. Includes lead regression tests, upstream geometry contracts, Honda illumination tests, and 23 longitudinal maneuver tests.
- All Python paths added/modified by the replacement merge pass ruff; final changed-source compilation and whitespace checks run separately.
- Safety suites `test_honda.py`, `test_elm327.py`, `test_body.py`: **272 passed, 9 failed, 262 skipped, 108 subtests passed** on BOTH original and rebuilt checkout environments. JUnit comparison confirms identical failure identities: eight Bosch forwarding tests and one Bosch-long relay-malfunction test. These are baseline failures, not a clean safety pass. Generic `test_body.py` is comma-body coverage, not dedicated coverage for the fork's Honda-body modes.
- Both checkouts use the same unchanged opendbc sources and compiled safety library. This is regression comparison, not a fresh firmware build or independent revalidation of the inner safety merge.
- Actual synthetic UI tool ran 119 frames with toggle leads and 16 frames at device size with ghost leads, failed chestnut and home status bar; no exception. Screenshots saved and inspected. This verifies startup/drawing, NOT real-route geometry correctness: the synthetic scene is not sufficient evidence that every lane/edge is visibly correct.
- No real-device test, full home-screen animation/offroad-to-onroad integration sweep, deterministic screenshot parity, or new panda firmware build performed. Do not treat this branch as deployment-approved.

Local (ignored) artifacts: `.pi/merge-redo-validation/` in the original checkout, including `ui-controls.xml`, safety baseline/candidate JUnit and logs, and screenshots. Test-only pytest dependencies live under `.pi/merge-validation-deps/`. The worktree links five existing compiled runtime artifacts from the original checkout; it does not rebuild them.

### Reproduce

From `.pi/merge-redo`:

```sh
../../.venv/bin/python scripts/audit_redo_3620e85.py
export PYTHONPATH="$PWD:$PWD/../merge-validation-deps:$PWD/../../:$PWD/../../msgq_repo:$PWD/../../openpilot"
../../.venv/bin/python -m pytest -q \
  openpilot/selfdrive/ui/tests/test_honda_illumination.py \
  openpilot/selfdrive/ui/tests/test_mici_lead_bars.py \
  openpilot/selfdrive/ui/tests/test_mici_upstream_geometry.py \
  openpilot/selfdrive/test/longitudinal_maneuvers
```
