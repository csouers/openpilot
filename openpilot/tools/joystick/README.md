# Joystick

**Hardware needed**: device running openpilot, laptop, joystick (optional)

With joystick_control, you can connect your laptop to your comma device over the network and debug controls using a joystick or keyboard.
joystick_control uses [inputs](https://pypi.org/project/inputs) which supports many common gamepads and joysticks.

## Usage

The car must be off, and openpilot must be offroad before starting `joystick_control`.

### Using a keyboard

SSH into your comma device and start joystick_control with the following command:

```shell
openpilot/tools/joystick/joystick_control.py --keyboard
```

The available buttons and axes will print showing their key mappings. In general, the WASD keys control gas and brakes and steering torque in 5% increments.

### Controller mapping

Defaults target an Xbox-style controller (an `Xbox 360 pad` clone is what's in the car), read as raw evdev axes:

| Control | evdev axis | maps to |
|---|---|---|
| left stick, left/right | `ABS_X` (-32768..32767) | steering (left = positive torque) |
| RT | `ABS_RZ` (0..255) | accelerate (`+`) |
| LT | `ABS_Z` (0..255) | brake (`-`) |
| Y (`BTN_NORTH`) | - | cancel cruise control |

`accel = expo(RT) - expo(LT)`, `steer = expo(left stick X, inverted)`, both with a 3% deadzone and the
same `EXPO` curve as before (`testJoystick.axes = [accel, steer]`, consumed by `joystickd`).
To use other axes, edit `steer_axis` / `gas_axis` / `brake_axis` / `cancel_button` in `Joystick.__init__`;
`invert_map` flips a sign (e.g. if the wheel turns the wrong way).

Two behaviours worth knowing:

- **Deadman (`--deadman`, default 3 s)**: a controller that stops reporting (switched off, power save,
  wedged USB) leaves the kernel's last axis values in place, which would pin the last steer/accel command
  forever. After 3 s without any event the axes are released to 0 and `no controller input ... releasing
  axes` is printed; the next event restores them. A deliberately held stick emits no events either, so a
  long steady hold gets released too — raise the timeout (e.g. `--deadman 10`) or pass `--deadman 0`.
- **Offroad start**: joystick mode replaces `controlsd`, so the tool only starts once openpilot is offroad.
  It now waits for the offroad transition instead of exiting, so a copy started by the manager (which
  spawns it as soon as `JoystickDebugMode` is set) comes up by itself.

### Joystick on your comma device

Plug the controller into the comma device's aux USB-C port. Then, SSH into the device and start `joystick_control.py`.
Best to run it detached (`tmux`/`setsid`) or let the manager own it: a copy started from an interactive ssh
shell dies with the ssh session when the device loses Wi-Fi.

Note on mici: a controller in the USB-C port sets the Type-C CC orientation, which used to make `modeld`
take the chestnut (big model) path and selfdrived soft-disable engagement 3 s after engage — i.e. joystick
mode was unusable. `chestnut_expected()` now requires a real chestnut enumeration, so an accessory in the
port is fine.

### Joystick on your laptop

In order to use a joystick over the network, we need to run joystick_control locally from your laptop and have it send `testJoystick` packets over the network to the comma device.

1. Connect a joystick to your PC.
2. Connect your laptop to your comma device's hotspot and open a new SSH shell. Since joystick_control is being run on your laptop, we need to write a parameter to let controlsd know to start in joystick debug mode:
   ```shell
   # on your comma device
   echo -n "1" > /data/params/d/JoystickDebugMode
   ```
3. Start webrtcd so your laptop can connect to the device:
   ```shell
   # on your comma device
   openpilot/system/webrtc/webrtcd.py --host 0.0.0.0
   ```
4. Start joystick_control and the bridge on your laptop. The bridge sends the `testJoystick` packets to the device so that openpilot can receive them:
   ```shell
   # on your laptop
   openpilot/system/webrtc/bridge.py {DEVICE_IP} --send testJoystick &
   openpilot/tools/joystick/joystick_control.py
   ```

---
Now start your car and openpilot should go into joystick mode with an alert on startup! The status of the axes will display on the alert, while button statuses print in the shell.

Make sure the conditions are met in the panda to allow controls (e.g. cruise control engaged). You can also make a modification to the panda code to always allow controls.

![](https://github.com/commaai/openpilot/assets/8762862/e640cbca-cb7a-4dcb-abce-b23b036ad8e7)
