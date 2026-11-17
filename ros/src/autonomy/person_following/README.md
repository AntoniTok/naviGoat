# Person-following prototype and camera test

The following logic and dummy demo use only the Python standard library (Python
3.9 or newer). A separate ROS runner can consume Enid's live tracks on the Jetson
and print hypothetical velocity commands. It receives middleman mode commands
and publishes following status, but sends no drive commands. No robot is needed
for the camera test.

## Run the demo

From the repository root:

```bash
python3 -B ros/src/autonomy/person_following/demo.py
```

Run one scenario at a time with `--scenario following`, `--scenario target-loss`
or `--scenario camera-timeout`. Use `--scenario nearest-person` to see selection
and target locking with multiple people. All scenarios run immediately using simulated
time; no Jetson or camera is needed.

The table shows input position, state, selected track ID, forward velocity
`v` (m/s), turning velocity `w` (rad/s), and any one-shot event. Positive lateral
position and positive turning velocity mean left. `enable`/`disable` rows are
explicit mode commands, followed by the control update at the same time.

- **following:** waiting for confirmation, forward speed adjustment, stopping
  inside the distance band, left/right steering, and explicit disable.
- **target-loss:** a 0.5 s slowdown, smooth same-ID resumption, then a 2 s timeout
  measured since the last actual sighting. Returning after exit does not restart
  following until explicitly enabled.
- **camera-timeout:** no new frames for more than 0.5 s causes an immediate zero
  command and exit, rather than the ordinary missed-detection slowdown.
- **nearest-person:** selects the nearest confirmed, observed person by horizontal
  camera range, `hypot(forward, lateral)`, when enabled. A closer newcomer cannot
  replace that target. Explicit disable and re-enable allow a new selection.

The control loop runs every 0.25 simulated seconds in this demo. Watchdog and
loss deadlines are acted on at the first update at or after their threshold
(the stale-input watchdog uses strictly greater than 0.5 s).

These are scripted observations, not simulated robot physics. They demonstrate
the logic; they do not measure physical following accuracy or stopping distance.

## How the files interact

`demo.py` supplies a `PerceptionFrame` to `FollowingStateMachine.update()` on each
control tick. On acquisition the state machine selects the nearest confirmed,
observed track, then retains its ID even if another person comes closer. It
checks freshness and timing, then calls `VelocityController.compute()` while
following. Its returned `FollowingOutput` contains state, command, target ID and
an optional status event. The demo prints that output.

Call `update()` regularly even when no perception result arrives. Use `frame=None`
for no new result, an empty `people` tuple for a fresh result with no person, and
`input_healthy=False` for a known camera/depth fault. Frame and control timestamps
must share one monotonic clock domain.

The controller defaults to a 1.0 m camera-forward distance with a 0.15 m tolerance,
0.2 m/s maximum forward speed and 0.5 rad/s maximum turning speed. It approaches
the upper distance-band edge (1.15 m); it does not reverse. At bearings of 30
degrees or more it requests turning in place. These are initial test parameters.

Same-ID resumption assumes the upstream tracker associated the person correctly.
Before moving-robot integration, address Enid's stationary-camera tracking
assumption. Drive-command routing remains future work; the middleman connection
is implemented below but still needs live ROS verification.

## Live Jetson + camera test

The camera stays stationary and level; you move in its field of view.

```text
Femto Bolt → Enid's ROS camera + depth registration
           → ONE YOLO-World detector + Kalman tracker
           → /social_perception/tracks (JSON in std_msgs/String)
           → perception_adapter.py → state_machine.py → velocity_controller.py
           → ros_camera_test.py prints velocities and positions
                               ↔ middleman mode commands / following status
```

Enid's eight files under `enid/social_perception/` and `docker_enid/Dockerfile`
were copied unchanged from `origin/enid/social-perception` at commit `7cd2df1`.
No unrelated navigation or simulation files were imported. Her original host
launcher mounts only the `enid` directory and assumes `~/naviGoat`, so use the
whole-repository camera-test launch below instead.

New integration files:

- `perception_adapter.py`: validates the existing JSON format and maps capture
  timestamps to monotonic time while preserving processing delay. Duplicate
  results cannot refresh the watchdog. Predicted tracks stay unobserved.
- `ros_camera_test.py`: subscribes with a one-message queue, runs control at
  20 Hz using a steady timer, logs outputs, exposes an enable/disable service,
  receives `/middleman/mode`, and publishes `/person_following/status`. It does
  not load a detector or publish drive velocities.
- `mode_bridge.py`: validates middleman commands, retains the selected person
  across repeated follow commands, rejects named contacts, and echoes request IDs
  in feedback. This module has no ROS dependency and is tested on the laptop.
- `run_camera_test.sh`: starts Enid's unchanged camera/tracker/dashboard pipeline
  and the console runner. It starts idle by default for voice/middleman control;
  pass `--enable-on-start` explicitly for a standalone camera test. It sets
  `TRACK_MAX_AGE=2.0` to match our loss window.
  It defaults `DASHBOARD_PORT` to `8766`, leaving `8765` for the voice pipeline's
  Whisper server. An explicit `DASHBOARD_PORT` environment value is preserved.
  It temporarily relaxes unset-variable checking while sourcing ROS setup,
  restores strict checking afterwards, and aborts if setup fails.
- `docker/Dockerfile.following-test`: adds explicit ROS registration/runner
  dependencies to Enid's existing `navigoat:people-dev` image.

### Prepare the Jetson

Transfer this whole checkout, including the new uncommitted/untracked files, to
the Jetson. A `git pull` alone will not transfer files that have not been committed
and pushed. Run the commands below from the repository root **on the Jetson**.

If Enid's working `navigoat:people-dev` image is already there, reuse it and build
only the small dependency layer:

```bash
sudo docker build -f docker/Dockerfile.following-test -t navigoat:following-test .
```

If `navigoat:people-dev` is absent, Enid's original build chain is:

```bash
sudo docker build -f docker/Dockerfile.dev -t navigoat:dev .
sudo docker build -f docker_enid/Dockerfile -t navigoat:people-dev .
sudo docker build -f docker/Dockerfile.following-test -t navigoat:following-test .
```

The original image has JetPack/CUDA-specific PyTorch dependencies. Its build and
GPU compatibility have not been verified on this laptop or on your Jetson; reuse
a known-working image where available. This test does not need Pete's Ollama/VLM
extras (`PETE=true`). The first YOLO launch may download its model checkpoint.

### Start the camera test

Connect the camera. Stop any other program/container using it, including a
previous `navigoat-social` pipeline. Start this single stack:

```bash
sudo docker run --rm -it --init --name navigoat-following-test \
  --runtime=nvidia --gpus all --network=host --ipc=host \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
  --device-cgroup-rule='c 189:* rwm' \
  -v /dev/bus/usb:/dev/bus/usb \
  -v /run/udev:/run/udev:ro \
  -v "$PWD:/workspace" \
  navigoat:following-test \
  bash /workspace/ros/src/autonomy/person_following/run_camera_test.sh --enable-on-start
```

This starts the camera driver, depth registration, person detector/tracker,
Enid's local dashboard, and our console runner. Ctrl+C stops the stack. The
console runner starts enabled with this explicit flag, waits for a confirmed person, and logs commands
twice per second plus state events. Wait for model startup before testing.

Typical output:

```text
state=FOLLOWING target=7 v=0.198m/s w=+0.149rad/s position=(2.00,+0.30)m observed=True ...
state=TEMPORARILY_LOST ... event=target_missing
state=IDLE ... event=target_lost
```

After loss or a fault, use another Jetson terminal to re-enable explicitly:

```bash
sudo docker exec navigoat-following-test bash -c \
  'source /opt/ros/jazzy/setup.bash && ros2 service call /person_following/set_enabled std_srvs/srv/SetBool "{data: true}"'
```

Use `{data: false}` to disable. Standalone sessions publish status with a null
request ID, so the middleman ignores them. Voice sessions echo the request ID
from the middleman's command and can update its mode when following ends.

If Enid's pipeline is already running in a compatible ROS environment, start
**only** the subscriber there, with this full checkout available:

```bash
source /opt/ros/jazzy/setup.bash
python3 -B ros/src/autonomy/person_following/ros_camera_test.py --enable-on-start
```

Do not start the all-in-one launcher as well. If the existing container only
mounts `enid/`, it needs access to our directory too. The service and subscriber
must share the pipeline's ROS domain and clock configuration. Live camera stamps
must use the same ROS system clock as the runner; the adapter rejects future or
stale capture times rather than guessing a clock offset. Simulation/bag time is
not configured by this live launcher.

Through our launcher, the dashboard listens on `127.0.0.1:8766` by default. To
view it from your laptop, forward port `8766` over SSH to your Jetson and open
`http://localhost:8766`. Whisper can continue using `127.0.0.1:8765`.
It shows tracking, while following commands appear in the terminal.

### Lia's voice/middleman connection

The following runner now implements both directions of the interface:

| Direction | Topic | Message |
| --- | --- | --- |
| Middleman → following | `/middleman/mode` | `std_msgs/String` JSON: `mode`, `target`, `intent`, `stop`, `request_id` |
| Following → middleman | `/person_following/status` | `std_msgs/String` JSON: `request_id`, `state`, `event`, `target_id` |
| Middleman → voice | `/pete/response` | Existing spoken-response text |

An empty-target follow command enables nearest-person acquisition. Stop or any
other mode disables immediately. Repeating a follow request while active keeps
the selected person. A new command after timeout can acquire again. Named
contacts are rejected rather than silently selecting someone else.

Each middleman mode decision has a unique `request_id`; following echoes it in
status messages. Matching terminal target-loss, stale-input, input-fault or
unsupported-contact events return the middleman to idle and produce a spoken
explanation. Acquisition/progress events do not end a mode. Reports from older
sessions, duplicate failures and standalone tests are ignored. `/middleman/done`
is not used for following, so losing someone never announces "Arrived".

Both the following runner AND the middleman must run the updated code from this
branch. Older middleman follow commands without a request ID are rejected. The
extra JSON field is ignored by the navigation consumer. Status is sent on events
and about twice per second while the runner is alive; the middleman currently
does not implement a separate watchdog for the runner process dying entirely.

For a typed-command integration test on the Jetson:

1. Start the camera stack using the Docker command above, but **omit**
   `--enable-on-start`. It should remain IDLE until a mode command arrives.
2. In another Jetson terminal, launch the middleman inside that same container:

   ```bash
   sudo docker exec -it navigoat-following-test bash -c \
     'source /opt/ros/jazzy/setup.bash && ros2 launch /workspace/ros/src/core/launch/middleman.launch.py use_qwen:=false'
   ```

3. In another terminal, enter the container and source ROS:

   ```bash
   sudo docker exec -it navigoat-following-test bash
   source /opt/ros/jazzy/setup.bash
   ```

   Inject typed speech requests:

   ```bash
   ros2 topic pub --once /speech/text std_msgs/msg/String "{data: 'pete start following'}"
   ros2 topic pub --once /speech/text std_msgs/msg/String "{data: 'pete stop following'}"
   ros2 topic pub --once /speech/text std_msgs/msg/String "{data: 'pete manual mode'}"
   ```

   Observe `/middleman/mode`, `/person_following/status` or `/pete/response` in a
   separate terminal using `ros2 topic echo TOPIC`. After acquiring a person,
   leave the frame for two seconds: both following and the middleman should
   return to idle, with a target-loss response. No drive commands are sent.

For actual microphone testing, use Lia's ROS voice launcher,
`ros/src/interaction/voice/start_voice_ros.sh`, instead of starting a second
middleman. Her voice launch already includes one. Ensure its container mounts
this updated checkout (its script defaults to `~/naviGoat`) and uses the same
ROS domain as the following container. Her non-ROS `start_voice.sh` only runs the
standalone listener and does not provide these ROS commands.

### Camera-test checks and current limits

1. Stand alone about 2 m in front of the camera: acquire one ID, with positive
   forward speed. Move towards 1 m: forward speed should reach zero in the band.
2. Move to the camera's left/right: positive/negative angular commands. At a
   large sideways angle, the forward command should be zero.
3. Leave the frame: speed ramps down. Return with the same track before timeout:
   following resumes. Stay absent for 2 s: following exits; re-enable to restart.
4. Disconnect the camera: the runner's input watchdog requests zero and exits
   once results are stale. Some driver failures may terminate the entire stack.
5. Compare reported positions with measured camera-to-person distances and note
   the reported processing latency. These are observations to collect on hardware,
   not results established by the laptop tests.

Enid publishes **confirmed tracks with valid depth**, not every YOLO detection.
Only those tracks are candidates for nearest-person selection. Tentative extra
people are not visible to this subscriber, and an
invalid-depth detection cannot be distinguished from a missed detection. That
case follows the loss ramp; an explicit fault or malformed position stops
immediately. A separate depth-health signal is needed to distinguish these later.
Restarting the perception process also resets its IDs: stop/re-enable the runner
when restarting perception rather than trusting a reused number.

This wrapper changes neither Enid's association algorithm nor her camera
assumptions. The live ROS runner, Docker build and physical camera test have not
been run on this laptop, which has no ROS, Docker or camera environment. Laptop
tests cover the JSON-to-following logic with dummy data.

## Run tests

```bash
cd ros/src/autonomy/person_following
python3 -B -m unittest discover -s tests -v
```
