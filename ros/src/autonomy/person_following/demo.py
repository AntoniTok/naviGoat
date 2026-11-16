"""Print following behaviour using scripted observations and simulated time.

Run python3 demo.py, or python3 demo.py --scenario target-loss.
No camera, ROS, robot connection, third-party libraries or real-time waits.
These positions are scripted inputs, not a simulation of the robot's physics.
"""

from __future__ import annotations

import argparse

from state_machine import (FollowingOutput, FollowingStateMachine,
                           PerceptionFrame, PersonObservation)


def person(stamp, forward=2.0, lateral=0.0, *, confirmed=True):
    return PerceptionFrame(stamp, (PersonObservation(
        track_id=7, forward_m=forward, lateral_m=lateral, confirmed=confirmed,
    ),))


def following_steps():
    """Enable, acquire, approach the distance band, then steer left/right."""
    yield 0.0, person(0.0), None, "Disabled: seeing a person does not start motion"
    yield 0.25, PerceptionFrame(0.25), "enable", "Wait for a confirmed person"
    yield 0.5, person(0.5, confirmed=False), None, "Tentative track: keep waiting"
    for stamp in (0.75, 1.0, 1.25, 1.5):
        yield stamp, person(stamp), None, "Person 2 m ahead"
    yield 1.75, person(1.75, 1.25), None, "Closer: reduce forward speed"
    yield 2.0, person(2.0, 1.1), None, "Inside distance band: stop forward motion"
    yield 2.25, person(2.25, 0.7), None, "Too close: do not reverse"
    yield 2.5, person(2.5, 2.0, 0.5), None, "Person left: turn left"
    yield 2.75, person(2.75, 2.0, -0.5), None, "Person right: smoothly change turning"
    yield 3.0, person(3.0, 2.0, -0.5), None, "Turning right"
    yield 3.25, person(3.25, 2.0, 2.0), None, "45-degree offset: turn in place"
    yield 3.5, person(3.5), "disable", "Explicit disable: zero immediately"


def target_loss_steps():
    """Brief disappearance, same-ID return, then loss requiring re-enable."""
    for stamp in (0.0, 0.25, 0.5, 0.75):
        yield stamp, person(stamp, lateral=0.3), (
            "enable" if stamp == 0 else None), "Following track 7"
    for stamp in (1.0, 1.25, 1.5):
        yield stamp, PerceptionFrame(stamp), None, "Working camera, no person: ramp to zero"
    yield 1.75, person(1.75, lateral=0.3), None, "Same track returns: resume smoothly"
    for stamp in (2.0, 2.25):
        yield stamp, person(stamp, lateral=0.3), None, "Following again"
    for index in range(8):
        stamp = 2.5 + index * 0.25
        yield stamp, PerceptionFrame(stamp), None, "Missing; timeout is since last sighting at 2.25 s"
    yield 4.5, person(4.5), None, "Person returns after exit: remain idle"
    yield 4.75, person(4.75), "enable", "Explicit re-enable permits acquisition"
    yield 5.0, person(5.0), None, "Following enabled again"


def camera_timeout_steps():
    """No new frames differs from a working camera reporting no people."""
    for stamp in (0.0, 0.25, 0.5):
        yield stamp, person(stamp), (
            "enable" if stamp == 0 else None), "Camera sending fresh frames"
    for stamp in (0.75, 1.0, 1.25):
        yield stamp, None, None, "No new frames; watchdog stops after 0.5 s of stale input"
    yield 1.5, person(1.5), None, "Camera returns: explicit enable still required"


def nearest_person_steps():
    """Select the nearest on enable; retain that target when others approach."""
    for stamp in (0.0, 0.25, 0.5):
        people = (PersonObservation(8, 2.0, 2.0), PersonObservation(7, 2.5, 0.0))
        yield stamp, PerceptionFrame(stamp, people), (
            "enable" if stamp == 0 else None), "ID 7 is nearer by horizontal range"
    for stamp in (0.75, 1.0):
        people = (PersonObservation(8, 1.0, 0.0), PersonObservation(7, 2.5, 0.0))
        yield stamp, PerceptionFrame(stamp, people), None, "ID 8 comes closer: keep ID 7"
    yield 1.25, PerceptionFrame(1.25, (PersonObservation(8, 1, 0),)), None, "ID 7 missing: do not follow ID 8"
    people = (PersonObservation(8, 1.0, 0.0), PersonObservation(7, 2.5, 0.0))
    yield 1.5, PerceptionFrame(1.5, people), None, "Selected ID 7 returns"
    yield 1.75, PerceptionFrame(1.75, people), "disable", "Exit clears target selection"
    yield 2.0, PerceptionFrame(2.0, people), "enable", "New session selects nearest: ID 8"


SCENARIOS = {
    "following": following_steps,
    "target-loss": target_loss_steps,
    "camera-timeout": camera_timeout_steps,
    "nearest-person": nearest_person_steps,
}


def print_row(stamp, input_text, output: FollowingOutput, note):
    target = "-" if output.target_id is None else str(output.target_id)
    print(f"{stamp:5.2f}  {input_text:<22} {output.state.value:<18} "
          f"{target:>3}  {output.command.linear_mps:>6.3f} "
          f"{output.command.angular_rps:>7.3f}  {output.event or '-':<18} {note}")


def run_scenario(name):
    machine = FollowingStateMachine()
    print(f"\nScenario: {name}")
    print(" t(s)  Input                  State               ID   v(m/s)  w(rad/s)  Event              Note")
    for stamp, frame, action, note in SCENARIOS[name]():
        if action:
            output = machine.enable() if action == "enable" else machine.disable()
            print_row(stamp, action, output, "Explicit mode command")
        if frame is None:
            input_text = "no new frame"
        elif not frame.people:
            input_text = "no person"
        elif len(frame.people) == 1:
            target = frame.people[0]
            input_text = f"x={target.forward_m:.2f}, y={target.lateral_m:+.2f} m"
        else:
            input_text = "IDs " + ",".join(str(p.track_id) for p in frame.people)
        output = machine.update(stamp, frame)
        print_row(stamp, input_text, output, note)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=["all", *SCENARIOS], default="all")
    args = parser.parse_args()
    print("Dummy observations, simulated seconds. No commands are sent to hardware.")
    print("x = camera-forward, y = left; positive angular velocity turns left.")
    print("Positions are scripted, so they do not respond to the velocity commands.")
    for name in SCENARIOS if args.scenario == "all" else [args.scenario]:
        run_scenario(name)


if __name__ == "__main__":
    main()
