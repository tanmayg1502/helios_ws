# Validation record — 2026-09-28

## Scope and environment

Xcode 26.4, Swift 6 language mode, iPhone 17e simulator with iOS 26.4. Companion gateway changes are in the [upstream pull request](https://github.com/pran99-git/helios_ws/pull/1). All command tests used the gateway's **simulated process backend**. No physical rover was connected, actuated or commissioned.

## Passed checks

- iOS app and test targets compile for Simulator.
- **10 unit tests** in three suites: demo playback/reset and trajectory units; endpoint/token validation; monotonic freshness including huge finite ages; invalid/versioned payloads; disconnected/demo state; typed command JSON; cancellation before transport; disconnected operation guards.
- **2 simulator UI tests:** authenticated fixture telemetry → disconnect → demo; explicit control acquisition → fixture-source check → movement confirmation → simulated motor-service start → stop.
- Production Swift telemetry client/controller against the real gateway fixture: correct decoding, rejected token, corrected credentials, background pause/resume, disconnect cancellation and demo transition.
- Fault harness using the gateway's actual HTTP/state implementation: delayed responses remain stale; a temporarily unavailable gateway is automatically reconnected when it starts.
- Production Swift operations harness against the actual command fixture: 20-operation catalog, exclusive lease, required confirmation, simulated process starts, navigation/joystick conflict rejection, provider-stop dependency rejection, stable command outcomes across heartbeats, ordered shutdown, background lease expiry, and explicit reacquisition after returning to foreground.
- Independent review completed. Fixed network-transit freshness, large-age conversion, misleading demo wording, keyboard dismissal, heartbeat overwriting command results, and cancellation of outstanding command/acquisition requests.
- Visual review of the WHOOP/Tesla-inspired dark redesign. Compact rover hero, readable status and cards, and active jobs/shutdown controls above the command catalog. Fixture labels and accessibility identifiers retained.

The simulator initially stalled during the original telemetry setup; restarting it recovered subsequent runs. Early UI tests also exposed keyboard dismissal and accessibility-selector issues; the passing suites include those fixes.

## Evidence and reproduction

- Unit result: `/tmp/helios-ios-verified/Logs/Test/Test-Helios-2026.09.28_13-48-26--0700.xcresult` (check actual latest bundle in that folder if Xcode regenerates it).
- Final UI result: `/tmp/helios-ios-verified/Logs/Test/Test-HeliosIntegration-2026.09.28_13-49-48--0700.xcresult`.
- Commands and reproduction steps: `README.md`, `OPERATIONS.md`, `Scripts/check-gateway.sh`, `Scripts/check-operations.sh`, and `Tests/run_gateway_faults.py`.
- Captured app screens: `Documentation/fixture-overview.png` and `Documentation/fixture-operations.png`.

## Limits

The tests establish app/API behavior, not real ROS launch success, map-file quality, discovery/QoS, Jetson device permissions, certificate trust on a physical phone, motor stopping behavior, or navigation readiness. The gateway starts only allowlisted owned processes and does not take over laptop-launched stacks. Lease expiry requests orderly shutdown; it is not an emergency stop and cannot guarantee an unresponsive process will stop. Commands default disabled on the real gateway until explicitly enabled by the operator.

Direct velocity commands, navigation goal submission/cancellation, GUI tools, package/device/network administration, calibration, battery, live map rendering and video streaming are not implemented. See `OPERATIONS.md` for the supported 20-operation scope. Hardware testing and physical-phone signing remain deployment work.

---
This file is part of the mirrored [Helios setup bundle](README.md). Shell commands and source paths refer to the original app or robot repository root, as specified in the guide.
