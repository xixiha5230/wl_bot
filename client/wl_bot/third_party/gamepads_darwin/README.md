# gamepads_darwin (local fork)

Vendored copy of `gamepads_darwin` 0.1.3 (from
https://github.com/flame-engine/gamepads), wired in through a
`dependency_overrides` entry in `../../pubspec.yaml`.

The only change is in
`macos/gamepads_darwin/Sources/gamepads_darwin/GamepadsListener.swift`:

- **Background input** — sets `GCController.shouldMonitorBackgroundEvents = true`
  so the controller keeps working while the app is not frontmost.
- **Already-connected pads** — enumerates `GCController.controllers()` on start
  (`GCControllerDidConnect` is not guaranteed for pads that were paired before
  launch, e.g. a DualShock 4 connected over Bluetooth).
- **Wireless discovery** — calls `startWirelessControllerDiscovery()` so a
  powered-on pad that is not yet connected still gets picked up.
- **De-duplicated attach** — a helper registers each pad exactly once and
  handles disconnects by identity.

Everything else (including the Dart-side normalization, which lives in the
upstream `gamepads` package) is untouched, so the public
`Gamepads.normalizedEvents` API is unchanged.
