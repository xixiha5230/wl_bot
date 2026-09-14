import Foundation
import GameController

class GamepadsListener {
    var gamepads: [GCExtendedGamepad] = []
    var listener: ((Int, GCExtendedGamepad, GCControllerElement) -> Void)?

    init() {
        // Keep delivering input while the app is not frontmost. Without this
        // macOS only forwards controller events to the active app.
        if #available(macOS 11.3, *) {
            GCController.shouldMonitorBackgroundEvents = true
        }
        NotificationCenter.default.addObserver(
            self,
            selector: #selector(joystickDidConnect),
            name: .GCControllerDidConnect,
            object: nil
        )
        NotificationCenter.default.addObserver(
            self,
            selector: #selector(joystickDidDisconnect),
            name: .GCControllerDidDisconnect,
            object: nil
        )
        // Pick up controllers that were already connected before launch:
        // GCControllerDidConnect is not guaranteed for those.
        for controller in GCController.controllers() {
            attach(controller)
        }
        // Finish pairing for wireless controllers that are powered on but not
        // currently connected (safe no-op when there are none).
        GCController.startWirelessControllerDiscovery(completionHandler: nil)
    }

    deinit {
        NotificationCenter.default.removeObserver(self)
    }

    @objc private func joystickDidConnect(notification: NSNotification) {
        if let controller = notification.object as? GCController {
            attach(controller)
        }
    }

    @objc private func joystickDidDisconnect(notification: NSNotification) {
        if let controller = notification.object as? GCController,
           let gamepad = controller.extendedGamepad {
            gamepads.removeAll(where: { $0 === gamepad })
        }
    }

    /// Registers a controller exactly once and starts forwarding its events.
    private func attach(_ controller: GCController) {
        guard let gamepad = controller.extendedGamepad else {
            return
        }
        guard !gamepads.contains(where: { $0 === gamepad }) else {
            return
        }
        gamepads.append(gamepad)
        let gamepadId = gamepads.count - 1
        controller.playerIndex = toPlayerIndex(index: gamepadId)

        gamepad.valueChangedHandler = { [weak self] gamepad, element in
            self?.listener?(gamepadId, gamepad, element)
        }
    }

    private func toPlayerIndex(index: Int) -> GCControllerPlayerIndex {
        switch index {
        case 0:
            return GCControllerPlayerIndex.index1
        case 1:
            return GCControllerPlayerIndex.index2
        case 2:
            return GCControllerPlayerIndex.index3
        case 3:
            return GCControllerPlayerIndex.index4
        default:
            return GCControllerPlayerIndex.indexUnset
        }
    }
}
