# Software Architecture — Sam & the Guides

The robots are dumb on purpose. **All intelligence lives in the software (the "hub").**
Because 3 XRP robots can't reliably mesh directly, the hub is the single coordinator: it holds a
Bluetooth link to each robot, reads their telemetry, and sends commands ~5×/second.

## 1. Layers (top to bottom)

```
┌─────────────────────────────────────────────────────────────┐
│  CLIENTS                                                     │
│   • Hub UI (blind user / operator): voice in, mission, live  │
│   • Family view (read-only, remote)                          │
├─────────────────────────────────────────────────────────────┤
│  CLOUD  (optional — Supabase Realtime)                       │
│   • relays live state to remote family • logs journeys       │
├─────────────────────────────────────────────────────────────┤
│  HUB / COORDINATOR   (web app, laptop or phone, Web Bluetooth)│
│   MissionController · Planner · Coordinator · SafetySupervisor│
│   Voice(STT/TTS) · Fleet · TelemetryStore · MapModel · CloudSync│
├─────────────────────────────────────────────────────────────┤
│  TRANSPORT   BLE (Nordic UART) — one link per robot          │
├─────────────────────────────────────────────────────────────┤
│  ROBOT FIRMWARE  (MicroPython, identical on all 3 XRPs)      │
│   line-follow · follow-leader(ultrasound) · stop-on-obstacle │
│   · LED/arm signals · emit telemetry                         │
└─────────────────────────────────────────────────────────────┘
```

## 2. Modules (each: one job, clear interface, testable alone)

| Module | Responsibility | Interface (in → out) |
|---|---|---|
| **RobotLink** | One BLE connection to one robot | `connect()`, `send(cmd)`, `onTelemetry(cb)` |
| **Fleet** | Owns the 3 RobotLinks + assigns roles | `assign(role)`, `broadcast(cmd)`, `get(role)` |
| **TelemetryStore** | Latest known state of each robot | `update(id,t)`, `get(id)`, `subscribe(cb)` |
| **MapModel** | Places + routes as a graph (edge = taped line, with length) | `route(from,to) → [segments]`, `length(route)` |
| **Planner** | Route choice, ETA, scheduled departure | `plan(dest)`, `eta(route,speed)`, `departAt(arriveBy)` |
| **Coordinator** | 5 Hz control loop: formation, speed & spacing sync | consumes TelemetryStore → emits Fleet commands |
| **SafetySupervisor** | Watchdog, e-stop, obstacle policy | veto power over all commands |
| **Voice** | Speech-to-text (destination) + narration (turns/alerts) | `listen()→text`, `say(text)` |
| **MissionController** | Orchestrates a whole trip | `go(dest)` → runs plan → drive → monitor → narrate |
| **CloudSync** | Publish state to Supabase for family view | `publish(state)` |
| **UI** | Hub dashboard + family view | renders TelemetryStore + feed |

## 3. Robot roles (same firmware, role set by the hub)
**Sam is the blind *person*.** Three robots serve Sam:
- **Lead guide** — follows the line to the destination, reports floor markers (turns/stations).
- **Guard guide** — travels beside/behind as a second safety layer.
- **Companion** — stays right next to Sam; follows the guides by ultrasound (keeps a fixed gap), stops on obstacle, taps its arm as a physical cue. *In the live demo this robot represents Sam (the person).*

## 4. The coordination loop (the "brain"), ~5 Hz
```
read telemetry from all 3
  ├─ if any robot sees an obstacle      → SafetySupervisor: STOP ALL + Voice.say("obstacle")
  ├─ if companion.gap > target + margin → Fleet.lead.slow()  + Fleet.companion.speedUp()
  ├─ if companion.gap < target - margin → Fleet.lead.speed() / Fleet.companion.hold()
  ├─ if lead at a marker (turn/station) → Voice.say("turning right") ; log
  └─ everyone caps speed to the slowest member (Sam)
CloudSync.publish(state)   // family view updates in real time
```

## 5. Data flow of one trip
```
Voice "take me to the store"
   → MissionController.go("store")
   → Planner.plan → route + ETA ("leave 5:52 to arrive 6:00")
   → Coordinator drives Fleet along the route
   → robots stream telemetry → Coordinator adjusts + SafetySupervisor guards + Voice narrates
   → CloudSync → Family view (live)
   → arrival: Voice "you have arrived", Sam arm-tap, log journey
```

## 6. Why this shape
- **One coordinator, dumb robots** = matches the real hardware (no reliable robot-to-robot BLE) and keeps all logic in one testable place.
- **Small modules with clear interfaces** = each can be built and tested alone (mock a RobotLink to test the Coordinator with no hardware).
- **Cloud is optional** = the whole system works offline on the laptop; Supabase only adds the remote family view.
- **Firmware stays identical** on all 3 robots; only the *role* differs — easy to swap a robot, easy to scale past three.

## 7. Build order (each piece independently demoable)
1. `RobotLink` + firmware telemetry → prove one robot talks to the hub.
2. `Fleet` + `Coordinator` (follow-leader + e-stop) → prove 3 robots hold formation.
3. `MapModel` + `Planner` → prove routing + ETA.
4. `Voice` + `MissionController` → prove "say destination → trip runs".
5. `CloudSync` + Family view → prove remote live monitoring.
