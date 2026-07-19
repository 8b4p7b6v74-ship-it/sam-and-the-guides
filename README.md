# Sam & the Guides 🤖🦯

**Navigation and guidance for blind and visually-impaired people** — a camera-free convoy of three XRP robots plus a software hub that physically guides a blind traveler ("Sam") through a mapped mini-city.

> Capstone project — Engineering & Robotics, July 2026
> By Violet, Samuel and Gabriel

## The formation

```
  OPENER  →→→        SAM        →→→  CLOSER
  leads the route    no sensors      follows Sam
  front sonar: obstacles  (blind)    front sonar: constant gap
  rear sonar: tracks Sam
```

All three run the **same firmware**; the hub assigns each robot its role over Bluetooth, plans routes on the city map, mirrors driving commands to Sam, and narrates every step out loud through the voice interface.

## Repository layout

| Path | What it is |
|---|---|
| `src/firmware.py` | MicroPython firmware (XRPLib) — identical on all 3 robots; roles Opener / Sam / Closer, line + floor-marker navigation, ultrasonic gap-holding, telemetry |
| `docs/hub.html` | **Fleet Hub** — the coordinator web app (Web Bluetooth): connects the 3 robots, assigns roles, dispatches routes, safety stop, live monitoring |
| `docs/voice-interface.html` | **Voice interface** for Sam — speaks every step (free system voice), listens for destinations, big accessible controls |
| `docs/flowchart.html` | System process flowchart (mission logic + 5 Hz coordination loop) |
| `docs/architecture.html` | Layered software architecture diagram |
| `docs/ARCHITECTURE.md` | Architecture in text (modules, roles, control loop, build order) |
| `docs/SAM_AND_THE_GUIDES.md` | Concept one-pager |
| `docs/MARKET.md` / `docs/research_report.html` | Market & feasibility research (80 sources) |
| `docs/PRESENTATION.md` | Capstone presentation content (slides 1–11) |

## Quick start

1. **Robots** — in [XRPCode](https://xrpcode.wpi.edu/), paste `src/firmware.py`, save as `main.py` on each robot, RUN.
2. **Hub** — serve the repo (`python3 -m http.server 8779`) and open `http://localhost:8779/docs/hub.html` in desktop Chrome → Connect each robot (roles are assigned automatically).
3. **Voice** — open `docs/voice-interface.html`, tap Start, say a destination.

*Web Bluetooth needs Chrome on localhost or https. One BLE central at a time — disconnect XRPCode before connecting the hub.*

## Hardware (per robot)

XRP kit (SparkFun/WPI): 2 driven wheels with encoders, 2 line-reflectance sensors, HC-SR04 ultrasonic (front; the Opener carries a second one facing rear), servo arm (light, 3D-printed), LED, Raspberry Pi Pico W — MicroPython + XRPLib. **No camera, no microphone, no speaker** (the phone/laptop is the system's voice).
