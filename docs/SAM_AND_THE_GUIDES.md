# Sam & the Guides
### *Three robots, one piece of software — getting a blind traveler anywhere, safely.*

---

## The problem
One in four people lives with vision impairment. Moving through a space you can't see means depending on someone else. A white cane only reveals what's right in front of you. A guide dog costs a fortune and takes years to train. Independence shouldn't be that expensive.

## The idea
**Sam** is a **blind person** who wants to get around on their own. Sam is led by **two guide robots** that act as their eyes and a **companion robot** that travels right beside them — all orchestrated by a single piece of **software**, the brain of the system.

*In our live demo, the companion robot **represents Sam** (we can't put a real blind person on stage); the two guide robots lead it exactly as they would lead the person.*

## The software is the product
The robots are just hands and eyes. Everything intelligent lives in the app:

- **🗺️ The map** — it knows the routes and the destinations.
- **🎙️ The interface** — the blind user speaks or types: *"Take me to the store."*
- **🧮 Planning & ETA** — it computes the route length ÷ robot speed to know exactly how long the trip takes, so the user can say *"I want to be there at 6:00 PM"* and the app answers *"Leaving at 5:52"* — and launches the mission on time by itself.
- **🔗 Real-time coordination** — it reads all three robots' telemetry over Bluetooth ~5×/second and constantly re-balances their **speed and spacing**.
- **🛡️ Safety** — it stops everything on an obstacle, keeps the formation tight, and makes sure no one is ever left behind.
- **👨‍👩‍👧 Family view** — a private live link lets a relative anywhere follow the whole journey in real time.

## How it works — with no camera
1. **Sam** opens the **phone app** and says where they want to go.
2. On a **pre-mapped floor** (routes marked as lines the robots follow), the two guide robots **lead the way** — one scouting the path ahead, one staying alongside as a guard-rail.
3. The **companion robot** stays right beside Sam, locking onto the guides with its ultrasonic sensor to hold a safe, steady distance — so Sam is always led.
4. The software **coordinates the trio**: if the companion (and Sam) falls behind, it tells the lead guide to slow down; everyone slows in turns and near obstacles — the formation moves as one.
5. The companion's **front sensor** halts the group if something blocks the path, its **light arm taps** a gentle "stop / turn" cue, and the **phone narrates** every step: *"Turning right… obstacle ahead… arriving."*

## Real-time coordination, in plain terms
```
Lead guide → "speed 45%, line OK"        ┐
Companion  → "gap to guide = 30 cm"       ├─→  SOFTWARE  →  "Sam is dropping back:
Guard      → "clear on the side"          ┘                  slow the lead, hold pace"
```
Nobody gets lost. The slowest member (Sam) sets the pace.

## Why it matters
No camera. No internet. No expensive hardware. Just three small robots and a phone, giving a blind person something priceless: **the freedom to go where they want, when they want — safely, and on their own terms.**

*Accessibility × swarm robotics × smart software.*
