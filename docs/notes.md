# Implementation notes

What was built against [`design.md`](design.md), where it deviates, and
why. Every deviation came from a failing test or a measured problem, and
the evidence is recorded here.

## Milestones

| # | Milestone | Gate tests | Screenshot |
| --- | --- | --- | --- |
| 1 | Skeleton | `check.sh` (build, clippy `-D warnings`, test) | – |
| 2 | Ring + IDM | `tests/m2_ring_idm.rs` | – |
| 3 | Viewer | UI at 10× without stutter | `progress/m3_viewer.png` |
| 4 | Corridor | `tests/m4_corridor.rs` | `progress/m4_corridor.png` |
| 5 | Human mode | `tests/m5_human.rs` | `progress/m5_human.png` |
| 6 | Split view | `tests/m6_split.rs` | `progress/m6_split.png` |
| 7 | Auto mode | `tests/m7_auto.rs` | `progress/m7_auto.png` |
| 8 | Merges + mix | `tests/m8_merge_mix.rs` | `progress/m8_single_av_ring.png` |
| 9 | Events + analysis | `tests/m9_events_analysis.rs` | `progress/m9_events_analysis.png` |
| 10 | Batch + polish | `tests/m10_batch_replay.rs`, `traffic-batch/tests/cli.rs` | – |

## Deviations from the design

### Road

- **City ends are freeway terminals** (`city_terminal = true`). A single
  auxiliary-lane ramp carries about 1,800–2,000 veh/h, so City A's
  4,500 veh/h morning peak would gridlock at the ramp. At the city ends
  vehicles therefore enter and leave on any mainline lane. Mid-corridor
  interchanges use the designed auxiliary lanes, merge zones and ramp
  queues. Set `city_terminal = false` to make the city ramps ordinary
  ramps.
- **Interchanges at miles 7 and 13** in the bundled corridor, as an
  example; the open question is still open (`interchanges_mi`).
- Lanes are numbered from the right: lane 1 is the rightmost mainline
  lane and lane 0 the auxiliary lane.

### Human drivers

The design's Human Driver Model components are all implemented. Five
additions were needed to keep crash rates believable. Before them the
corridor had about 110 crashes per hour at rush hour; after them, about
one in two hours.

1. **Temporal anticipation (HDM).** Treiber et al. (2006) has drivers
   extrapolate their delayed view forward over their reaction time,
   assuming constant speeds. Without it, single-leader IDM with τ > T/2
   produced rear-end crashes within a minute on a ring.
2. **Watching 2 to 3 leaders** (design: 1 to 3). Every pile-up traced
   back to a single-leader driver. With two or more leaders the ring runs
   crash-free, and stop-and-go waves remain (speed SD ≈ 3.5 m/s).
3. **Time gap ≥ reaction time** (`gap_per_tau = 1.0`). Drivers who react
   slowly keep longer headways.
4. **Self-regulated distraction.** Drivers look away only when the car
   ahead is more than 2 s away and not closing. Peripheral vision ends
   the distraction when the leader looms (time to collision < 3 s), and
   reaction delay still applies after that.
5. **Lane-change perception.** Mirrors still have the 60 m range, but a
   car closing fast is noticed if it would arrive within 8 s (up to
   200 m). Nearly every cut-in crash was a stopped car entering a lane
   where traffic approached at 25 m/s from just beyond 60 m. Drivers
   **signal** for 2 s (autos 1 s) before changing lanes and keep
   signalling while they wait for a gap. Drivers behind **yield** to a
   signalling car just ahead when that takes only comfortable braking.

Measured with these settings: reaction lag = τ exactly, wave
amplification 1.51 at rush-hour density, jam head 13.8 km/h upstream.

### Automated vehicles

- **CACC** follows Ploeg et al.: h·u̇ = −u + kp·e + kd·ė + kff·u_leader,
  with kp = 0.2, kd = 0.7, h = 0.6 s, a 0.1 s bus delay and a 0.2 s
  actuator lag. Two safety layers sit on top. An IDM envelope takes over
  when the gap is under half the desired gap (cut-ins, merges).
  Emergency braking stops within the gap plus the leader's own stopping
  distance when the leader brakes.
- **Overrides go on the bus.** While a hard-brake event controls an
  auto, it broadcasts its real acceleration, not its CACC command. This
  was found by the comms-outage test.
- **The Stern-style dampener is off by default.** It holds a vehicle near
  its 30 s average speed behind an oscillating human. That dissolves ring
  waves (one auto takes ring speed SD from 1.21 to 0.13 m/s). On the
  corridor, though, it lowers bottleneck discharge: at 30% autos mean
  speed fell to 54 km/h versus 91 km/h with no autos at all. With it off,
  results improve steadily with auto share. `single_av_ring.toml` turns
  it on (`[auto] dampener = true`).
- **Merge coordinator.** Each mainline auto opens a gap for the first
  signalling merger directly ahead of it. A merging auto drops back
  behind a vehicle alongside it unless that vehicle is the one assigned
  to yield, which prevents a mutual-yield deadlock found by the gate
  test.

### Split view and replay

- The split view is two worlds in one `Sim`, stepped with `rayon::join`
  each tick. One sim thread runs the lockstep, rather than two worker
  threads plus a barrier.
- Slider changes are recorded in `mix_timeline` and injected events in
  `events`, so a saved scenario replays a run bit for bit.

### Metrics

- Detectors sit at miles 0.5, 1.5, …, 19.5 in each direction (20 per
  direction), aggregated over 60 s windows.
- The fuel proxy is positive work per vehicle-km (Σ max(a, 0)·v·dt /
  km); compare it only between panes.
- Shockwave speed comes from cross-correlating speed series 1 km apart
  on the space-time grid (cells with traffic slower than 12 m/s only).

## Answers assumed for the design's open questions

| Question | Default used |
| --- | --- |
| Intermediate interchanges | Miles 7 and 13 (configurable) |
| Lanes per direction | 3 (2 to 6 supported) |
| Truck share | 8% |
| City B end loop | Carries the 10% through share; everyone else exits |
| Demo OS | Developed and tested on Linux (Wayland); eframe/wgpu builds on Windows and macOS |
