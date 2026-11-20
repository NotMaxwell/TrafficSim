# Highway Traffic Simulator — Architecture

Nov 19, 2026 · @Maxwell

## Requirements

The app simulates a 20-mile (32.2 km) highway between City A and City B, closed into a ring so traffic never runs out of road, and runs a Human mode and an Auto mode side by side on identical inputs.

| Area | Requirement |
| --- | --- |
| Corridor | 20 mi between City A and City B, 3 lanes per direction (configurable 2 to 5) |
| Topology | Ring: eastbound 20 mi + westbound 20 mi joined at each city = 40 mi (64.4 km) closed loop |
| Ramps | On- and off-ramps at both cities in both directions; optional mid-corridor interchanges |
| Vehicle model | Continuous car-following and lane changing (IDM + MOBIL) with per-driver noise |
| Human mode | Limited perception, human reaction delay, estimation errors |
| Auto mode | Shared position and velocity for all autos, short reaction time, cooperative control |
| Comparison | Side-by-side split view, mixed traffic % slider |
| Analysis | Time-space diagrams, flow-density curve, throughput, shockwave speed |
| Events | Inject hard braking, crashes, lane closures, demand surges |

**How the ring maps to the corridor.** At each city end, most vehicles leave through the off-ramp (a sink) and the city's on-ramp feeds new vehicles into the opposite direction (a source). Only a configurable through share wraps around the end loop. This keeps density controlled by city demand while the road itself has no hard end.

## Tech stack

Decision: a native desktop app written entirely in Rust. The simulation engine is a library, the interface uses egui, the GPU drawing uses wgpu, and rayon runs the two worlds on separate CPU cores. One language, one process, one executable.

**What the requirements actually demand**

- **Raw compute on the CPU.** Up to about 46,000 vehicles across two worlds, 100 ticks per second at 10× playback, with branchy logic (lane changes, delay buffers, merges). That needs compiled code and real threads.
- **Two worlds in strict lockstep.** Easiest when both live in one process and share memory directly.
- **Bit-for-bit repeatable runs.** Fixed time step, seeded RNG, no garbage-collection pauses.
- **Live drawing of tens of thousands of moving vehicles plus live plots.** GPU rendering and a plotting library built for real-time data.
- **Not required:** running in a browser, sharing by link, or 3D.

**Options compared**

| Option | Sim speed and threads | Visuals and plots | Build complexity | Verdict |
| --- | --- | --- | --- | --- |
| Rust + egui + wgpu (native) | Compiled; the compiler rejects data races, so running two worlds in parallel is safe by construction | GPU-drawn vehicles; egui\_plot for live plots | One language, one build tool (cargo) | **Chosen** |
| C++ + Dear ImGui + ImPlot | Compiled and just as fast; threading safety is on you | ImPlot is the strongest real-time plotter of the group | CMake, manual dependency setup, memory bugs possible | Close second |
| Python + Numba + PyQt | Numba speeds up simple loops, but branchy per-vehicle logic and true multithreading are awkward | Excellent for after-the-fact analysis | Fastest to prototype | Use only for offline analysis |
| Browser (TypeScript + WebAssembly) | WebAssembly threads need Web Workers, shared-memory buffers and special server headers | WebGL is fine | Two languages plus message passing between threads | Rejected: pays real complexity for link sharing, which is not a requirement |
| Game engine (Godot, Unity) | Engines run on a variable frame clock, which fights a fixed-step, repeatable sim | 3D for free, but 3D is not needed | Large engine, sim logic squeezed into scripts | Rejected |

**Why Rust over C++.** Performance is a tie. Rust wins on the two things this project leans on most: safe parallelism (two worlds plus a UI thread) and painless builds and dependencies. C++ with Dear ImGui would also be a sound choice; [ImPlot](https://github.com/epezent/implot) is a GPU-accelerated, immediate-mode plotter geared toward real-time data.

**Terms used here**

- **Immediate-mode GUI (egui):** the whole interface is redrawn every frame from the current state. That fits a simulation whose state changes every frame, and keeps UI code short.
- **wgpu:** a Rust graphics API that runs on Vulkan, Metal or DirectX. [eframe, the egui app framework, uses wgpu for rendering by default](https://docs.rs/crate/eframe/latest).
- **rayon:** a Rust library that runs work in parallel across CPU cores with a few lines of code.
- **Triple buffer:** three copies of the render data, so the sim writes one while the UI reads another and neither waits.

**Stack**

| Layer | Choice | Role |
| --- | --- | --- |
| Project layout | Cargo workspace, 3 crates | `traffic-core` (engine), `traffic-app` (GUI), `traffic-batch` (command-line runs) |
| Sim engine | `traffic-core` | Pure step(world, dt); no I/O, no drawing |
| Parallelism | rayon | Added at milestone 6 to step both worlds at once; one world on one thread before that |
| GUI shell | eframe + egui | Panels, sliders, event buttons, camera controls |
| Vehicle drawing | egui Painter meshes first; raw wgpu only if profiling demands it | One colored rectangle per vehicle; GPU instancing later if needed |
| Plots | [egui\_plot](https://github.com/emilk/egui_plot) + a custom wgpu texture | Line and scatter plots; scrolling time-space heatmap |
| Sim to UI handoff | Mutex-swapped snapshot (triple buffer later if needed) | Simple first; upgrade only if the UI stutters |
| Randomness | rand\_chacha, one stream per purpose | Repeatable runs and fair split view |
| Scenarios | TOML files via serde | Human-editable, shared by app and CLI |
| Offline analysis (optional) | Python + pandas + matplotlib | Reads the CSV/Parquet files the CLI writes |

**Trade-off accepted.** A native app has to be installed: one executable per OS (Windows, macOS, Linux). If sharing by link ever becomes a requirement, eframe can [compile the same app for the web](https://docs.rs/crate/eframe/latest) without a redesign, but the architecture does not plan around it.

## Architecture

The sim core is a pure Rust library with no I/O. Two copies run on separate simulation threads, one per pane, synced every tick. The UI thread only sends commands and draws snapshots.

&#91;embedded content: component architecture · UI thread, 2 Workers, core modules\]

The lockstep barrier is what makes the split view a fair comparison: both worlds see the same arrivals and events, so any difference comes from the controllers.

## Road model

The road is one closed 1D coordinate s from 0 to 64,374 m (40 mi), with lanes as integer indices. A vehicle's position is (s, lane), and every gap is computed modulo the ring length, so there is no start or end to handle.

| Segment | s range (mi) | Length | Contents |
| --- | --- | --- | --- |
| Eastbound mainline | 0 to 20 | 32.2 km | 3 lanes, City A on/off ramps at mi 0, City B off-ramp near mi 20 |
| City B end loop | 20 to 20 + loop | about 0.5 km | Through traffic only; 1 to 2 lanes at lower speed limit |
| Westbound mainline | 20 to 40 | 32.2 km | 3 lanes, City B on-ramp at mi 20, City A off-ramp near mi 40 |
| City A end loop | 40 back to 0 | about 0.5 km | Through traffic only |

**Ramps** are short auxiliary lanes, each with a merge or diverge zone of about 300 m (configurable). An on-ramp vehicle must merge before the zone ends; at the end it stops and waits, which reproduces real ramp queues. An off-ramp vehicle starts moving right about 2 km before its exit and is removed at the gore point.

**Demand** is set per ramp in vehicles per hour, with a time-of-day profile (for example a morning peak from City A to City B). Spawning uses a Poisson process with a gap check, so a full ramp backs up instead of spawning cars on top of each other.

**Mid-corridor interchanges** are optional ramp pairs at any mile marker, added from the scenario file.

**Speed limits and grades** are per-segment attributes (for example 70 mph mainline, 35 mph loops). Grade can lower the effective top acceleration for trucks; it stays off by default.

**Rendering the ring.** Three camera views use the same data:

1. **Corridor strip:** both directions drawn as a straight 20-mile highway, eastbound on top, westbound below, with City A on the left and City B on the right. This is the main view and shows the highway between two cities.
2. **Follow camera:** a zoomed window that tracks one vehicle or one jam front.
3. **Ring overview:** a small minimap circle that shows the whole loop and where the waves are.

## Vehicle dynamics

Both modes share the same physics: the Intelligent Driver Model (IDM) for speed control and MOBIL for lane changes. The modes differ only in what each vehicle perceives and how fast it reacts. That keeps the comparison fair.

**Longitudinal: IDM** ([Wikipedia](https://en.wikipedia.org/wiki/Intelligent_driver_model)). Acceleration depends on own speed v, gap s to the leader, and closing speed Δv:

```latex
\dot v = a\left[1-\left(\frac{v}{v_0}\right)^{\delta}-\left(\frac{s^*(v,\Delta v)}{s}\right)^2\right],\qquad s^* = s_0 + vT + \frac{v\,\Delta v}{2\sqrt{ab}}
```

The first term is free-road acceleration toward desired speed v0. The second term brakes when the gap falls below the desired gap s\*.

**Lateral: MOBIL.** A driver changes lanes when their own acceleration gain, minus politeness p times the loss imposed on the new and old followers, exceeds a threshold. A safety check rejects any change that would force the new follower to brake harder than b\_safe. A keep-right bias moves slower vehicles back to the right lane.

**Baseline parameters** (highway values from [Treiber's set, as used in arXiv:2406.15511](https://arxiv.org/pdf/2406.15511)):

| Parameter | Symbol | Human baseline | Auto baseline |
| --- | --- | --- | --- |
| Desired speed | v0 | 120 km/h (75 mph), per-driver spread | Speed limit, no spread |
| Desired time gap | T | 1.5 s | 0.6 s (CACC) |
| Jam distance | s0 | 2.0 m | 2.0 m |
| Max acceleration | a | 1.4 m/s² | 1.4 m/s² |
| Comfortable deceleration | b | 2.0 m/s² | 2.0 m/s² |
| Acceleration exponent | δ | 4 | 4 |
| MOBIL politeness | p | 0 to 0.5, per driver | 0.5 (cooperative) |
| MOBIL threshold | Δa\_th | 0.2 m/s² | 0.1 m/s² |
| Max safe braking | b\_safe | 4 m/s² | 4 m/s² |

The auto 0.6 s gap matches the shortest CACC setting drivers chose in [FHWA's CACC human factors study](https://www.fhwa.dot.gov/publications/research/safety/17025/005.cfm). Trucks get their own row (longer length, lower a and v0).

**Integration:** fixed step dt = 0.1 s with the ballistic update: x += v·dt + ½·a·dt², v += a·dt, with v clamped at 0. This avoids negative speeds and stays stable at the step sizes needed for real-time play.

## Human mode

Human drivers run IDM + MOBIL on delayed, noisy, partial inputs. This follows the Human Driver Model of [Treiber, Kesting and Helbing (2006)](https://arxiv.org/abs/cond-mat/0404736), which adds finite reaction times, estimation errors, and looking several vehicles ahead. That paper shows multi-anticipation enlarges the space and time scales of stop-and-go waves, in line with real traffic data.

| Component | What it models | Default (tunable) |
| --- | --- | --- |
| Reaction delay τ | Time from cue to action | Per driver, lognormal, median 1.1 s, clipped 0.7 to 2.5 s |
| Perception range | How far the driver can see | 1 to 3 leaders in own lane, within 200 m |
| Occlusion | Trucks block the view | A car behind a truck sees only the truck |
| Mirror range | Lane-change awareness | Adjacent lanes, 60 m behind, 100 m ahead |
| Brake-light cue | How a leader's braking is seen | Binary brake light, not the leader's actual deceleration |
| Estimation error | Misjudging gap and closing speed | Wiener process, about 5% of the gap, correlation time about 20 s |
| Driver spread | Differences between drivers | v0 ±10%, T from 1.0 to 2.0 s, politeness p from 0 to 0.5 |
| Distraction | Eyes off the road | Poisson, about once per 5 min, 1 to 3 s with perception frozen |
| Action points | Not every tiny change gets a response | Update acceleration only when the perceived change crosses a threshold (optional) |

The reaction delay range comes from [Green (2000)](https://www.informahealthcare.com/doi/abs/10.1207/STHF0203_1): about 0.70 to 0.75 s when the driver expects the signal, about 1.25 s for a lead car's brake lights, and about 1.5 s for surprise events.

**Implementation.** Each human vehicle keeps a ring buffer of its perceived inputs (gap, leader speed, own speed) at every 0.1 s tick, up to 25 entries for a 2.5 s ceiling. The controller reads the entry from t − τ, with linear interpolation between ticks. Memory is about 300 bytes per vehicle, or about 2 MB for 7,000 humans.

**Crashes are allowed.** With long delays and distraction, a human can close a gap to zero. The sim records a collision, freezes both vehicles as a blockage, and logs it as an emergent event. This is where Human and Auto modes diverge most visibly.

## Auto mode (hive mind)

Auto vehicles share one state table: every auto knows every other auto's position, speed, acceleration and intent, slightly delayed by communication. They react in about 0.2 s instead of about 1 s, and they plan for traffic kilometers ahead instead of only the car in front.

**Shared State Bus.** Each tick, every auto publishes {id, s, lane, v, a, intent, timestamp}. Readers see the snapshot from latency L ago, so the bus is double-buffered. A spatial index of 500 m bins makes "what is happening 2 km ahead" a cheap lookup.

| Bus setting | Default | Purpose |
| --- | --- | --- |
| Latency L | 100 ms | V2V message delay |
| Packet loss | 0% | Raise it to test degraded comms |
| Broadcast range | Whole corridor | Lower it to model local-only V2V |
| Outage event | Off | Event that kills the bus to show fallback |

**Control stack** (each layer adjusts the one below):

1. **CACC car-following.** Commanded acceleration = k\_p × gap error + k\_d × speed error + k\_ff × leader acceleration (from the bus). The time gap is 0.6 s. The feed-forward term is what makes a platoon string-stable: a disturbance shrinks as it travels back instead of growing. [Analysis of CACC controllers](https://arxiv.org/pdf/2103.13830) finds smaller time gaps and larger delays and actuator lags lead to string instability, which is why latency is a user setting.
2. **ACC fallback.** If the leader is a human, or the bus is down, the auto drops to sensor-only ACC: 1.1 s gap, no feed-forward, 200 m radar range.
3. **Wave look-ahead.** Each auto reads mean speed in downstream bins up to 2 km ahead. If a slowdown is forming, it eases toward that speed early and opens a buffer gap, absorbing the wave instead of passing it on. [Stern et al. (2018)](https://arxiv.org/pdf/1705.01693.pdf) showed this works in the field: on a ring with more than 20 vehicles, controlling one vehicle's speed dampened the stop-and-go waves.
4. **Merge coordinator.** One coordinator per ramp zone gives each merging auto a slot. Mainline autos ahead of the slot open a gap (zipper merge), and lane-change intents go out on the bus so neighbors yield in advance.
5. **Speed harmonization (optional).** A corridor-wide controller sets a target speed per segment from bus data, like variable speed limits.

**Actuation.** A first-order lag (time constant 0.2 s, tunable) sits between commanded and actual acceleration, so autos are fast but not instant.

**Humans in the hive's view.** Autos see humans only through their own sensors; humans are never on the bus. An optional "roadside sensing" toggle publishes human positions too, to show what infrastructure could add.

## Split view, mixed traffic and events

The left and right panes are two independent worlds stepped in lockstep. They share the same seed, demand and event schedule, so the only difference is the controller mix you set for each side.

**Split view.** Each pane has its own auto-share slider (default: left 0% autos, right 100% autos). Each tick, both worlds step in parallel on separate CPU cores, and the next tick starts only when both finish, so time and events line up exactly. Plots overlay both worlds in two colors.

**Common random numbers.** Vehicle arrivals, driver traits and noise come from separate RNG streams keyed by vehicle id. Vehicle #4,812 enters at the same second with the same personality in both worlds. Only its controller differs if the mix makes it an auto on one side.

**Mixed traffic %.** At spawn, each vehicle becomes an auto with probability equal to the slider. Changing the slider mid-run only affects new arrivals, so the shift spreads through the corridor over time. Autos are drawn in a distinct color.

**Event injection.** Events go into a shared schedule, so they hit both worlds at the same time and place. Click the road or use the event panel.

| Event | Parameters | What it shows |
| --- | --- | --- |
| Hard brake | Vehicle or location, target speed, duration | Phantom-jam trigger; does the wave grow or die out |
| Crash | Location, lanes blocked, clear time | Capacity drop, queue growth, rubbernecking slowdown in the other direction (optional) |
| Lane closure | Start and end mile, lanes, duration | Work zone merge behavior |
| Demand surge | Ramp, extra vehicles per hour, duration | Rush hour or an event letting out in one city |
| Weather | Segment, speed and gap multipliers | Rain or fog: lower v0, longer T, longer human τ |
| Comms outage | Segment or corridor, duration | Auto fallback to ACC (Auto mode only) |

Events and the slider settings save into the scenario file, so a demo can be replayed exactly.

## Metrics and plots

All metrics come from two collectors inside the sim core: virtual loop detectors (fixed points that count passing vehicles) and a space-time grid sampler. The frontend only draws what they produce.

| Output | How it is computed | Plot |
| --- | --- | --- |
| Time-space diagram | Mean speed per 100 m × 5 s cell, per direction | Scrolling heatmap texture; jams show as diagonal bands moving upstream |
| Fundamental diagram | Flow and density from each detector over 1 min windows | Scatter of flow (veh/h/lane) vs density (veh/km/lane) |
| Throughput | Vehicles reaching each city's off-ramp per hour | Line over time, both worlds overlaid |
| Travel time | City A to City B per vehicle, from tags at spawn | Line (median, 90th percentile) + histogram |
| Shockwave speed | Cross-correlation of speed series at neighboring detectors | Number + trend; real waves run about 15 km/h upstream |
| Wave amplification | Speed dip at the trigger vs 2 km upstream | Ratio; above 1 = unstable (human), below 1 = damped (auto) |
| Braking events | Count of decelerations harder than 3 m/s² | Bar per 5 min |
| Collisions | Count and location | Markers on the corridor and time-space diagram |
| Fuel proxy | Sum of positive acceleration × speed | Relative bar, Human vs Auto |

The shockwave check is the key realism test. Empirical stop-and-go waves travel upstream at about 15 km/h, with typical values of 15 ± 5 km/h depending on the country ([Treiber and Kesting, Agents for Traffic Simulation](https://arxiv.org/pdf/0805.0300)).

**Export.** Every metric streams to a ring buffer; a button saves CSV. The native CLI writes the same series for parameter sweeps (for example travel time vs auto share from 0% to 100% in 10% steps).

## Data layout, tick loop and performance

Vehicles are stored as a struct of arrays (one array per field) with a per-lane index sorted by position. Leaders and followers are neighbors in that index, so finding them costs O(1) per vehicle.

**Vehicle store (SoA)**

- Hot, every tick: `s`, `v`, `a`, `lane`, `length`, `controller_kind`
- Per-driver params: `v0`, `T`, `a_max`, `b`, `politeness`, `tau`
- Human-only: perception ring buffer, noise state, distraction timer
- Auto-only: bus slot, actuator state, assigned merge slot
- Lane index: `Vec<u32>` of vehicle ids per lane, sorted by `s`. Order barely changes per tick, so an insertion sort pass keeps it sorted in near O(N).

**Tick order** (dt = 0.1 s, identical in both worlds):

1. Spawn from ramp queues; remove vehicles at off-ramp gores.
2. Publish auto states to the bus write buffer; swap buffers per latency.
3. Perceive: humans push noisy, occluded inputs into their delay buffer; autos read the bus plus sensors.
4. Decide lanes: MOBIL with the safety check, on a staggered schedule (each vehicle checks every 1 s, offset by id) to avoid two cars taking the same gap.
5. Compute acceleration: delayed IDM (human) or CACC/ACC + look-ahead + actuator lag (auto).
6. Integrate with the ballistic update; apply lane changes; re-sort lane indices.
7. Detect collisions; apply scheduled events.
8. Sample detectors and the space-time grid.
9. Every frame (not every tick): copy `s`, `lane`, `v`, `kind` into the shared snapshot the UI thread draws from.

**Performance budget**

| Quantity | Estimate |
| --- | --- |
| Vehicles per world, rush hour (about 35 veh/km/lane × 64.4 km × 3 lanes) | about 6,800 |
| Vehicles per world, worst-case jam | about 23,000 |
| Two worlds, worst case | about 46,000 |
| Ticks per second at 10× real time | 100 |
| Vehicle-updates per second, worst case | about 4.6 million |
| Render payload per frame | 16 bytes × 46,000 = about 740 KB, handed to the UI as one snapshot |

About 4.6 million IDM-style updates per second is light work for compiled code, about one CPU core per world at most. With each world on its own core, faster playback or a longer corridor mostly costs more cores, not a redesign.

## Building with AI

An AI assistant writes most of the code. The design compensates for limited line-by-line review with three safeguards: the Rust compiler catches memory and threading bugs, automated validation tests catch wrong physics, and small milestones keep any mistake contained.

**Rules for every AI session**

1. **One milestone per session.** Paste the milestone's goal, its gate tests and the relevant section of this doc. Never ask for the whole app at once.
2. **Tests before features.** The AI writes the milestone's gate tests first, confirms they fail, then writes code until they pass.
3. **A failing test means broken code**, even if the animation looks right. Never let the AI weaken or delete a test to make it pass without a stated physics reason.
4. **Commit after every green milestone.** `git commit` once `cargo test` passes, so a bad session can be rolled back with one command.
5. **The engine never imports GUI code.** `traffic-core` has no egui, no files, no printing. This keeps it testable and lets the AI change the UI without touching physics.
6. **One concept per file**, under about 400 lines. Small files fit in the AI's context and keep each change easy to review.
7. **Compiler errors go back to the AI verbatim.** Paste the full `cargo build` output rather than describing it.

**Version pinning.** egui, eframe and wgpu release breaking changes often, and AI models tend to write code for older versions. Pin exact versions in `Cargo.toml` (for example `egui = "=0.x.y"`) and keep a `docs/deps.md` noting the pinned versions with links to their docs and examples. When the AI writes code that does not compile against them, give it the current example code for that version.

**Keep the stack small until a test or profiler says otherwise**

| Start with | Upgrade to | Only when |
| --- | --- | --- |
| One world on one background thread | Two worlds stepped in parallel with rayon | Milestone 6 (split view) |
| Mutex-swapped render snapshot | Lock-free triple buffer | UI visibly stutters while the sim runs |
| egui Painter rectangles | Custom wgpu instanced renderer | Frame rate drops below 30 fps at target vehicle count |
| Single-threaded world step | Lane-parallel step with rayon | One tick takes more than 10 ms in a release build |

**Quality gates the AI must run before calling a milestone done**

- `cargo build --release` with zero warnings
- `cargo clippy -- -D warnings` (Rust's linter, warnings treated as errors)
- `cargo test` all green, including every earlier milestone's tests
- A short screen recording or screenshot of the app at that milestone, saved in `docs/progress/`

**Prompt template for each milestone**

```markdown
Context: Rust traffic simulator. Read docs/design.md sections <X, Y>.
Milestone <N>: <goal>.
Gate tests (write these first, in crates/traffic-core/tests/): <list>.
Constraints: pinned versions in Cargo.toml; traffic-core has no GUI or I/O;
files under ~400 lines; no unsafe code.
Done when: cargo build --release, cargo clippy -- -D warnings and cargo test all pass.
```

## Repository layout

Three crates in one Cargo workspace. Each engine file owns one concept from this doc and has a matching test file, so an AI session can be pointed at exactly the files a milestone touches.

```text
traffic-sim/
├── Cargo.toml                 # workspace + pinned dependency versions
├── docs/
│   ├── design.md              # this document, exported
│   ├── deps.md                # pinned versions + doc links
│   └── progress/              # screenshot per milestone
├── scenarios/
│   ├── sugiyama_ring.toml     # 22 cars, 230 m ring (validation)
│   ├── single_av_ring.toml    # Stern-style damping test
│   └── city_a_to_b.toml       # the 20-mile corridor
├── crates/
│   ├── traffic-core/          # engine: no GUI, no files, no printing
│   │   ├── src/
│   │   │   ├── lib.rs         # World, step(world, dt)
│   │   │   ├── road.rs        # ring geometry, lanes, ramps, merge zones
│   │   │   ├── vehicles.rs    # SoA store, lane-sorted index
│   │   │   ├── idm.rs         # IDM acceleration
│   │   │   ├── mobil.rs       # lane-change decision
│   │   │   ├── human.rs       # delay buffer, perception, noise, distraction
│   │   │   ├── auto.rs        # CACC, ACC fallback, look-ahead
│   │   │   ├── bus.rs         # shared state bus with latency/loss
│   │   │   ├── merge.rs       # ramp merge coordinator
│   │   │   ├── demand.rs      # spawning and removal at ramps
│   │   │   ├── events.rs      # scheduled brakes, crashes, closures
│   │   │   ├── metrics.rs     # detectors, space-time grid
│   │   │   ├── rng.rs         # per-purpose ChaCha streams
│   │   │   └── scenario.rs    # TOML -> World (serde)
│   │   └── tests/             # one file per milestone gate
│   ├── traffic-app/           # eframe GUI
│   │   └── src/
│   │       ├── main.rs        # window, panels, controls
│   │       ├── runner.rs      # sim thread(s), snapshot handoff
│   │       ├── view_road.rs   # corridor strip, follow cam, minimap
│   │       └── view_plots.rs  # egui_plot charts, time-space heatmap
│   └── traffic-batch/         # headless CLI: run scenario, write CSV
│       └── src/main.rs
└── analysis/                  # optional Python notebooks reading CSV
```

## Validation, milestones and open questions

**Validation scenarios** (each a saved scenario file and an automated test run by cargo test):

| Test | Setup | Pass condition |
| --- | --- | --- |
| Phantom jam | 22 human cars on a 230 m single-lane ring, as in [Sugiyama et al. (2008)](https://searchworks.jioinstitute.edu.in/articles/edb__42225032) | A jam forms with no bottleneck and moves backward |
| Single-AV damping | 20+ cars on a ring, one auto with look-ahead, as in [Stern et al. (2018)](https://arxiv.org/pdf/1705.01693.pdf) | Wave amplitude falls once the auto is switched on |
| Shockwave speed | Full corridor, hard-brake event at rush-hour density | Upstream wave speed in the 10 to 20 km/h range |
| String stability | 100% autos, hard-brake event | Wave amplification below 1 |
| Determinism | Same scenario run twice | Bit-identical metrics |
| Fairness | Split view with both sides at 0% autos | Identical results in both panes |

**Milestones**

Each milestone is one AI session (or a few). It is done only when its gate tests and every earlier gate pass. Physics comes before graphics, so the engine is proven correct before anything is drawn.

| # | Milestone | What gets built | Gate tests (must pass) |
| --- | --- | --- | --- |
| 1 | Skeleton | Workspace, pinned deps, empty eframe window, `check.sh` running build + clippy + test | Builds with zero warnings; clippy clean |
| 2 | Ring + IDM (headless) | Single-lane ring, IDM, ballistic integration, seeded RNG | Sugiyama phantom jam forms; same seed gives identical output; speeds never negative; no overlaps |
| 3 | Viewer | Sim on a background thread, ring drawn with egui Painter, play/pause/speed controls | UI stays responsive at 10×; screenshot saved |
| 4 | Corridor | Multi-lane MOBIL, 40 mi ring, city ramps, demand profiles, corridor strip view | Vehicle count conserved (spawned = removed + present); no lane change forces braking beyond b\_safe; free-flow ramp throughput within 5% of demand |
| 5 | Human mode | Delay buffers, perception limits, estimation noise, distraction, collisions | Braking response lags by τ; wave amplification above 1 at rush-hour density; shockwave speed 10 to 20 km/h upstream |
| 6 | Split view | Two worlds stepped in parallel with rayon, shared seed and event schedule | Identical settings give identical panes; parallel run matches serial run bit for bit |
| 7 | Auto mode | State bus, CACC, ACC fallback, look-ahead | Amplification below 1 at 100% autos; single-auto ring damps waves; comms outage causes no collisions |
| 8 | Merges + mix | Merge coordinator, mixed-traffic slider | Zero merge collisions at 100% autos; auto share of spawns within 2% of slider over 10,000 vehicles |
| 9 | Events + analysis | Event panel, detectors, time-space heatmap, fundamental diagram, CSV export | Crash drops throughput, which recovers after clearing; detector counts match vehicle passages; CSV round-trips |
| 10 | Batch + polish | `traffic-batch` sweeps, scenario save/load, follow camera, minimap | 0% to 100% auto-share sweep completes headless; save then load reproduces the run |

**Open questions**

- [ ] Are there intermediate interchanges between the cities, and at which mile markers?
- [ ] Lanes per direction: keep 3, or match a specific real highway?
- [ ] What share of traffic is trucks?
- [ ] Should the City B end loop carry through traffic, or should every vehicle exit and respawn?
- [ ] Which OS will the demo machine run (Windows, macOS, Linux), and is it allowed to run an installed program?

## Sources

- [Treiber, Kesting, Helbing (2006), Delays, Inaccuracies and Anticipation in Microscopic Traffic Models](https://arxiv.org/abs/cond-mat/0404736)
- [Green (2000), How Long Does It Take to Stop?](https://www.informahealthcare.com/doi/abs/10.1207/STHF0203_1)
- [Sugiyama et al. (2008), Traffic jams without bottlenecks](https://searchworks.jioinstitute.edu.in/articles/edb__42225032)
- [Stern et al. (2018), Dissipation of stop-and-go waves via control of autonomous vehicles](https://arxiv.org/pdf/1705.01693.pdf)
- [Treiber and Kesting, Agents for Traffic Simulation](https://arxiv.org/pdf/0805.0300)
- [IDM parameter set (arXiv:2406.15511)](https://arxiv.org/pdf/2406.15511)
- [Intelligent driver model, Wikipedia](https://en.wikipedia.org/wiki/Intelligent_driver_model)
- [FHWA, CACC Human Factors Study (2017)](https://www.fhwa.dot.gov/publications/research/safety/17025/005.cfm)
- [Multi-Objective H∞ Control for String Stability of CACC (arXiv:2103.13830)](https://arxiv.org/pdf/2103.13830)
- [eframe documentation (docs.rs)](https://docs.rs/crate/eframe/latest)
- [egui\_plot](https://github.com/emilk/egui_plot)
- [ImPlot](https://github.com/epezent/implot)
