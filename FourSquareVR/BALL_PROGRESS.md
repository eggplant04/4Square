# Multiplayer VR Handball: development progress

Last updated: 30 October 2026, Australia/Sydney. Workspace: `S:/Github/4Square/FourSquareVR`.

## Current stage and approval state

**Stage 4 C++ implementation is complete. Stop here for user review. Stages 5-9 are unapproved and have not been implemented.** UE 5.6.1 Win64 Development Editor builds successfully; **91 automated tests executed and passed**, zero failures/warnings/not-run. A 20-second rendered PIE automation run passed with the existing VR Pawn, diagnostic moving palms and the saved ball/court. Human Editor acceptance, physical VR calibration/interaction quality and Quest qualification remain outstanding; rendered automation is not manual acceptance.

The complete specification, plan and progress file were read before gameplay changes. `BALL_SYSTEM_SPEC.md` remains unchanged and is the source of truth. Its expected SHA-256 is `3C37DC3C28F1E0474BED79F1A327AB333404A55201797F670238C8707948CE79`.

Git remains usable through approved read-only access to repository root `S:/Github/4Square`, above the sandboxed project. Starting HEAD was `1188085` (user's updated test map). No staging, commits, checkout, reset, clean, branch/config changes or destructive Git operations were performed. Changes remain uncommitted. The user's saved VRTemplateMap, Blueprint Pawn, Grip/Aim controllers, teleport/grab/input code, Steam/Advanced Sessions/OpenXR configuration, `.uproject`, targets and `.ini` files were preserved. Build.cs adds only the required public `HeadMountedDisplay` dependency; existing private `PhysicsCore` still supplies read-only material metadata. No Chaos rigid body controls the ball.

## Stage 3 entry investigation and corrections

1. Prior `EXCEPTION_INT_DIVIDE_BY_ZERO`: exported Engine DLL RVAs locate `FSceneViewport::EnqueueBeginRenderFrame` in the hit-proxy/ActorFactory path. Installed `SceneViewport.cpp` contains a buffered-render-target modulo that is invalid with no initialized Slate render handles in this NullRHI workflow. Engine PDBs are not installed, so this is source/export/call-stack evidence rather than a symbolized debugger session. Rendered RHI ActorFactory placement of stock Actor, BallTestCourt and HandballBall succeeded. The current user map loads and plays; no corrupt gameplay asset was identified. Evidence: `Saved/Automation/BallStage4/EngineExportLookup.json`, prior Stage 3 TestLevelSetup log and Stage 4 rendered logs.
2. Real PIE exposed a startup integration defect: a long first Editor frame could publish no gameplay or eight steps and permanently overrun. Before the first completed state only, the coordinator now declares/counts a startup pause, waits for three frames fitting its catch-up allowance, and establishes a fresh frame-zero timebase. It never fabricates gameplay steps. Runtime overload after gameplay still pauses explicitly, retains debt and requires a new declared recovery boundary.
3. A new natural-drop regression exposed a real low-speed limit cycle missed by initially-supported rest fixtures: a default 520 cm drop still bounced after 60 simulated seconds. The old full pre-drift gravity kick reflected force that belonged after impact. Response now uses force-stage velocity interpolated to impact time, recomputes remaining forces, and projects force-driven re-entry as sustained support without extra restitution/events. Fixed dt and collision-free semi-implicit behavior remain intact. The 60-second drop now rests at Z=3.020000430 cm, Vz=0. This correction was documented in the plan before implementation.
4. History writes were moved outside a `check(...)` expression so builds with checks disabled still execute `History.Push`. Shipping itself was not built.

Entry verification: Editor build succeeded and all **54 tests passed** (52 previous cases plus actual registered ticks/startup and natural drop). Real rendered user-map PIE ran 20 seconds, 12 observed bounces, stable rest and maximum transform error 0.0000007693 cm. Evidence: `IntegrationFixed/index.json`, `Stage3RenderedEditorValidation.json`, `Stage3RenderedSettlingFixed.log` under the Stage 4 Saved directories. Confirmed integration defects were corrected before substantial hand work.

## Stage 4 functionality implemented

- Native `UVRHandCollisionComponent` references the existing left/right Grip controllers, camera and VROrigin. Discovery requires unique **motion sources** LeftGrip/RightGrip, ignores Aim, validates ownership/attachment/registration and offers explicit Blueprint registration. Runtime automatic attachment is opt-in via the ball's default-enabled `bAutoRegisterVRHands`; existing manually added components are reused. Only locally controlled player Pawns capture physical tracking. Possession changes release/reacquire capture ownership. The diagnostic rig uses an explicit scripted-tracking flag.
- Controller/head/origin observations carry monotonic capture time, separately mapped simulation timebase, player/side, sequence, tracking/context/configuration versions, WorldToMeters, world transforms and derived linear/angular velocities. No use of Unreal component velocity for hand strikes. Configurable independent palm radius/offset/material; center includes rotated controller-local palm offset.
- Bounded 256-sample raw rings; timestamp interpolation and shortest-arc orientation interpolation. Offset arcs subdivide until default <=0.02 cm chord error, at most eight segments per collider/frame. Missing future translation uses default <=1/60 s extrapolation; 8 ms exponential filtering adds at most 0.1 cm deviation from raw constant-velocity extrapolation. Known paths and collision-response velocity always agree. Extrapolation expiry clips at an intra-step lifetime boundary.
- Rejection of malformed/nonfinite/nonunit/out-of-order/stale poses, tiny deltas, excessive velocity/acceleration/head movement/reach. Loss/reacquisition, origin movement, recenter notification, world scale changes, rebindings and live tuning establish new continuity. Two coherent observations are required before new trajectories. Shape/profile versions prevent changing radius/material/offset from reinterpreting older samples, including late-frame Blueprint changes. Explicit discontinuity hooks cover runtimes that do not broadcast recentering.
- Pure moving-sphere relative CCD with TOI, normals/contact points and response against relative surface velocity. A moving palm strikes a stationary or sleeping supported ball. Sphere offset rotation is included once; optional surface spin adds tangential motion around the palm center. Existing restitution/friction/energy handling is reused.
- `FHistoricalBallCollisionWorld` combines immutable static engine geometry and recorded moving palms/proxies in one remaining-time solver. Physics clips at all motion knots and lifetime boundaries without changing 120 Hz dt or applying extra force kicks. Relative travel determines tie tolerance; world and palm impacts can both occur inside one fixed step.
- Canonical checkpointed dynamic-pair episodes suppress repeated restitution/events while preserving inelastic physical push/support. Spatial release defaults to 0.1 cm, at least twice skin. Unknown/new-lifetime initial overlaps or positions inside the contact-release band are quarantined until separation, preventing invented past strikes and large MTD pushes through geometry. Static constraints take priority; physically infeasible moving constraints are rejected in stable collider order until separation. This deterministic bias is explicit and must remain independent of packet arrival order in future adjudication.
- Shared immutable numerical motion-frame history: match epoch, frame, timebase/revision, environment revision, dt, shape/material/profile versions, piecewise motion, lifetime fractions and raw sample provenance. One frame is shared by all balls. All-ball preparation precedes history/state commit; missing dependencies, manifold/history capacity and iteration failures are atomic. Replay preflights the entire range and uses historical paths rather than current transforms.
- Complete ball-state codec **schema 3** persists twelve dynamic-pair records including armed/blocked reason, episode and event counter, and reads schemas 1/2. Static IDs reserve namespaces for hands/kinematics; old-ID saved values require a fresh simulation baseline before resuming a world. Same-platform complete-state and regenerated event-trace equality are tested; no cross-platform bitwise guarantee.
- `IBallRewindableCollider` and `UBallKinematicColliderComponent` record opted-in movable, unit-scale, nonsimulating spheres and fixed-orientation translating boxes. Rotating boxes are explicitly rejected. Recorded orientation comes from historical samples; destroyed/rotated live sources cannot change old frames.
- Actual engine world-origin rebasing translates the ball once and establishes a fresh environment/timebase/generation with incompatible histories cleared before further simulation. Pawn teleport affects hand continuity while the ball remains in its own world state.
- Optional `ball.Debug`, `ball.DebugHands`, per-instance drawing, palm shapes/current and previous endpoints/sweeps, estimated velocities/status, ball sphere, contact normals/points, event IDs and overlap/conflict reasons. Disabled debug returns before drawing. CPU trace scopes cover capture, relative CCD, simulation, world transaction and replay; counters expose frame/debt, source counts, history bytes, snapshot build time and rejection/slice work.
- Placeable `ABallHandTestRig` uses actual Grip components and the same capture/history/CCD pipeline, enabling no-headset Editor tests. It neither replaces the existing VR Pawn nor claims physical headset tracking.

No Stage 5 RPC/remote pose replication/clock synchronization, Stage 6 network prediction/reconciliation or Stage 7 late-strike adjudication/committed ledger was introduced. Current local collision facts are provisional. The configurable **250 ms** revision window is unchanged. Future scoring/elimination/round/reward consumers must wait for the committed frontier AND admitted pending replay; immediate predicted feedback must not wait for finality. Provisional score/UI must support correction/cancellation.

## Build and executed tests

Engine: **5.6.1**, CL 44394996, installed at `S:/Program Files/EpicGames/UE_5.6`; module `FourSquareVR`. Final Editor build succeeded after source formatting/UHT with zero C++/UHT errors. Log: `Saved/Logs/BallStage4/EditorBuildFinal.log`. Installed MSVC 14.44.35215 and Windows SDK 10.0.26100.0 were used. UE warns this MSVC is not its preferred 14.38 toolchain and that the project retains Unreal5_3 include order; both existing settings were preserved. No Shipping/Android package was compiled. A final Build.cs whitespace-only cleanup was rechecked successfully (UHT zero generated changes, target up to date); log: `Saved/Logs/BallStage4/EditorBuildWhitespaceAudit.log`. Gameplay/test code was unchanged after the 91-test run.

Commands (PowerShell):

```powershell
& 'S:/Program Files/EpicGames/UE_5.6/Engine/Build/BatchFiles/Build.bat' FourSquareVREditor Win64 Development '-Project=S:/Github/4Square/FourSquareVR/FourSquareVR.uproject' -WaitMutex -NoHotReloadFromIDE
& 'S:/Program Files/EpicGames/UE_5.6/Engine/Binaries/Win64/UnrealEditor-Cmd.exe' 'S:/Github/4Square/FourSquareVR/FourSquareVR.uproject' -unattended -nop4 -nosplash -NullRHI -nosound -nohmd -nosteam '-ExecCmds=Automation RunTests FourSquareVR.Ball' '-TestExit=Automation Test Queue Empty' '-ReportExportPath=S:/Github/4Square/FourSquareVR/Saved/Automation/BallStage4/Final02' '-abslog=S:/Github/4Square/FourSquareVR/Saved/Logs/BallStage4/Final02.log' -stdout -FullStdOutLogOutput
```

Final02: **91 succeeded, 0 failed, 0 warnings, 0 not-run**, actual execution, not declaration counting or compilation inference.

| Suite | Passed |
|---|---:|
| Core | 15 |
| History | 9 |
| Collision | 21 |
| World | 9 |
| Hand | 18 |
| HandIntegration | 6 |
| HandTrajectory | 10 |
| HandPerformance | 3 |

Coverage includes all prior ball tests and the new numeric moving/stationary/opposing/glancing/high-speed/tangent palm cases; simultaneous/permuted/conflicting hands; persistent/unknown overlaps; motion reversal knots; world+palm remainder; sleeping-ball wake; raw/motion ring wrap/ownership/pinning; invalid/stale/fast samples; offset arc error; bounded extrapolation/filter/profile changes; independent causal 90 Hz observations across render 30/60/72/90/120/240 Hz plus 72/90/120/240 Hz tracking trajectories; full state/legacy migration; historical collision-event identity/timing/response; wrong match epoch/missing history; atomic overfull manifolds; real component ticks, binding, loss/teleport/ownership, actual engine world rebase and replay after box rotation/destruction. Analytic position/velocity tolerances are generally 1e-6 cm/cm/s or tighter, TOI 1e-8-1e-9 normalized; same-process replay compares complete encoded state exactly. Engine/cook/platform equivalence remains approximate.

Example analytic expectation: radius-3 stationary ball, radius-7 palm moving -20 -> 0 cm in 1/120 s, e=.8 and zero friction: TOI=.5, Vout=4320 cm/s, final X=18.02 cm including 0.02 cm separation skin. Glancing tests verify vector normal/velocity and passive energy dissipation. The additional rotating-palm friction test verifies surface contact at (-7,0,0), Vout=(-1920,-20,0) cm/s, OmegaZ=-50/3 rad/s and 42 kg cm^2/s^2 tangential energy from 84 units of motor work. These are numeric behavior checks, not collision booleans.

Intermediate results are retained: HandCore01 69/69; Native02 80 passed/1 fixture failure (original unregistered descriptor used by tangent test); Complete01 64/25 failures after a new strict query contract exposed an end-probe fraction mismatch. Corrected the normalized [F+1,0] endpoint and local-player ownership guard. Complete02 89/89 passed; Final01 adds the executed pure-kernel benchmark and passes 90/90. Final02 additionally checks the implemented rotating-palm friction/work-energy response and passes 91/91; gameplay source is unchanged from the successful rendered run. An intermediate test-source implicit int-to-bool compile error was corrected. None of those failed intermediate results is presented as completion evidence.

## Rendered Editor and asset integration evidence

Final functional run: `Saved/Logs/BallStage4/Stage4RenderedValidation03.log`, script `Saved/Automation/BallStage4/ValidateStage4Editor.py`, result `Stage4RenderedEditorValidation.json` in the same automation directory. Actual D3D11 RHI, RenderOffscreen, PIE Play (not NullRHI), no HMD/Steam, 20 game seconds/~19.92 wall seconds after startup:

- Existing `/Game/VRTemplate/Maps/VRTemplateMap` loaded and played.
- Existing **VRPawn_C**, locally player-controlled, automatically acquired the native component with exact **MotionControllerLeftGrip / LeftGrip** and **MotionControllerRightGrip / RightGrip** references. Its scripted-tracking flag stayed false.
- Additional diagnostic rig used the explicit test flag. Stationary zero-gravity diagnostic ball was struck, maximum observed speed 402.540929 cm/s; tracked rig status reached None/valid.
- Original saved drop ball settled. No ball Chaos simulation, runaway, pause or crash during the functional run. Maximum diagnostic ball Actor/state error **1.11e-16 cm**.
- No package was saved and no map/Blueprint asset was retained/modified on disk. Temporary in-memory Editor actors were discarded on process exit. Native runtime binding is implemented; no manual Blueprint asset wiring is silently claimed.

Automation limitations investigated: RF_Transient Editor actors were excluded from PIE duplication, so the functional script uses temporary in-memory nontransient actors without saving. A separate high-resolution screenshot task stalled shader compilation, produced a D3D11 buffer-visualization material warning and eventually triggered the existing runtime-overrun pause policy. The screenshot's view did not establish interaction quality. Functional verification was repeated successfully without that task. Failed harness logs remain (`Stage4RenderedValidation.log`, `...02.log`); they are not counted as manual acceptance. **No person operated the Editor/VR headset and no physical controller/headset quality test passed by implication.**

## Measured PC performance and memory

Intel **i5-12600K**, Win64 Development Editor, NullRHI; no controlled core affinity/thermal/load. Final02 uses six engine court components, **eight moving hand proxies plus four recorded kinematics**, 512 forward steps at each initial speed and 256 repetitions of a 30-frame historical replay. Actual hand/world events occurred. These measurements exclude rendering, XR driver capture, networking/clock transport, event-revision ledger/admission and multiplayer concurrency.

| Initial ball speed cm/s | Step median us | Step p95 us | Step p99 us | Replay 30 median us | Replay p95 us | Replay p99 us | Replay maximum us |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1,000 | 70.702 | 79.799 | 114.098 | 2,504.400 | 2,847.400 | 3,181.300 | 3,219.900 |
| 6,000 | 71.898 | 80.600 | 136.700 | 2,653.700 | 3,022.000 | 3,191.300 | 3,665.900 |
| 12,000 | 67.201 | 86.401 | 187.699 | 2,468.400 | 2,884.600 | 3,738.500 | 4,027.000 |

Run variation is material: Final01 measured step medians 20.999/20.903/20.299 us and replay p99 **990.100/999.100/939.400 us**; Complete02 measured replay p99 **3,203.700/3,823.800/3,264.100 us**. Gameplay source did not change between Final01 and Final02 (only one analytic test was added). Core placement/system load/Editor context were not controlled; no specific cause or optimization gain is established. Latest desktop replay p99 is above the aspirational <=2 ms Quest target. Retain all evidence rather than selecting only the faster run.

Maximum supported knot stress: 12 colliders, 96 segments, 85 slices/87 queries, median **1,564.499 us**, p95 **1,853.097 us**, p99 **2,086.397 us**, maximum **2,403.300 us** per step. This is an intentional capacity stress, not typical measured headset input.

Pure relative-sphere CCD: 524,288 mixed sweeps, 65,536 contacts; median **0.127246 us/sweep** (2,048-call batches). This excludes manifold response and engine queries. Eight-hand raw-to-frame reconstruction (256 samples/hand): median **25.500 us**, p95 **26.600 us**, p99 **28.000 us**, max **107.400 us** over 1,024 repetitions, excluding XR polling. CPU trace scopes remain available for attribution; source inspection suggests static-epoch validation, repeated input validation/broadphase and storage layout deserve measurement, not an assumed diagnosis.

| Counted object/storage | Bytes |
|---|---:|
| Complete ball state | 1,888 |
| Ball history frame | 2,144 |
| One motion frame, fixed maximum layout | 11,008 |
| Shared 256-frame motion history | 2,818,048 (2.6875 MiB) |
| One 256-frame ball history | 548,952 |
| Two raw hand histories | 180,224 |
| Eight raw hand histories | 720,896 |
| Four optional kinematic raw histories | 360,448 |
| Native ball Actor object | 6,016 |
| Two ball step-output buffers | 56,448 |
| Native hand component object, excluding its raw allocations | 2,528 |
| Shared six-component static adapter cache | 151,572 |

Rings/cache did not grow during tested writes/replays. Counted one-ball/two-hand storage plus static cache is **3,761,260 bytes (~3.587 MiB)** before UObject components/shared assets/engine query internals/network/event ledger/allocator overhead. Optional kinematic sources and additional balls increase it. The default layout **misses the proposed <=2 MiB runtime target**. The maximum 16-ball registry is a safety cap, not a qualified Quest count. Custom hot paths use bounded preallocation; engine scene-query internals may allocate and the bounded result limit is explicit.

Recommendations for review before full rollback qualification: bounded sparse segment storage instead of full 96-slot snapshots for every ordinary frame; verify once-per-transaction static epoch leases without weakening invalidation; profile broadphase/validation and exact replay bursts; evaluate explicitly configured retention (e.g. 64 frames) only after proving revision+lead+jitter+pending/pin deadlines fit. The default 256 history, 120 Hz simulation, geometry accuracy and configurable 250 ms revision policy were **not reduced** to meet an assumed budget. No Quest/ARM/Android, thermal, headset driver or four-machine network performance has been validated.

## Required Editor/Blueprint setup and manual tests

Existing user map already contains BallTestCourt and HandballBall. For an isolated reproducible scene:

1. Open UE **5.6.1**. Duplicate/create a separate empty level under a new name such as `/Game/Ball/Tests/BallStage4Manual`; preserve VRTemplateMap. Place **BallTestCourt** at (0,0,0), unit scale.
2. Place **HandballBall** at (0,0,100), unit scale, Initial Linear/Angular Velocity zero. Default Physics gravity tests falling/bouncing/rest. For a stationary scripted-hand test set Physics.Gravity=(0,0,0), LinearDrag=0, AngularDrag=0 before Play. No Simulate Physics on ball proxy or visual mesh.
3. Place **BallHandTestRig** at (0,0,0), unit scale. It animates a left palm near Z=100 cm; right palm is parked. It is an additional unpossessed diagnostic Pawn, not the VR default Pawn. Play, enable its debug display and `ball.Debug 1`. Ball should move from a derived palm collision, then interact with court surfaces. Remove the rig for physical VR testing.
4. For actual VR, use the existing VR Template GameMode/Pawn/PlayerStart and VR Preview. Ball `Auto Register VR Hands` defaults true; unique motion-source discovery binds the existing Grips automatically at runtime. To persist custom palm tuning in the Pawn asset, optionally add **VRHandCollisionComponent** to the existing VRPawn Blueprint; the auto path reuses it instead of adding a duplicate.
5. Optional explicit BeginPlay node **Register Motion Controllers**: Target=that VRHandCollisionComponent; Left=`MotionControllerLeftGrip`; Right=`MotionControllerRightGrip`; Head=existing `Camera`; TrackingOrigin=`VROrigin`. Check true. Components must belong to the same Pawn and be attached under VROrigin. Do not register Aim controllers or attach replacement physics bodies.
6. Tune **LeftPalm / RightPalm** Radius, CenterOffset (controller-local cm), restitution/friction and optional rotating-surface friction independently. Default offset is zero and **is not calibrated physical palm placement**. Inspect debug spheres against actual palms for each device; do not assume controller-model axes or grip location equal palm contact center. Standard WorldToMeters=100/unit-scale origin is the validated setup; other scale requires recalibration.
7. Use component `Debug Draw` or `ball.DebugHands 1` for spheres, endpoints/sweeps, velocity/status. Use ball `Debug Draw` or `ball.Debug 1` for ball sphere, contact points/normals, event IDs and overlap/conflict reasons. Set cvars to 0/disable flags for normal operation.
8. Origin changes and engine recenter delegate are detected. If a runtime/Blueprint recenters or relocates tracking without them, call **Notify Tracking Discontinuity** immediately from that operation. Tracking collision resumes after coherent observations; an already overlapping new palm must withdraw to separate. Continuous artificial origin locomotion remains conservative until the later origin-authority design.
9. Optional moving environment: add **BallKinematicColliderComponent** to an Actor with a movable, unit-scale, nonsimulating tracked root/component. Select sphere radius/offset/material or box half extents; keep box orientation fixed. Call **Notify Discontinuity** for scripted teleports. Arbitrary rotating meshes/Chaos feedback are unsupported; do not assume all WorldDynamic objects collide without an opted-in recorded proxy.
10. After a runtime overrun/static geometry change, use **Get Simulation Coordinator -> Rebuild Static Environment** only as an explicit new generation/timebase/history boundary. This recovery preserves current ball position/velocity, resets incompatible history and is not rollback. Startup recovery is automatic only before any gameplay step.

Still required, **not passed**: human Editor placement/configuration/debug readability and play/launch/reset acceptance; physical left/right palm offset/radius calibration, stationary/moving/fast opposing/glancing swings, simultaneous hands, repeat contact feel, tracking loss/reacquisition, recenter/snap-turn/teleport with no false hit, world scale changes, and original teleport/grab/UI/input regression. No physical headset interaction quality or Quest performance is claimed. Multi-process pose transport, prediction/correction and latency/jitter/loss tests belong to unapproved Stages 5-7.

## Files created or modified this stage

All source paths below are under `Source/FourSquareVR/` (absolute root `S:/Github/4Square/FourSquareVR`). **23 files created:**

- Paired `Public/Ball/*.h` and `Private/Ball/*.cpp`: `BallHandTypes`, `BallHandPoseHistory`, `BallMotionHistory`, `BallHandCollisionSolver`, `HistoricalBallCollisionWorld`, `BallMotionContactPolicy`, `VRHandCollisionComponent`, `BallKinematicColliderComponent`, `BallHandTestRig` (18 files).
- `Public/Ball/BallRewindableCollider.h`.
- `Private/Tests/Ball/BallHandCollisionTests.cpp`, `BallHandIntegrationTests.cpp`, `BallHandTrajectoryTests.cpp`, `BallHandPerformanceTests.cpp`.

**20 existing source/build files modified:**

- `FourSquareVR.Build.cs`.
- `Public/Ball/BallCollisionSolver.h`, `BallCollisionWorld.h`, `BallPhysicsConfig.h`, `BallPhysicsTypes.h`, `BallSimulationClock.h`, `BallSimulationSubsystem.h`, `BallStateCodec.h`, `HandballBall.h`.
- `Private/Ball/BallCollisionSolver.cpp`, `BallPhysicsConfig.cpp`, `BallPhysicsTypes.cpp`, `BallReplayRunner.cpp`, `BallSimulationSubsystem.cpp`, `BallSimulator.cpp`, `BallStateCodec.cpp`, `HandballBall.cpp`, `UnrealBallCollisionWorld.cpp`.
- `Private/Tests/Ball/BallCollisionReplayTests.cpp` (actual legacy layout migration fixture), `BallWorldCollisionTests.cpp` (entry regressions).

Planning files updated: `BALL_IMPLEMENTATION_PLAN.md`, `BALL_PROGRESS.md`. Final read-only audit: Git diff --check exited successfully (LF/CRLF conversion advisories only), a separate new/modified source/document trailing-whitespace scan passed, and the specification SHA-256 was reverified unchanged. No specification, Content, Config, project/target or session plugin files modified. Generated Binaries/Intermediate/Saved are ignored build/automation evidence, not retained gameplay assets. `Saved/Automation/BallStage4/ChangedCppFiles.json` records the exact 42 header/cpp formatting paths; Git status is the reviewable manifest.

## Known limitations, risks and remaining stages

- Local tracked hands/recorded proxies only. Current IDs and origin/context versions are local capture identities, **not authenticated server origin authority**. Stage 5 must map ownership, match epoch, clock uncertainty and trusted profile/origin data; remote controllers do not replicate themselves. Stage 6 immediate prediction/feedback/reconciliation and Stage 7 admitted scratch rollback/event revisions/finality are pending.
- Completed motion frames are frozen. Late real samples do not automatically revise past physics in Stage 4. Stage 7 must rebuild dirty input views including interpolation and filter dependencies, pin every dependency and replay all consequences from a complete checkpoint, never apply an old impulse again or rewind only a transform. Already committed gameplay stays immutable.
- Unknown/reacquired overlapping palms require separation before contact; impossible opposing squeezes have documented stable-ID rejection bias. Eight simultaneous normals can be exceeded by extreme multi-player crowds even though 12 dynamic colliders are registered: the tested behavior is atomic failure/pause, not silent truncation. Defaults retain 12 impact batches, up to 8 microintervals, 64 facts/events, bounded slices, 64 world candidates, 256 raw hits, 4096 static components and 16 registered balls.
- Tracking is game-thread observation of provider poses, not a calibrated sensor clock. Between-sample curvature, runtime prediction and silent small recenter jumps remain uncertain. Quaternions use shortest arc; bounds cannot recover unobserved revolutions. Headset/palm calibration and thresholds need hardware. Continuous origin movement currently disables continuity; time dilation/custom WorldToMeters policies need further validation.
- Static geometry is an immutable live epoch, not a copied historical Chaos scene. Component fingerprint/registration/level changes invalidate it; in-place geometry/instance edits preserving fingerprints need explicit invalidation. Prefer closed simple collision. Complex/open/one-sided triangles, heightfield MTD, arbitrary cooks, per-triangle physical materials and engine combine-mode flags are unqualified. Use opted-in recorded dynamic proxies for supported moving geometry.
- CCD is exact for recorded linear segments; acceleration integration and offset interpolation have documented numerical tolerances, not globally exact parabolic/real-hand paths. Skin changes potential energy slightly. Nearly dependent normals (<1e-10 pivot), extreme contact counts or unsupported MTD can fail explicitly. Coulomb friction is iterative, not an exact global maximum-dissipation solution; passive fallback may reduce corner bounce. Material values are tunable game values.
- Lossless checkpoints and same-process replay passed; x64/ARM/cook/query feature rounding may differ near degeneracy. No cross-platform bitwise determinism. Legacy checkpoint loading is value migration, not permission to resume with mismatched old geometry/ID/config epochs.
- Default memory exceeds target and CPU variability/maximum-knot cost needs controlled device profiling. No accuracy/retention/rate/finality requirement was weakened. No Quest qualification or full multiplayer readiness is claimed.

| Stage | Status |
|---|---|
| 1 inspection/architecture | Complete, reviewed |
| 2 foundations | Complete; original 23 tests retained |
| 3 static world/Actor | Complete, 52 original tests retained; entry startup/rest defects corrected, 2 added regressions |
| 4 local hands/historical dynamic inputs | C++ complete; 37 additional tests, 91 total passed; rendered automation passed; physical/human acceptance pending |
| 5 networking / 6 prediction / 7 adjudication | Unapproved, not implemented |
| 8 presentation/forces / 9 qualification | Unapproved future work |

## Exact next-session steps

1. Read `BALL_SYSTEM_SPEC.md`, `BALL_IMPLEMENTATION_PLAN.md`, then this file in full. Inspect user changes and read-only Git status before touching gameplay.
2. Present/review Stage 4 results and outstanding physical tests/performance risks. **Without explicit Stage 5 approval, perform only specifically requested Stage 4 follow-up; do not add networking or proceed automatically.**
3. With a headset/operator, execute the manual checklist and calibrate each palm; record device/runtime/world-scale settings and quantitative false-hit/recenter/latency findings. Keep manual acceptance separate from automated results.
4. Review the default-memory target miss and uncontrolled CPU variation. Agree a bounded-storage/validation profiling plan and obtain target Quest measurements before claiming rollback feasibility; preserve 120 Hz/complete inputs/configurable revision finality.
5. If Stage 5 is explicitly approved, rebaseline the Editor build/91 tests, then implement authenticated owned transport/server clock-origin-profile validation and numerical remote providers as planned. Preserve existing Pawn/sessions and use recorded history contracts. Stop at the next approved stage boundary.

## Session history

- 2026-10-29: read-only inspection, then specification/architecture planning.
- 2026-10-29/30: Stage 2, UE 5.6.1 build and 23 tests, early free-flight replay benchmarks.
- 2026-10-30: Stage 3, UE build and 52 tests, real static collision/replay evidence; Editor asset automation remained unresolved at that checkpoint.
- 2026-10-30: Stage 4 approval; entry defects investigated/fixed before hand code, local tracking/historical CCD integration implemented, final build and 91 tests passed, rendered original-Pawn/rig interaction passed, memory/CPU limits documented. Stopped for review before Stage 5.
