# Multiplayer VR Handball: architecture and implementation plan

Status: architecture approved; Stage 2 implemented, compiled with UE 5.6.1, and verified by 23 passing Automation tests. Stopped for review. Stages 3-9 require subsequent approval.

Prepared: 29 October 2026, Australia/Sydney.

Stage 2 evidence update: 30 October 2026, Australia/Sydney; exact commands/results are in `BALL_PROGRESS.md`.

## 1. Authority, scope, and document precedence

Read at the start of every development session, in order:

1. `BALL_SYSTEM_SPEC.md`.
2. This document.
3. `BALL_PROGRESS.md`.

The specification is the requirements source of truth. This plan describes proposed engineering decisions, not completed features. Changes to scope, limits, or acceptance criteria must be recorded explicitly and checked against the specification.

The entire specification was read: 1,818 lines, 54,116 bytes; SHA-256 `3C37DC3C28F1E0474BED79F1A327AB333404A55201797F670238C8707948CE79` at planning time. Check for changes before implementation.

The user's approval gate overrides the specification's instructions in its opening, sections 24/29, and elsewhere to continue without stopping. Architecture and Stage 2 have now been explicitly approved. Implement Stage 2 only, compile and execute its tests, update evidence, then stop. No Stage 3 Actor/world response, Blueprint integration, session or unrelated configuration changes are authorized in this session.

Stage 2 approval adds early bounded replay/history performance evidence. Introduce `FBallReplayRunner` (pure fixed-input replay, not the future server rollback manager), `FBallStateCodec` (versioned complete-state serialization, not a network snapshot codec), `FBallFinalityPolicy` (eligible boundary calculation, not a gameplay event dispatcher), and `FBallPerformanceCounters` (allocation-free timing/cost aggregation). These refine the planned foundations without changing authority or collision architecture. Stage 2's recorded analytic query adapter may detect a contact, but the simulator must explicitly reject unsupported collision response until Stage 3; it must not silently tunnel through it.

Stage 2 now implements those foundations. Related contacts/forces/query values are consolidated in `BallPhysicsTypes.h` and `BallCollisionWorld.h`; there are no empty Actor/network/Pawn classes. Additional concrete foundation types are `FBallClockSettings`, `FBallClockDiagnostics`, `EBallClockRole`, `EBallClockStatus`, `FBallForceState`, `EBallQueryResult`, `EBallHistoryWrite`, and `FBallReplayReport`. The timing service is currently the concrete `FBallPerformanceCounters` in `BallDiagnostics.h/.cpp`, not an empty `FBallDiagnostics` wrapper. `FBallHistoryFrame` owns complete boundary state and numeric per-step input copies. Future hand/environment/event services are still planned, not implemented.

Fixed state capacities are eight contact episodes and four persistent force-provider records; serialized checkpoints include every live contact/provider/support/sleep/identity/version/seed/ordinal field. State(F) means the start of frame F. Inactive array slots have no simulation meaning and are default-initialized on load. The versioned, CRC-checked checkpoint codec is lossless and bounded to 8,192 bytes; future network compression remains separate. The Stage 2 simulator rejects resting/support/contact-manifold steps with `ContactSolverRequired` until Stage 3, preserving their entire state. Ordered sampled acceleration inputs and angular integration are implemented; arbitrary force-provider execution, sleep/contact response and hand strikes are not.

Actual Win64 measurements: `sizeof(FBallState)=1504`, `sizeof(FBallHistoryFrame)=1760`, 256-frame history allocation including owner = 450,648 bytes, fully populated checkpoint = 1,368 encoded bytes. Final PC fixture replayed 30 frames 512 times: mean 8.306 us, median 8.200 us, p95 8.400 us, maximum 41.600 us on an i5-12600K. These include validation/copy/trace/timing overhead but exclude UWorld queries, collision response, hand history/CCD, event ledgers and networking. They neither prove nor disprove the eventual Quest budgets. Repeat qualification with those real costs before Stage 7; do not extrapolate these results into an unsupported device-performance claim.

### Proposed architecture in one paragraph

Run a plain-C++, restorable sphere simulator at 120 fixed steps per second on the game thread, coordinated by a world subsystem. Use Unreal queries for an explicitly stable static collision environment and mathematical relative-motion CCD for tracked palms and supported historical kinematic primitives. Keep ball state, inputs, collision environment, networking, and visuals separate. Send timestamped hand trajectories through replicated components on the existing owned VR Pawn. Predict locally; validate on the server; retain a bounded mutable history; atomically restore and resimulate when admitted late poses change that history. Regenerate collision events during replay, reconcile event revisions, and commit gameplay consequences only outside the compensation window.

## 2. Verified project integration baseline

| Area | Existing project and implication |
|---|---|
| Engine | `.uproject` association 5.6; installed engine and existing editor receipt 5.6.1, CL 44394996, compatible build ID 43139311. Use this engine's APIs. |
| Module | `FourSquareVR`, runtime; `FOURSQUAREVR_API`. Source presently contains module startup and an empty non-UObject `MyClass`, not a gameplay framework. |
| Build rules | `Core`, `CoreUObject`, `Engine`, `InputCore`; explicit/shared PCH; Game and Editor targets, `BuildSettingsVersion.V5`; no Server target. |
| Pawn | `/Game/VRTemplate/Blueprints/VRPawn`, parent `APawn`; preserve it. `VROrigin` contains Camera and four motion controllers. |
| Hand sources | `MotionControllerLeftGrip` / `LeftGrip` and `MotionControllerRightGrip` / `RightGrip`; Aim sources serve UI. Explicit references take precedence over discovery. |
| Components | Existing `HandLeft` / `HandRight` visuals, XR controller visualization, HMD mesh, widget interactions, teleport visualization, VR notifications. No existing palm collision components. |
| Input and locomotion | Enhanced Input, template teleport/snap turn/grab flows. Preserve their behavior; introduce only explicit integration hooks where authority/discontinuity handling requires them. |
| Grabbing | Template `GrabComponent` attaches objects and changes physics locally; Pawn stores nonreplicated held-component references. It must not drive the custom ball. |
| Replication | Pawn inherits actor/movement replication from `APawn`, but custom variables are not replicated; no custom tracking RPCs found. Client tracking is not automatically sent to the server. |
| Spawn/possession | `VRGameMode` derives from `AGameModeBase`, selects `VRPawn`; no custom spawn/possession policy found. Existing engine PlayerController/GameState defaults apply. Listen-server runtime behavior has not been exercised. |
| Map | `/Game/VRTemplate/Maps/VRTemplateMap`, also selected by map settings; one existing map. |
| Online | SteamNetDriver with IP fallback; App ID 480; AdvancedSessions and AdvancedSteamSessions present and included in existing build. No host/find/join flow was found. Do not replace or expand session architecture as part of ball physics. |
| XR | OpenXR, OpenXREyeTracker, OpenXRHandTracking enabled; Meta/Oculus OpenXR runtime installed. This is not evidence of verified headset operation. |
| Collision | Standard profiles, including PhysicsActor; `3DWidget` trace channel occupies `ECC_GameTraceChannel1`. No ball/hand profile. Avoid consuming or changing that channel. |

The previous Chaos/Predictive Interpolation suggestion is superseded. Do not enable ball rigid-body simulation, default movement replication, Chaos physics replication, or engine substepping as the simulation driver.

### Native and Blueprint boundary

- Add one `UVRHandCollisionComponent` to the existing Pawn, managing both hands and its owned network relay. A second competing pose transport is unnecessary.
- Expose explicit Blueprint-callable registration of left/right motion controllers, Camera, and VROrigin. Auto-discovery matches motion sources `LeftGrip`/`RightGrip`, verifies a unique result and owner, and logs an actionable error on ambiguity. Never silently bind Aim or rely only on component names.
- Register at BeginPlay/possession readiness; refresh ownership on possession; unregister on EndPlay, unpossession, travel, and component destruction. Registration is idempotent and uses weak references outside the solver.
- Keep existing hand meshes local where appropriate. Apply received poses to remote Grip components only; do not let remote XR polling overwrite them. Apply remote head pose to the remote camera/HMD representation without taking over the local camera.
- Add explicit tracking-discontinuity hooks to successful teleport, snap turn, recenter, and tracking-origin changes during the later integration stage. Local detection of transform jumps is a fallback, not proof that a move was authorized.
- Do not add the template `GrabComponent` to the custom ball. Initial handball interaction is striking. A future catch/serve mechanic must be a versioned simulator command with its own tests; it is not implicitly implemented by existing grabbing.
- The ball can be placed directly in the existing map or spawned by existing server gameplay. No replacement GameMode or Pawn is proposed.

## 3. Framework decision and alternatives

| Approach | Benefits | Costs/limitations for this project | Decision |
|---|---|---|---|
| Chaos Network Physics / Resimulation | Engine history and rigid-body integration | Installed `NetworkPhysicsComponent.h` and replication cache are coupled to Chaos solver/rewind/rigid-body state. They do not supply historical custom palm/court semantics. Using a hidden rigid body would violate the specification. | Do not use to drive this ball. |
| Network Prediction plugin with a custom model | Can run non-Chaos Input/Sync/Aux simulation; fixed ticking, rollback services, cues | Available in 5.6.1, disabled by default and marked Beta. A shared ball with input from several owned Pawns still requires custom multi-source admission, historical collider storage, committed-event rules, and server-side late-input resimulation. Its scheduler would have to be the sole scheduler. | Technically viable alternative, not selected initially. |
| Custom narrow history/prediction layer over Unreal replication/RPCs | Direct control over shared-ball authority, pose history, revisions, and replay; easy pure tests | We own correctness, clock estimation, serialization, recovery, and tooling. Considerably more implementation/test work. | Recommended for this specific specification. |
| Delayed simulation without rollback | Simpler final event ordering | Large input delay harms host/remote responsiveness; cannot satisfy the required retrospective resimulation. | Not sufficient. |
| Rewind to confirm contact, then apply an impulse to today's ball | Bounded cheap work | Can invent a physically impossible trajectory, ignore intervening hands/walls, and double-apply energy. | Reject. |

This decision follows inspection of installed engine source, not an assertion that Network Prediction cannot support custom simulation. Reconsider only through an explicit architecture change before dependent implementation; never run two prediction/scheduling systems for the same ball.

Reference material:

- Installed engine: `Engine/Plugins/Runtime/NetworkPrediction/NetworkPrediction.uplugin`, `Source/NetworkPrediction/Public/NetworkPredictionModelDef.h` and rollback services.
- Installed engine: `Engine/Source/Runtime/Engine/Public/Physics/NetworkPhysicsComponent.h`, `Physics/PhysicsReplicationCache.h`.
- [UE 5.6 networked physics overview](https://dev.epicgames.com/documentation/en-us/unreal-engine/networked-physics-overview?application_version=5.6).
- [UE 5.6 RPC ownership and execution](https://dev.epicgames.com/documentation/en-us/unreal-engine/remote-procedure-calls-in-unreal-engine?application_version=5.6).

## 4. Proposed source layout and complete type inventory

Everything stays in `Source/FourSquareVR`. Public declarations live under `Public/Ball/`; matching implementations under `Private/Ball/`. Tests live under `Private/Tests/Ball/`. Do not create empty files/classes to match this inventory. Small related value types share headers.

### Runtime objects and owners

| Type / suggested file | Owner and responsibility |
|---|---|
| `AHandballBall` / `HandballBall.h/.cpp` | Level/server Actor; owns scene root, non-simulating sphere proxy, mesh, network/visual components, live state, per-ball runtime buffers and immutable config references. Registers with the world coordinator. Owns server-only reset/launch/state/config APIs and Blueprint event boundary. |
| `UBallSimulationSubsystem` / `BallSimulationSubsystem.h/.cpp` | Per game/PIE world `UWorldSubsystem`; owns common fixed clock, ball/hand/collider registry, input admission queue, world revision, and ordered game-thread execution. No global cross-world singleton. |
| `FBallSimulationTickFunction` / same file | Subsystem-owned registered `FTickFunction`, `TG_PostPhysics`; prerequisites on hand capture and relevant movers. Drives accumulator, admission, rollback transactions, fixed steps, snapshot publication, and presentation finalization in that order. |
| `UVRHandCollisionComponent` / `VRHandCollisionComponent.h/.cpp` | Existing owned Pawn component, replicated and Blueprint-spawnable. Binds both Grip controllers/head/origin; samples local tracking, relays input and clock messages, stores received remote pose history and presents remote hands/head. Never accepts caller-specified player identity. |
| `UBallNetworkComponent` / `BallNetworkComponent.h/.cpp` | Ball-owned replicated component; publishes baseline/config/snapshots/event revisions, receives replication notifications, drives client reconciliation and bounded baseline recovery. No client-to-server ball RPC assumption. |
| `UBallVisualComponent` / `BallVisualComponent.h/.cpp` | Ball-owned component; owns visual correction offset, display transform and cosmetic cue ledger. Writes only the mesh transform. State/scene root remains simulator-owned. |
| `UBallPhysicsSettings` / `BallPhysicsConfig.h/.cpp` | Optional designer Data Asset wrapper around validated configuration/material table. Server copies it into immutable runtime values; clients receive values/hash/version, not a dependence on matching local assets. |
| `UBallKinematicColliderComponent` / `BallKinematicColliderComponent.h/.cpp` | Opt-in component on supported moving obstacles. Captures shape, transform/velocity, stable identity and lifetime at timestamps for historical queries. No rigid-body feedback implied. |
| `UBallRewindableCollider`, `IBallRewindableCollider` / `BallRewindableCollider.h/.cpp` | Unreal interface pair for custom collider providers. Supplies immutable geometry/motion samples, material and revision/lifetime information. Unsupported shape/motion reports failure explicitly. |
| `ABallTestCourt` / `BallTestCourt.h/.cpp` | Development/test-only-use placeable Actor generating a simple floor/walls and scripted launch/reset/hand paths; no session framework or scoring. Uses existing engine geometry. |

Engine-owned component types used by the ball: `USceneComponent`, `USphereComponent`, `UStaticMeshComponent`. The proxy and mesh always have `SimulatePhysics=false`. Mesh collision is disabled. Proxy is query-only or disabled for external queries; solver sweeps use explicit shape geometry, never proxy overlap delegates as impulses. Set `bReplicates=true`, `SetReplicateMovement(false)` on the ball. Visual scale changes do not change numeric collision radius.

### Plain C++ services

| Type / suggested file | Ownership and function |
|---|---|
| `FBallSimulator` / `BallSimulator.h/.cpp` | Ball runtime; one-step orchestration, state transitions, force/integration/collision calls. Core math has no Actor dependency. |
| `FBallIntegrator` / `BallIntegrator.h/.cpp` | Simulator helper; fixed microinterval force/drag update and angular integration. Centralizes numerical convention. |
| `FBallCollisionSolver` / `BallCollisionSolver.h/.cpp` | Simulator helper; sphere TOI, static/kinematic candidate merging, penetration/contact response, deterministic manifold ordering. |
| `IBallCollisionWorld` / `BallCollisionWorld.h` | Narrow plain interface: gather supported sweep/overlap candidates at specified historical interval/revision. Never implicitly queries today's moving geometry during replay. |
| `FUnrealBallCollisionWorld` / `UnrealBallCollisionWorld.h/.cpp` | Subsystem-owned adapter; stable static scene queries, registered historical kinematic primitives, cached material/ID lookup and reusable scratch memory. |
| `FRecordedBallCollisionWorld` / `RecordedBallCollisionWorld.h/.cpp` | Test/replay adapter for immutable analytic geometry and recorded supported collider trajectories. Exercises the same simulator without a UWorld. |
| `IBallForceProvider` / `BallForces.h/.cpp` | Optional force extension evaluated in stable priority/ID order from state plus recorded input values. No hidden reads of live Actors, clocks or RNG. Gravity/drag are concrete simulator configuration, not a plugin system. |
| `FBallSimulationClock` / `BallSimulationClock.h/.cpp` | Subsystem-owned fixed accumulator, frame indexing, epoch, backlog and explicit pause/overrun state. |
| `FBallNetworkClock` / `BallNetworkClock.h/.cpp` | Client clock estimate plus server per-connection validation state; monotonic offset/drift/uncertainty estimation, frozen input mapping, effective-frame timebase changes. |
| `FHandPoseHistory` / `HandPoseHistory.h/.cpp` | Per participant/hand bounded samples, interpolated segments, deduplication, gap/discontinuity markers. Registry owns authoritative history; Pawn component owns local/remote presentation copies. |
| `FBallHistoryBuffer` / `BallHistoryBuffer.h/.cpp` | Per-ball preallocated state checkpoints, input/environment references and event slices at frame boundaries, with generation/revision validation. |
| `FBallEnvironmentHistory` / `BallEnvironmentHistory.h/.cpp` | Subsystem-owned geometry/version/lifetime and recorded kinematic frames; retained for every replayable interval. |
| `FBallRollbackManager` / `BallRollbackManager.h/.cpp` | Per-ball scratch transaction; earliest-dirty detection, restore, canonical replay, comparison, atomic commit or explicit failure. Never moves live world geometry backward. |
| `FBallEventLedger` / `BallEventLedger.h/.cpp` | Per-ball canonical provisional/committed event history; revision diffs, prediction matching, cue deduplication and finality. |
| `FBallNetworkCodec` / `BallNetworkTypes.h/.cpp` | Explicit bounded wire serialization, ranges, versions and quantization; shares decoded values with prediction. |
| `FBallDiagnostics` / `BallDiagnostics.h/.cpp` | Runtime counters, trace scopes, optional debug drawing and reproducible capture metadata. Dedicated `LogHandballBall` category; debug code compiled/disabled appropriately. |
| `FBallNetworkTestTransport` / tests | Deterministic in-process delivery queue for real message ingestion/reconciliation code under seeded delay/loss/reordering. It supplements actual Unreal networking tests. |

### Value types and enums

All proposed production value types are listed below. Additional implementation-local scratch tuples should remain local; a new externally meaningful type updates this inventory.

| Header | Types and contents |
|---|---|
| `BallPhysicsTypes.h/.cpp` | `FBallIdentity` (stable ball ID, world/session epoch, reset generation); `FBallFrameTime` (uint64 frame plus normalized fractional time); `FBallState` (position, linear velocity, rotation, angular velocity, boundary frame, identity, sleep/support/contact state and deterministic provider state); `FBallContactState` (bounded pair IDs, contact episodes, normal/tangential accumulated state, hysteresis/support); `FBallStepInputs` (immutable hand/collider views, forces/commands, environment/config versions, collision-world reference); `FBallStepOutput` (bounded events, diagnostics, success/failure); `FBallStepDiagnostics` (substeps, queries, iterations, fault reason); `EBallActivityState`, `EBallStepResult`. |
| `BallPhysicsConfig.h/.cpp` | `FBallPhysicsConfig` (numeric simulation settings with units); `FBallSurfaceMaterial` (restitution/friction/spin/rolling coefficients, combine rule); `FBallMaterialTable` (versioned stable material-ID mapping); `FBallNetworkSettings` (rates, windows, caps, precision, recovery); `FBallValidationSettings` (plausibility/time/rate limits); `FBallVisualSettings` (error thresholds/decay). All runtime values validated and versioned. |
| `BallCollisionTypes.h/.cpp` | `FBallCollisionQuery` (sphere trajectory, interval/filter/ignored IDs, revision); `FBallContact` (TOI, outward normal, point, surface velocity/material, stable collider/feature IDs); `FBallColliderId`; `FBallColliderShape` (tagged supported primitive parameters); `FBallColliderFrame` (immutable motion endpoints, generation, lifetime and material); `EBallColliderShape`, `EBallContactCategory`. |
| `VRHandTypes.h/.cpp` | `FHandIdentity` (server-issued participant ID, left/right, motion generation); `FHandPoseSample` (world controller/head pose, world palm position, capture time, mapped time, clock mapping ID, sequence, validity, derived linear/angular velocity, origin version); `FHandMotionSegment` (bounded continuous interval with interpolation/validity provenance); `FHandPlayerRecord` (registry metadata, connection, origin history, clock validation, histories and sequence windows); `EHandSide`, `EHandTrackingStatus`. |
| `BallHistoryTypes.h/.cpp` | `FBallHistoryFrame` (complete boundary state plus input/version/event references); `FBallInputRecord` (immutable accepted raw samples/commands/provider inputs); `FBallTimelineCommand` (server-issued launch/reset/config/authorized origin or environment boundary with effective time and stable order); `FBallForceInput` (provider ID/version plus explicit values); `FBallEnvironmentFrame` (revision/lifetime and kinematic references); `FBallRollbackRequest` (earliest affected time, new input IDs); `FBallReplayResult` (replacement history/event diff, cost, result); `EBallTimelineCommandType`, `EBallReplayResult`. |
| `BallEventTypes.h/.cpp` | `FBallEventId` (ball/generation/frame/fraction/collider/contact episode and ordinal); `FBallPredictionId` (owned participant/hand motion generation/sample pair/local attempt ordinal); `FBallCollisionEvent` (ID, impact category/time/point/normal, incoming/outgoing velocities, impulse magnitude, player/collider, revision/status); `FBallEventRevision` (base/new revision, replaced interval, removals/upserts, commit frontier); `EBallEventStatus` (Predicted, TentativeAuthoritative, CommittedAuthoritative, Rejected, Superseded). |
| `BallNetworkTypes.h/.cpp` | `FBallNetworkSnapshot` (complete replay baseline, frame/epoch/generation/config/environment/revision IDs, sequence, commit frontier, clock anchor and input/decision acknowledgements); `FBallNetworkBootstrap` (protocol/config/materials/identity/latest snapshot); `FHandPoseBatch` (bounded bilateral/head samples, redundancy and sequence bases); `FRemoteHandPoseBatch`; `FBallHitHint` (prediction ID, ball identity, sample IDs and claimed interval, never an impulse); `FBallInputAcknowledgement` (contiguous base plus selective gap mask and dispositions); `FBallHitDecision` (Pending/Accepted/Adjusted/Rejected/Superseded, reason, canonical event and covering snapshot/revision); `FBallClockProbe`, `FBallClockReply` (four timestamp exchange); `FBallBaselineRequest` (owned recovery request); `EBallHitDecision`, `EBallInputRejectReason`. |

Reflect only designer settings, Blueprint event/API values and network USTRUCTs that need it. Keep internal state/history/services free of unnecessary reflection and UObject pointers. NetSerialize must encode fields explicitly; never transmit padding or raw native struct bytes.

### Dependencies and communication

`Pawn tracking component -> owned RPC -> server input registry -> immutable trajectories -> subsystem fixed scheduler -> simulator -> collision world/solver -> state + events -> ball network component -> client history/reconciliation -> visual component`.

Pure simulator depends on value types, integrator, collision solver/world interface and optional force interface. It does not know about Steam, controllers, RPCs, rendering or scoring. The adapter alone resolves engine collision objects/materials to numeric IDs. The event ledger mediates all external effects. Network callbacks enqueue immutable work for the coordinator; they do not mutate mid-step state.

Likely module additions after approval: `HeadMountedDisplay` for motion-controller types; `PhysicsCore` for optional UPhysicalMaterial mapping; `NetCore` only if the chosen serialization helpers require it. `EnhancedInput` only if native action binding is introduced; existing Blueprint actions can remain. No AdvancedSessions, Steamworks, Networking/Sockets, Chaos solver, NetworkPrediction, or engine-wide physics setting changes are required for core ball logic.

## 5. State, units, configuration, and invariants

- Use cm, s, kg, rad/s, kg*cm/s impulse, kg*cm^2 inertia; normal and TOI fraction are unitless. FVector uses the project's UE5 large-world numeric conventions; scalar time/integration calculations use double. Normalize quaternions safely.
- `State(F)` means the complete state at the start of frame F, after every frame before F has completed. Frame F advances exactly dt to State(F+1). Snapshots use this convention everywhere. Fractional contacts belong to frame F.
- Complete state includes persistent support/contact episodes, sleep dwell time, last-impact/rearm state and force-provider deterministic state. Position/velocity alone is not a replay checkpoint.
- Config, material table, collision filtering and force-provider versions are immutable within replay intervals. Changing radius/mass or other unsupported live settings makes an explicit reset boundary. Changing shared fixed frequency creates a new world timeline epoch. Do not reinterpret existing frames using a new dt.
- Resets increment ball generation, clear prediction/contact/history/cue state, and publish a new baseline. New world/travel epoch prevents recycled IDs from matching old packets. Handle integer sequence wrap with tested modular comparisons; never compare uint32 packet sequences using naive greater-than across wrap.
- Default appearance: engine sphere mesh, scaled to a tunable numeric radius; no mesh-scale-derived collision math.

Provisional tuning values, not experimentally measured rubber-ball properties:

| Parameter | Initial proposal |
|---|---|
| Fixed frequency | 120 Hz; world timeline setting, not a per-ball independent clock |
| Ball radius / mass | 3 cm / 0.06 kg |
| Gravity | (0, 0, -980.665) cm/s^2 |
| Linear / angular damping | 0.02 / 0.1 s^-1, exponential and configurable |
| Restitution / sliding friction | 0.8 / 0.35; per-surface overrides, named deterministic combine rule |
| Angular inertia | Solid sphere I = 2/5*m*r^2 initially; replaceable documented model |
| Linear/angular speed caps | Designer caps disabled by default; non-finite/out-of-range states cause diagnostics/fault recovery, not unexplained gameplay clamps |
| Palm radius | 7 cm, plus per-controller calibrated offset |
| CCD / separation skin | Start at 0.01 / 0.02 cm; independently tune scene-query tolerance, TOI epsilon and penetration recovery |
| Initial contact iteration / microinterval caps | 12 contact batches / 8 microintervals per fixed step, subject to qualification |

Sleep/wake thresholds, low-speed restitution cutoff, dwell time, angular friction and rolling resistance must be named settings, with units and tests before final defaults are accepted. Baseline material combination: geometric mean of bounded [0,1] restitution; geometric mean of nonnegative friction; explicit override option. Multipliers/arcade hit boosts are separate, ordered gameplay inputs, default identity.

## 6. Fixed-time execution and numerical method

### Scheduling and threading

One world timeline for all registered balls, hand inputs and supported colliders. Initially independent balls do not collide with one another; adding interacting balls requires group checkpoints/rollback and is an explicit extension.

All capture/admission, queries, simulation and replay run on the game thread initially. No unsafe physics-thread query/state mutation and no implication of a dedicated 120 Hz OS thread. At 72 Hz rendering, frames alternate one/two simulation steps; at 240 Hz, some execute none.

Tick ordering: Pawn movement and controller game-thread tracking ticks -> hand component capture tick in TG_PostPhysics with explicit prerequisites -> subsystem simulation tick with capture prerequisites -> remote/predicted ball presentation finalization before rendering. Capture samples timestamp the actual sampling time, not the last rendered DeltaTime. Verify actual UE tick ordering in Editor; render-thread late update is presentation data, not a second input stream.

Use a double accumulator and integer frame counter. Advance only dt steps, retaining remainder and debt. Initial normal catch-up cap is 8 steps per rendered frame; rollback has its own reserved budget and cap. These caps bound work, not authorize a larger dt or collision skipping.

Server's nominal input cushion is two simulation steps (16.67 ms) behind mapped real time. It reduces small late corrections but does not replace rollback. Client prediction targets estimated present capture time, ahead of this server head, bounded by configured lead from the last valid baseline. A listen-server player's local view also uses the prediction view; avoid a second unvalidated host impulse on the authoritative branch.

### Stall and overload policy

- Mild stall: retain all accumulator debt, execute bounded fixed steps, report backlog and elapsed simulation frame. Never silently drop dt intervals.
- Server debt beyond an initial 250 ms operating limit: enter explicit Overrun/Paused status. Preserve last complete state; stop advancing/committing gameplay and stop accepting ambiguous-time historical hits. Publish status, clock anchor and reason. Once load recovers, resume through a versioned re-anchor at a known frame; discard the paused real-time interval only as an explicit all-client pause/timebase boundary, not an invisible physics advance. Old-timebase hits receive a clear rejection. Persistent overload remains paused or requests match-level recovery; no automatic secret ball teleport.
- Client cannot keep up, loses baseline, or exceeds predicted lead/history: stop speculative stepping, invalidate unconfirmed cues as appropriate, request a full baseline via owned relay, then restart fixed prediction. Never increase dt to chase the server.
- Replay/collision capacity failure is transactional: do not publish partially advanced states/events. Record exact cause. Bounded retry/subdivision occurs only under documented caps; otherwise enter explicit fault recovery. Tests must exercise these branches.

### Integration

Start with a documented semi-implicit kick-and-drift scheme per deterministic microinterval: evaluate ordered forces, update velocity once, apply stable exponential damping, then drift with swept collision for that microinterval. A collision splits drift time; it does not apply gravity/drag again for the remainder. Angular orientation integrates over each elapsed drift segment using the current angular velocity so collision-induced spin affects the remainder consistently.

Microinterval count is determined from state/config and recorded motion curvature, never the rendering rate or machine speed. Acceleration chord-error bound (constant acceleration approximately |a|*h^2/8), palm-offset rotation, and collider-motion limits determine required subdivision. Straight relative sweeps are exact for the represented linear segments, not exact for arbitrary accelerated physical curves. Bound approximation error, conservatively inflate candidate discovery where needed, refine contact, and fail explicitly if configured caps cannot satisfy the accuracy envelope. Conservative discovery inflation must not become a large artificial gameplay hitbox.

Prefer this simple reproducible method initially over a more expensive integrator. Higher-order integration is possible behind `FBallIntegrator`, but requires rebaselining collision-time and replay tests, protocol/config versioning and documented changes in behavior.

## 7. Collision world, CCD, response, and support

### Static world and historical geometry

Unreal world queries provide spatial data; they never calculate the ball's final velocity. The static adapter initially supports stable court/floor/wall collision using simple collision by default. Complex collision is an explicit opt-in to qualify for seams, triangle features and cross-platform query variance.

Installed UE 5.6.1 APIs were checked: UWorld sphere sweeps, `UPrimitiveComponent::SweepComponent`, `ComputePenetration`. A multi-sweep by channel/profile returns only the closest blocking result, so sorting that array alone does not solve tied blocking contacts. Use broadphase overlap of the swept sphere's expanded path bounds, stable-sort bounded candidate components, then per-component sphere sweeps and penetration queries. Merge earliest candidates and near-simultaneous normals into a stable manifold. Cache candidate/material/ID mappings; overflow is explicit. Qualify an optimized closest-hit plus local-manifold path only after it matches the reference adapter.

Static component IDs derive from stable map/component identity plus registration/lifetime generation, not pointer/hash iteration order. Static geometry is only reusable during replay while collision shape/transform/filter/material revision remains unchanged. Detect level streaming, registration/destruction, collision/transform changes and explicit provider changes; make an environment boundary before further stepping/replay. Do not replay against today's altered geometry and label the result historical.

### Dynamic environment scope

- Hands: mathematical moving spheres with recorded pose segments.
- Other supported moving geometry: opt-in historical spheres and translating fixed-orientation boxes. Sphere-versus-box must solve faces/edges/corners of the rounded Minkowski shape, not treat an expanded AABB as an exact sphere contact.
- Record world transform, shape parameters, material, velocities and existence intervals for all supported dynamic colliders. Use relative motion and surface velocity. Despawned colliders retain numeric history until no replay can reference them.
- Arbitrary rotating/deforming meshes, skeletal world collision, unconstrained Chaos props and two-way rigid-body coupling are not initially solved. Existing template physics cubes do not silently become supported rollback geometry. Exclude unsupported objects through the ball query filter, warn when they are expected to interact, and document the playable court contract. Adding such geometry needs a qualified historical collision provider.
- Static geometry changes or unsupported mover changes during play require an explicit environment/generation boundary, with cancelled provisional events and rejected old-generation hints. Prefer changes between rallies. The system must surface this loss of compensation; this is not seamless dynamic-world rollback.
- Replay never rewinds live Actors/Chaos bodies. Numeric collider history and stable static geometry form an isolated historical query view.

### Moving sphere CCD

For a common remaining interval, compute s = B0-H0, d = (B1-B0)-(H1-H0), R = ball radius + palm radius. Solve (d.d)t^2 + 2(s.d)t + (s.s-R^2)=0 for the earliest valid root in [0,1]. Use a numerically stable quadratic implementation and scale-aware tolerances.

Handle initial overlap separately; near-zero displacement; zero/nearly negative discriminant; grazing contacts; separation without approach; opposing motion and crossing between samples. Evaluate both centers and surface velocity at the same solved time. For later collisions in a step, slice the hand trajectory to the remaining interval and recompute relative motion after every change in ball velocity.

Palm center comes from controller pose plus rotated calibration offset. Rotation does not change spherical geometry, but moves an offset sphere center. Derive center velocity consistently; do not add the offset angular term twice when differencing palm-center positions. Contact surface velocity can additionally include omega cross contact-radius for friction if that rotating-surface model is enabled and documented.

### Ordering and response

1. Generate environment/hand candidates for the same drift interval.
2. Find earliest TOI. Quantize only the ordering key to a fine fractional-frame slot; retain full TOI for advance. Gather a contact manifold within the stated TOI/spatial tolerance.
3. Advance simulation position/rotation to contact.
4. Normal points obstacle -> ball. Compute ball contact velocity minus surface contact velocity; only closing normal velocity generates restitution.
5. Resolve a stable ordered manifold, then repeat queries over the remaining time. Never replay a precomputed later hit after an earlier impulse changes its trajectory.

Use normal impulse with ball inverse mass and prescribed kinematic surface motion; Coulomb-limited tangential impulse with sphere inverse inertia; update linear/angular momentum coherently. Solve manifold constraints with accumulated impulses and a cached restitution target derived once from pre-impact closing velocity. Repeated solver passes must not each add a new bounce. Static passive contacts with restitution <=1 and nonnegative friction cannot increase kinetic energy beyond tolerance; moving hands may add physical energy.

Separate overlap correction from velocity impulse. Use bounded MTD/separation with stable fallback normals and previous safe state, an explicit skin, and progress checks. Distinct contact episodes rearm after meaningful separation or a verified new closing impact; no long global hit cooldown. Persistent pair episode state is checkpointed, including after rollback. Contact-cap exhaustion retries using bounded smaller intervals; if unresolved, fail the step transaction and report the fault rather than discard remaining time or tunnel.

Resting requires a valid support manifold, low relative linear/angular motion over a dwell interval, and gravity/force balance within friction constraints. Do not sleep a ball that must slide down an incline. Revalidate support and moving-hand candidates while asleep; wake on a strike, unsupported surface, material/force change, reset or command. Low-speed restitution suppression avoids perpetual micro-bounces; support correction prevents sinking. Rolling/spin resistance dissipates energy through explicit configured terms.

### Collision filtering

Do not alter `3DWidget`. Proposed adapter settings expose object/channel/profile filtering and exclude the ball, cosmetic meshes, Pawn bodies, ordinary template hand meshes and unsupported dynamic props. Hands enter only through registered analytic proxies, preventing duplicate query-plus-hand impacts. A query-only custom-ball profile can be added later if needed; no new project channel is mandatory for the first implementation. Physical material assets are mapped to versioned custom coefficients, not allowed to invoke Chaos response.

## 8. VR tracking, validation, and origin authority

Capture headset and both controllers only for a locally controlled Pawn. A 120 Hz simulation cannot create tracking measurements that a 72/90 Hz runtime never supplied: use timestamped real samples and bounded reconstruction. Preserve raw values, derived velocities, filtering parameters and validity provenance for replay.

Estimate linear velocity from real timestamp differences and angular velocity from shortest-arc quaternion differences. Protect small/zero dt, normalize valid quaternions, reject non-finite data, and apply bounded causal filtering without retiming samples. Reconstruct intervals using linear center interpolation and quaternion slerp with bounded subdivisions for rotating offsets. Rapid reversals must remain represented when captured.

Tracking loss/reacquisition, recenter, possession changes, origin motion discontinuities and large jumps increment hand motion generation. Mark the gap non-colliding; do not sweep across it. Require enough fresh valid samples before re-enabling hits. Hold/fade remote visuals separately from collision validity. Gap >50 ms initially is not a valid long interpolated strike; extrapolation for remote prediction is capped at two fixed steps and then frozen/invalid for new impact claims.

Frequent hand packets are untrusted measurements, not physical proof. Server validates identity through ownership; finite values; quaternion norms; monotonic capture/sequence windows; server-approved origin revision; head/palm reach; speed/acceleration; future/old bounds; hand radius/offset config; batch/RPC sizes and rates. Derive authoritative velocities from admitted positions, not transmitted claimed velocity. Limits must be generous, configurable and measured with real fast swings; diagnostics distinguish rejection reasons and confidence.

Existing client-local Pawn locomotion is a security/integration gap. Pawn actor replication alone does not authorize a client's world origin changes. Proposed minimal hooks submit teleport/snap-turn requests through the owned component, validate destination/navigation and allowed locomotion policy on the server, and record an accepted origin boundary. Preserve the Blueprint locomotion UX; reconcile its origin when necessary. Competitive mode should disable in-rally teleport unless explicitly allowed. If a trustworthy origin path is unavailable, report reduced validation and do not claim competitive anti-cheat. Client-reported HMD/controller data can still be spoofed within plausibility limits; software cannot prove a real physical swing occurred.

Remote visual hands use a short adaptive interpolation buffer (~40 ms initial) with bounded extrapolation. Simulation uses the separately timestamped numeric trajectories for its own target time. Never collide against the lagged remote display mesh. Head and origin context are included for validation; the full body does not need a new movement framework.

## 9. Network protocol, time, bandwidth, and responsibility

### Proposed operating envelope

| Setting | Initial proposal / interpretation |
|---|---|
| Initial match size | Four players/eight palms/one active ball; multiple independent balls supported through bounded registration |
| Authoritative snapshots | 25 Hz, configurable 20-30 Hz independently of 120 Hz simulation |
| Local capture | Actual game-thread XR sample rate, up to 120 samples/s target; no invented sensor samples |
| Hand transmission | 60 batches/s maximum, newest + recent overlapping samples (~50 ms, capped count); adaptive after profiling |
| Remote hand publication | 30 Hz, packed bilateral poses/head context, relevancy aware |
| Server input cushion | 2 ticks / 16.67 ms |
| Maximum retrospective change | 250 ms / 30 ticks behind current server head, with bounded future allowance of 2 ticks |
| Stored history | Initial 256 ball frames (~2.13 s) and 256 real samples per hand; enforce time coverage when capture rates vary |
| Client prediction lead limit | 250 ms beyond latest accepted authoritative baseline; baseline recovery beyond limit |
| Clock probes | Startup burst, then ~2 Hz; estimated offset, drift, RTT and uncertainty |

These are proposed support limits, not guarantees of smooth play at all corresponding pings. Packet redundancy may need adjustment on Quest. The stored history is deliberately longer than the admitted compensation window; storage does not entitle a late input to compensation.

### Transport and ownership

- Ball replicates authoritative bootstrap/config/latest snapshots using a replicated USTRUCT property/RepNotify on its network component. Unreal may coalesce property updates; it is not a promise of delivery of every 25 Hz sample. Each sample has an application sequence and timeline revision.
- Client-to-server hand batches are bounded unreliable RPCs on the replicated Pawn component. A loss can be repaired by overlapping raw samples. Deduplicate by participant/hand motion generation/sequence and preserve allowed out-of-order insertion. Equal sequence with different content is rejected; no history overwrite attack.
- Hit hints are small, idempotent, rate-limited reliable requests on that same owned component because they are sparse. Include referenced pose samples/redundancy when necessary; never accept a hint without reconstructible admitted trajectory. Hints accelerate evaluation, not authorize hits.
- Clock probes, baseline requests and low-rate server replies/decision revisions use the owned component. Never send a Server RPC from an unowned ball. The server routes decisions to the correct participant, not a caller-supplied player ID.
- Revision/event recovery uses bounded reliable deltas plus full mutable-ledger resync on a missing base revision. Rate-limit reliable traffic and cap pending counts; do not enqueue an unbounded reliable stream of pose samples.
- Pose receipt acknowledgements use contiguous base plus a selective mask. Receipt/admission is distinct from hit confirmation and from final event commitment. An absent pose sequence is not implicitly acknowledged by the maximum sequence seen.

### Time mapping

Use a server monotonic clock/epoch and uint64 simulation frames; wall-clock UTC is for logs only. Bootstrap carries current head frame, nominal dt, server timestamp/epoch, input delay, timebase version and backlog/pause status. Unreal GameState server time can seed the estimate, but does not replace capture timestamps or uncertainty measurement.

Use a four-timestamp exchange to estimate round-trip delay and offset, reject outliers, favor low-delay samples and smoothly estimate drift. Network path asymmetry remains unknown. Server bounds the client's proposed mapping against its own per-connection observations; do not trust a client that labels every strike 200 ms old. Freeze mapped sample time and mapping version once admitted. Later clock refinement never moves old accepted inputs to different frames.

New captures use the current mapping; mapping adjustments do not permit time to go backward or skip unprocessed simulation steps. Large discontinuities invalidate outstanding timebase input and require a fresh handshake. No search through the entire lag window for whichever timestamp makes a client hit successful. Start with one frozen mapped time, not a generous hitbox inflation; measure contact sensitivity to clock uncertainty. Excess uncertainty is explicit Pending/ClockUncertain or rejected, not a hidden favorable retiming.

### Wire precision and ordering

Protocol/version, world/ball identity and config/environment hashes precede state. Serialize complete dynamic baseline, including persistent contacts/sleep and necessary replay metadata, not only a transform. Use bounded counts/ranges; reject malformed packets before allocation.

Initial codec target: position error <=0.05 cm within declared court-relative bounds; velocity error <=0.5 cm/s; angular/orientation precision evaluated against <=0.1 degree replay comparison; fractional timestamp error <=0.1 ms. Preserve raw high-precision capture time internally. These are codec acceptance targets to verify, not final bit layouts. Encode out-of-range values explicitly or reject/request baseline; never silently wrap/clamp them. Local prediction should use the same decoded transmitted hand samples/materials where practical, while retaining raw local data for diagnostics.

Accept snapshots by epoch/generation, application sequence, config compatibility and revision. A newer revision at the same frame is valid. Stale snapshots/deltas cannot overwrite a newer correction. Bootstrap must be self-contained for late join; no reliance on old multicasts or prior collision cues. Include a collider identity manifest: canonical map/component keys for static geometry, authoritative IDs/lifetimes for registered dynamics and hands, and semantic collision-content/config revisions. Normalize PIE instance prefixes; do not derive network identity from pointers or platform-specific cooked-byte hashes. Clients resolve their matching local static assets; this does not transmit the court mesh every join. If a referenced config/environment/identity is unavailable or incompatible, wait/request bootstrap and report the mismatch rather than simulate incompatible values.

### Responsibility split

Server: own authoritative states, origin/collider registry, accepted poses, collision decisions, mutable/committed event frontier, resets/config, and replay admission/budget. Host samples follow the same validation and contact rules, although it naturally has no network transit delay.

Client: capture own tracking, send input, predict ball/local hits, reconstruct known remote poses conservatively, store prediction history, reconcile baselines/revisions and smooth visuals. Never set authoritative ball state, impersonate another hand or infer future remote strikes. Dedicated server runtime paths must not require an HMD or local player; the repository currently lacks a dedicated-server build target.

## 10. Complete locally predicted strike sequence

1. Bootstrap establishes world/ball epoch, config/materials, clock mapping and authoritative state. Local Pawn has registered valid hands with an approved origin.
2. Capture controller/head data at actual sample time. Apply calibrated palm offset, derive velocity, update validity and store raw sample with unique hand sequence/motion generation.
3. Queue a bounded pose batch; use the same sample IDs in any local prediction/hit hint. Local render-frame capture is not itself a hit.
4. Coordinator advances fixed predicted intervals toward the mapped present target. Slice the local palm trajectory, combine it with supported world/remote-hand candidates, and solve earliest TOI.
5. On a new valid local impact, the same collision solver updates predicted linear/angular state and contact episode. Store checkpoint/input/event output. Create a `FBallPredictionId` referencing the contributing samples and attempt ordinal.
6. Display the resulting motion this render frame or the next fixed step (at most one dt algorithmic delay); do not add a routine round-trip or one-tick interpolation delay to this local strike.
7. Play bounded speculative haptic/audio once through the cue ledger. Mark the event Predicted. No score/last-hitter rule or irreversible external gameplay effect may be committed by it.
8. Send the hint through the owned component, with trajectory references and identity/timebase. Never send a desired impulse/ball velocity as authority.
9. Continue prediction and retain all replayable inputs and pending decision IDs until covered by authoritative baseline/finality or explicit rejection. If disconnected/history-limited, enter documented recovery rather than predict forever.

## 11. Server adjudication and full historical resimulation

### Revision and finality policy

The server's current head is authoritative for networking, but results within the configured 250 ms mutable tail may be revised by valid late input. Calling an event Accepted does not make it immutable. Publish two states: TentativeAuthoritative and CommittedAuthoritative. Only events at/before a monotonic committed frontier become irreversible gameplay input.

Commit frame F only after its end is older than the compensation window, all already-admitted replay work affecting it is resolved, and no environment/timebase boundary prevents a valid replay. A late packet is evaluated against the frontier at actual admission, not merely claimed send time. There is no promise to accept arbitrarily late data.

This adds up to ~250 ms to final scoring/event finality, while keeping motion/haptics speculative and responsive. It avoids attempting to undo irreversible game rules every time an earlier strike invalidates a later one. Downstream systems that choose to consume tentative events must implement revision-aware undo themselves; the default gameplay delegate exposes committed events separately.

The revision window is configurable, not a fixed 250 ms protocol constant. Convert it conservatively to whole simulation frames and validate that configured replay/history capacities cover it. Future double-bounce, out-of-bounds, last-hitter, scoring, elimination, promotion and serve transitions must consume committed events in canonical order. A tentative UI indication may be shown immediately, but must be retractable; do not eliminate a player, advance a round, or reset the ball based on a provisional bounce. Finality delay can exceed the configured window due to input cushion, pending admitted replay, network delivery or a pause. Window changes require an explicit timeline/config boundary and cannot reopen already committed events.

### Admission and dirty interval

1. Authenticate component ownership; assign participant from connection. Check batch bounds, config/origin/timebase/generation, time windows and pose plausibility.
2. Deduplicate raw samples; identical repeats are no-ops. Insert allowed out-of-order samples in sequence/time order into scratch admission data.
3. Determine all changed interpolation segments, starting with the preceding sample/segment, not only the hint's claimed TOI. An inserted endpoint can change motion before its timestamp. Use a sealed pose boundary at the committed frontier; never alter committed segments. Too-old samples cannot reopen it.
4. Mark the earliest affected mutable frame for each potentially intersected ball, even if the client sent no hit hint. Broadphase must conservatively cover old and new paths. This prevents fairness depending on whether a client predicted a hit correctly.
5. If endpoints are missing, keep a bounded Pending hint until sufficient input arrives or its deadline expires. Do not invent an arbitrarily long sweep from the latest server hand position. Pending requests never block history commitment indefinitely.
6. Reject impossible/future/old/unsupported-environment claims with explicit reason. Admitted physical pose data may remain useful even when a particular collision hint misses; hint rejection is not automatic rejection of all tracking. Samples rejected for retrospective admission cannot mutate sealed historical segments or be retimestamped to create a present-day strike. A usable newest pose may seed future tracking only across an explicit non-colliding discontinuity boundary, preserving the rejected historical interval in the audit log.

### Replay transaction

1. Coalesce new dirty inputs for a ball into one earliest-dirty request per coordinator frame. Reserve bounded replay/query capacity before treating them as applied to authoritative collision history.
2. Restore the complete boundary State(F) before the changed interval, including support, sleep, pair episodes, provider state and config/material/environment versions. F must be after the immutable committed prefix, or at its sealed boundary with no change to committed outputs.
3. Use an immutable view of accepted sample trajectories plus proposed admitted changes; recorded timeline commands/forces; supported historical collider trajectories/lifetimes; unchanged versioned static geometry.
4. Resimulate F through the current head using the normal simulator. Re-detect all collisions and contact times. The old bounce/hit list is an audit/reference output, NEVER a list of impulses to reapply blindly. New earlier trajectories may avoid or create wall/hand contacts later.
5. For a late hint, compare the regenerated canonical events against the submitted sample/interval. The server independently derives TOI, surface velocity and impulse. Classify Accepted (equivalent), Adjusted (legitimate but different), or Rejected (no contact/invalid). Missing data remains Pending only within a bounded deadline.
6. Revalidate every later provisional hand strike. Revoke previously accepted-but-provisional events if the new earlier path makes them impossible. Preserve the committed prefix. Server commands such as an explicit reset are immutable timeline boundaries, never regenerated collision impulses.
7. Compare scratch results/invariants, create an event diff and replacement checkpoints. Only then atomically swap the mutable history and current authoritative state, increment timeline revision, and publish a covering authoritative snapshot plus ordered event/decision revision.
8. No external gameplay/audio/haptic callbacks run inside scratch replay. Failure leaves the last complete authoritative branch intact and emits a clear Pending/rejection/fault result, not a half-replayed branch.

New pose data is not finally acknowledged as applied until the transaction commits. Work budgets must prevent an attacker causing unbounded replay. Start with a maximum 32 replayed fixed frames per transaction (covers the 30-tick window), bounded candidates/microintervals, a coalesced per-ball replay, per-connection token buckets and measured global CPU budget. An exhausted budget may defer at most a small configured number of coordinator frames without crossing commitment; otherwise reject retrospective admission as CapacityExceeded and surface server degradation. Never apply the strike at today's position as a fallback. Zero such rejections at nominal four-player load is a release criterion.

### Simultaneous and conflicting strikes

Order by validated physical TOI; use a quantized fractional-time key for deterministic ties, then category/collider/participant/hand IDs assigned stably for the match. Avoid non-transitive epsilon comparators. Exactly coincident contact normals form a manifold solved in stable order, with later constraints rechecked after earlier impulses. A match-stable player-slot assignment prevents always privileging the host's memory/registration order; some exact-tie ordering bias still exists and must be documented.

Example: A's late pose creates an earlier contact; B's already-published provisional strike was later on the old path. Restore before A, recompute all drift/contacts, and accept A only if the new canonical path really contacts it. If the ball now misses B, supersede B and send its replacement/removal. Do not keep B's old impulse because it was once Accepted. If B is already committed, A is outside the allowed mutable causal window and cannot change it.

### Stable identities and replay consistency

Input identity is immutable: participant session + hand + motion generation + sample sequence. A server event ID contains ball/generation/frame/fraction/collider/contact-episode/ordinal, with revision separate. Contact episodes and ordinal state are restored with checkpoints. An altered trajectory can legitimately produce different event IDs; the revision explicitly removes old provisional IDs before adding replacements.

Prediction IDs are correlation tokens, not authoritative event IDs. Server maps a token to zero/one canonical episode or a replacement set; duplicate hints return the latest disposition/revision without an additional impulse. Distinct client tokens for the same physical episode map to the same authoritative result. Generations, pose admission IDs, contact episode state, event ledger and replay output suppression together prevent duplicate hits. None alone is sufficient.

Decision records carry revision and finality, not just an Accepted/Rejected enum. Invalid ownership/malformed data or an old-generation claim can be terminally rejected. A plausible NoContact result inside the mutable tail is a tentative rejection: a newly admitted endpoint can legitimately change it to Accepted/Adjusted on a later revision. Retain correlation records until commitment/deadline, re-evaluate them against each replacement interval and notify clients of the new disposition. Do not let hint deduplication freeze an obsolete rejection, or let repeated hints reopen a committed decision. Clients recover from a tentative rejection immediately but may later receive a valid revised outcome.

An event delta identifies base/new revision and replaced interval. Missing delta -> request full current mutable ledger + baseline. Do not patch an event list onto the wrong branch. Recently committed IDs/decision watermarks remain bounded and sufficient for exactly-once consumption across retransmission; overflow triggers explicit resync, not silent eviction of live decisions.

## 12. Client acceptance, adjustment, rejection, and reconciliation

1. Receive an authoritative update. Validate identity, protocol/config/environment, sequence/revision, complete state and acknowledged input/decision ranges before modifying local history.
2. Match its boundary State(F) to predicted history of the same generation and compatible branch. Compare position, linear/angular velocity, rotation, sleep, contact episodes/support, config and decision state. Small transform error alone is not grounds to ignore a rejected hit or changed contact state.
3. Accepted equivalent strike: mark token TentativeAuthoritative, correlate canonical event, suppress duplicate cue; a covering snapshot still anchors subsequent prediction. Later commitment makes it final.
4. Adjusted strike: remove the predicted event mapping, insert the canonical replacement, restore the covering authoritative state and replay only subsequent valid inputs. Do not add the new authoritative impulse on top of a predicted impulse already represented in the baseline.
5. Rejected strike: revoke predicted-only state/cues and remove invalid input if the server rejected its underlying sample. Apply a covering snapshot whose boundary is after the adjudicated interval; replay later input only. This prevents replaying the rejected historical hit as a fresh input. If a decision arrives without a covering baseline/revision, hold it as pending correction and request one rather than faking a velocity subtraction.
6. A legitimate pose can remain even when its hint misses. Reconstructed collision against corrected state decides future contacts. Contact episode state seeded from authority prevents the same overlapping hand being treated as a new impact after correction; no hand-wide cooldown hides mistakes.
7. Restore authoritative full state at F, replay recorded hand/provider/control inputs with supported environment to the current fixed prediction target, and replace local history. Ball roots/proxies move directly to the corrected simulation state.
8. If F is missing, config/world history is incompatible, or replay exceeds the bounded lead, use full-baseline recovery and clear affected speculation. Do not reuse today's dynamic collider transform as historical input.
9. Apply event removals/upserts before dispatching new cues. Confirmed/committed IDs play at most once; replay itself is silent. A haptic already felt cannot be undone: rejection prevents repeats and may produce a small correction cue, not a claim of reversing sensation.
10. Commit visual correction as a mesh-only offset from the previous rendered pose to the new predicted pose. Subsequent frames decay it subject to impact/visibility constraints.

Clients cannot accurately predict unknown remote future hands. Use bounded extrapolation, stop it when invalid, and accept that remote impacts can create corrections. Local and server query/math differences are reconciled; they are not proof of broken transport by themselves.

## 13. Presentation, events, commands, and lifecycle

Physics state is the only collision authority. At ordinary rendering frames display current predicted state with bounded render extrapolation from the fixed-step remainder if qualified, rather than mandatory one-full-tick trailing interpolation. Never use rendered extrapolation as hit-test input. Use the same simulated quaternion/angular state; no independent cosmetic spin affects physics.

On correction, retain a bounded mesh-only position/rotation error offset and decay it, initially over 30-60 ms with small spatial caps. Clear/reduce the offset immediately on local impact; clamp wall penetration and large mismatch; snap on reset/generation change, large errors or unsafe wall crossing. A visually shifted ball is harder to strike: cap normal smoothing displacement relative to the 3 cm ball radius (initial target <=1 cm) and expose the difference in debug overlays. Large errors prioritize truthful collision position over a smooth but misleading mesh. This trade-off needs physical VR tests.

Provide separate delegates for PredictedContact, AuthoritativeContactRevision, CommittedHandHit, CommittedCourtBounce, CommittedWallHit, Rest/Wake, Correction and Reset. Event payload includes all specification section 14 fields. Scoring rules and court-square logic remain outside the solver. Last hitter and bounce-count consumers use committed events or explicitly reversible provisional state.

Server-only Blueprint APIs: ResetBall, LaunchBall, SetBallState, SetBallPhysicsConfig. Client calls cannot mutate authority; request permissions through an owned relay if a practice tool needs it. Read APIs return current state/linear/angular velocity and authority/prediction status. Commands have immutable IDs/effective frames and participate in replay; reset/config-geometry barriers invalidate old generations explicitly.

Late join receives compatible config/materials, current full state, clock anchor, generation/revision and current decision frontier; no historical cue playback. Clear registrations and buffers on travel/destruction; tombstone disconnected hands/colliders for retained replay intervals. A disconnected player's earlier admitted trajectory may still be historically relevant, but no new inputs can extend it after its lifetime boundary.

## 14. Performance, memory, and extensibility

Preallocate ring buffers, pair-contact capacity, candidate/hit arrays, event/decision storage and replay scratch. Reuse query params. No UObject allocation or regular heap growth inside fixed simulation/replay. Allocation during registration/bootstrap/config/capture-export is allowed, bounded and measured. Unreal query APIs can allocate internally; instrument warmed-up adapters and remove avoidable caller-side growth instead of claiming global zero allocation without evidence.

Conservative initial memory estimate per world at four players/one ball (measure sizeof, padding and actual retained payload later):

| Storage | Budget assumption |
|---|---|
| Ball checkpoints | 256 * ~1 KiB including bounded contacts/metadata = ~256 KiB |
| Eight hand histories | 8 * 256 * ~192 B = ~384 KiB |
| Per-frame references/events/control/environment | ~512 KiB initial bounded allowance |
| Scratch replay + ledgers + presentation | ~512 KiB initial allowance |
| Total target | Approximately 1.6 MiB, initially cap/measure <=2 MiB excluding engine collision assets and diagnostic exports |

Additional balls share hand/environment history but own checkpoints/replay/ledgers. Do not duplicate full per-player history into each snapshot/frame. References must pin the numeric data/config versions until their consuming frames expire; ring wrap cannot mutate a still-referenced replay input. Capacity bounds include adversarial event density, not only one bounce/frame.

At four players, ordinary analytic palm tests are about 960 pairs/s per ball before microintervals. A full 250 ms replay covers up to 30 fixed steps; static query counts and worst-case corner iterations dominate. Coalesce late arrivals and profile worst case, including 150 ms RTT and a jitter burst. Initial measured targets: average ball-system CPU <=1 ms/rendered frame on target Quest hardware, replay-burst p99 <=2 ms, <=2 MiB runtime storage, and no unbounded queues. These are qualification targets, not results or guaranteed device budgets; failure requires measured optimization or explicit design review, never silently lower physics accuracy.

Rough payload estimates: 100-180 B bilateral/head sample set, 3-4 redundant sets * 60 batches/s gives ~18-43 KiB/s uplink per player before transport overhead; ~150-350 B complete snapshot *25 = ~4-9 KiB/s per ball/client; remote poses can add ~10-25 KiB/s/client. These are deliberately broad estimates and may be too expensive for the final target. Profile measured wire bytes; compress redundancy/pose deltas and use relevancy before changing physics rate. Snapshot contact data and reliable event bursts must be counted. No claim of final network efficiency until capture measurements exist.

Trace simulation/replay/query cost, backlog, frame count, TOI/iteration caps, history occupancy, packet bytes, clock uncertainty, rejected inputs by reason, revision rate and correction magnitudes. Optional overlays cover every specification section 17 visualization. Capture reproducible numeric inputs/config/geometry revisions and seeds; never rely on unrecorded live world state in an offline replay.

Force extension: stable priority/provider ID order, immutable recorded `FBallForceInput`, explicit deterministic provider state. Wind, Magnus, abilities and alternative gravity are later concrete providers; only interfaces/basic angular/contact behavior are initially required. No future force can bypass the common authoritative/predicted pipeline.

## 15. Risks, ambiguities, and decisions for review

| Issue in specification/project | Proposed resolution / remaining decision |
|---|---|
| Immediate implementation instructions conflict with this request's approval gate | Gate wins. Only two planning documents change now; requirements remain intact. |
| Specification background assumes networked VR players; project has no tracked-pose transport | Implement owned Pawn component transport and remote presentation explicitly. No assumption that Steam/Advanced Sessions supplies it. |
| Rollback and already-authoritative events have no finality policy | Mutable 250 ms authoritative tail + explicit revisions, monotonic commit frontier; irreversible gameplay waits. Approval of this latency/fairness trade-off is a major architecture decision. |
| "Replaying collisions" could mean replaying stale impulses | Replay raw trajectories/commands/environment; recompute collision outputs; old event list is audit only. |
| Unreal scene queries do not rewind world geometry | Stable static revision plus recorded supported kinematic primitives. Unsupported arbitrary movers excluded; topology changes are explicit compensation barriers. |
| Dynamic geometry requirement is broad | First support spheres and translating boxes; arbitrary rotating/deforming/Chaos bodies remain documented unsupported cases, as specification 5.5 permits. No false claim of general dynamic-mesh CCD. |
| Fairness cannot be guaranteed with untrusted clocks and unobserved physical hands | Bound capture-time mapping, uncertainty, origin/reach/kinematics, horizon and rate. Document residual spoofing/path-asymmetry/host advantage. No favorably searching history for a hit. |
| Existing locomotion is local and Pawn transform replication is downstream | Add minimal owned origin-authorization/discontinuity hooks in later Pawn integration; competitive in-rally locomotion policy still needs product approval. |
| Immediate feedback vs finite fixed dt / render rate | Local impulse appears in the next eligible 120 Hz step and current render, without network wait. Cannot promise sub-frame display or real 120 Hz samples from slower tracking. |
| Render smoothing can put the displayed ball away from collision state | Small capped mesh-only corrections, impact-aware snap, debug difference and headset acceptance tests. No collision against a lagged cosmetic mesh. |
| Unlimited collision accuracy vs bounded CPU | Configured spatial/TOI error envelope, bounded subdivisions/candidates and explicit transactional failure/recovery, not discarded remainder. Qualify maximum supported speeds/geometry. |
| Portable determinism | Same recorded scenario should replay tightly on one platform; x64/ARM and Unreal scene queries only approximate consistency. Stable ordering helps but cannot promise bitwise equivalence. |
| Spin scope uses "where practical" | Include angular state/integration/damping immediately; implement tested friction-induced spin and basic rolling resistance by final stage. Magnus/wind/complex hands remain extensions, not falsely completed features. |
| Ball-ball interaction unspecified | Independent ball instances initially. Mutual ball impacts require group rollback and a reviewed extension. |
| Quest transport and SDK support | Core stays platform-neutral; Steam sessions do not establish a standalone Quest multiplayer strategy. Android SDK/NDK/package and alternative transport validation are future deployment work. Preserve current Steam setup. |
| Dedicated server support vs current project | Runtime paths remain headless-safe; no Server.Target.cs currently. Separate approved target/source-build setup may be necessary; no dedicated build claimed. |
| Precise palm calibration, material values, validation limits and wire layout absent | Start with labelled tuning proposals, measure real swings/court bounce, version config, approve final codec after precision/bandwidth tests. |
| Unsupported live geometry/config mutations | Explicit boundary/reset; keep history versions or cancel unresolved provisional outputs. Do not silently finalize them early just to free history. |

Unresolved choices do not weaken requirements. Major changes to the recommended finality horizon, dynamic support scope, origin authority or networking framework require an explicit plan revision and review before affected implementation.

## 16. Implementation stages, acceptance gates, and tests

Stage numbering follows specification section 24. Tests are created/executed with their dependent stage, not postponed to Stage 9. Each stage updates `BALL_PROGRESS.md` with exact files, build command/result, executed tests/results, unverified manual cases and next steps. Approval of this plan does not authorize skipping subsequent stage review boundaries.

### Stage 1 - inspection and architecture planning (completed planning phase)

Deliverables: complete specification reading, project/API inspection, this plan and progress log. Acceptance: source-of-truth/approval conflict recorded; every major subsystem/type/ownership boundary and causal replay policy defined; no gameplay/config/asset changes. Verification: document checks and filesystem change audit only. No gameplay tests or compile implied.

### Stage 2 - core structures, clock, replayable inputs and mathematical foundations

Implement value types/settings/validation, fixed clock, preallocated histories, complete state save/restore, pure simulator/integrator and analytic collision-world test fixtures. Include frame/generation/sequence semantics from the start; do not create empty future runtime components.

Additional approved acceptance: instrument step/replay duration, work counts and allocated history bytes now; test replay-frame caps, pinned-history exhaustion and atomic failure with unchanged destination/state. Report measured PC evidence and fixture limitations. A free-flight/analytic replay benchmark excludes future scene-query/contact/network costs and establishes no Quest feasibility claim. Repeat with real collisions at Stage 3 and tracked multi-player inputs at Stages 4-6 before Stage 7.

Acceptance: exact fixed-step counts at recorded 30/45/60/72/90/120/144/240 Hz rendering over equal elapsed time; zero/multi-step frames and remainder tested; explicit overrun/pause recovery; gravity/drag/rotation tests; no NaN; state and all persistent contact/provider data restore; ring wrap/generation/sequence-wrap safety. Same-platform replay target <=0.001 cm position / <=0.01 cm/s velocity in deterministic analytic fixtures, with justified scale-dependent tolerances. Compile Editor target and run relevant `FourSquareVR.Ball.Core`/`History` Automation suites. No claim of world collision/multiplayer yet.

### Stage 3 - static CCD, response/rest, ball Actor and engine adapter

Implement actual placeable `AHandballBall`, coordinator/tick registration, sphere/mesh with no Chaos simulation, query adapter/reference broadphase, materials, gravity/drag/swept remaining-time response, support/rest and initial penetration. Add analytic moving-sphere/box kernels needed by later stages and native test-court basics. Frame state/event protocol remains compatible with later rollback.

Acceptance: high-speed floor/thin-wall tests up to at least 20,000 cm/s with supported settings; no tunnelling under qualified interval/geometry envelope; tangent/separating/initial-overlap/corner/zero-time/cap-failure tests; multiple bounces preserve remaining time; passive energy inequality; stable resting for 30 s without sinking beyond configured skin/tolerance; slope that must slide does not sleep; wake on scripted strike; no engine collision impulse path. Compile/run pure and UWorld adapter Automation tests and manually place/launch/reset the ball in Editor. Headset not required. Scene-query tie behavior compared to analytic reference.

### Stage 4 - VR hands, recorded dynamic colliders and Pawn integration

Implement tracking component/discovery, timestamped histories, real-sample interpolation, hand CCD/response/episode state, supported historical kinematic component and exact Blueprint integration instructions. Add approved components/hooks to the existing Pawn only during this implementation stage; preserve original locomotion/grabbing for other objects.

Acceptance: explicit references and source discovery bind correct Grip components; duplicate/ambiguous registration fails safely; local-only capture; crossing/opposing/stationary/grazing palm tests; rotating offset without double angular velocity; no repeat energy from sustained contact; loss/reacquisition/teleport discontinuity cannot strike across gap; translating sphere/box history works, unsupported rotation identified. Automated scripted hand sequences plus Editor mock hands. Physical fast swings, calibration, tracking-loss/recenter and regression of teleport/UI/grabbing remain mandatory headset tests.

### Stage 5 - networking foundation, clocks and authoritative baseline

Implement owned RPC hand/clock transport, validation/origin authority hooks, immutable config/bootstrap, snapshots, remote pose interpolation, reset/generation and late join. At this stage remote historical contacts without the later compensation pipeline are explicitly not complete multiplayer lag compensation.

Acceptance: two processes/players send only their own hands; dedicated/headless paths require no XR; complete serialized state round trips within precision targets; malformed/oversized/nonfinite/spoofed samples rejected; dedup/out-of-order/gap mask and sequence-wrap tests; clock drift/future/old/discontinuity tests; stale snapshot/newer-same-frame-revision handling; current moving-state late join and reset-old-packet rejection. Compile/run codec/clock/transport tests and actual multi-process Unreal networking via supported local/IP test path. Steam two-machine discovery/connection remains a separate manual test, not a reason to redesign sessions.

### Stage 6 - immediate prediction, client reconciliation and cue identity

Implement predicted client branch, baseline comparisons/restoration, full replay, local event correlation, bounded lead/history recovery and preliminary mesh-only correction. Host visual prediction remains separate from server state but follows the same input/solver rules.

Acceptance: immediate scripted local hit without round-trip wait; corrected snapshot restores the right historical boundary and replays to the same target frame; rejected/adjusted predicted velocity does not remain or double-apply; persistent-contact mismatch reconciles even with tiny position error; duplicate cues suppressed; missing history/config/revision triggers baseline recovery. Seeded delay/loss/reorder harness must use actual production message handlers. Compile/run prediction tests; two-process 80 ms RTT test with logged input/prediction/authoritative events. Server retrospective acceptance is not claimed until Stage 7.

### Stage 7 - bounded server lag compensation, finality and conflicting events

Implement input-driven retrospective evaluation, immutable historical environment/commands, coalesced scratch rollback transactions, provisional event revision protocol, commit frontier, pending deadlines, budget admission and all outcome mappings.

Acceptance: delayed-pose delivery (with and without a hint) matches an on-time reference trace when all inputs arrive before finality and under the same supported environment; new interpolation endpoints dirty the correct preceding interval; late A invalidates provisional B without reapplying old impulses; committed prefix is byte/logically unchanged; equal-time contacts stable under batch permutations; duplicate hints/samples cause one episode; a tentative NoContact rejection can be revised after a valid new endpoint, while a committed disposition cannot; rejected retrospective data never produces a time-clamped current strike; old-generation/environment/future/overload cases reject with reasons; missing delta recovers full baseline/ledger; all clients converge after rejected predictions. Run 80/150 ms RTT, jitter/loss/out-of-order tests and compare complete state/contact/event traces, not only final transform. No stage completion based solely on visual similarity.

### Stage 8 - presentation, spin/forces, diagnostics and usable test workflow

Finish impact-aware correction, structured Blueprint events/APIs, contact spin/angular damping/rolling resistance, ordered force extension fixtures, debug overlays/traces, authority-safe native test tools and concise Editor integration guide. Basic angular state and event IDs must already exist; do not retrofit replay-critical state late.

Acceptance: visual correction never moves simulation proxy; reset bypass; local strikes remove excessive smoothing; display/collision offset caps tested; no repeat sound/haptic on replay/retransmission; committed event consumers exactly once; contact-induced spin conserves/loses energy appropriately for passive surfaces; force replay order stable; all required overlays toggle; test court and no-headset scripted controls work. Compile/run presentation/event/force tests; manual VR assessment of perceived contact and corrections is required.

### Stage 9 - integrated qualification and PCVR/Quest readiness evidence

Run the complete test matrix, soak/fuzz/capacity tests and profiling; fix failures in their owning modules; document exact supported operating envelope and remaining platform work. No blanket "complete" if hardware/multi-machine requirements remain unexecuted.

Acceptance: specification tests 1-32 and scenarios A-I have explicit results or clearly unverified entries; seeded network trace convergence after traffic settles; 30/72/90/120 Hz render and artificial stalls; 0/80/150 ms RTT plus 0/20/50 ms jitter, 0/1/5% loss, reordering/duplication and 200 ms burst loss; invalid hostile data; repeated resets/travel/disconnect/late join; 30-minute minimum soak; bounded memory/queues; nominal four-player replay admission without capacity rejections. Test asymmetric links and clock uncertainty separately from symmetric RTT. Quest/x64 consistency and CPU/bandwidth targets require actual device evidence; absence is a documented unverified deliverable, never a fabricated pass.

### Specification section 18 test coverage map

| Required test IDs | Planned primary stage / suite |
|---|---|
| 1 gravity, 2 rendering-rate consistency, 7 zero velocity, 8 finite values | 2 Core; repeat with UWorld/network in 3/9 |
| 3 floor bounce, 4 energy, 5 settle, 6 wake | 3 Physics; moving-hand wake repeated in 4 |
| 9 floor tunnelling, 10 thin-wall tunnelling, 15 tangent, 16 penetration, 17 multiple contacts, 18 zero-time loop | 3 CCD/WorldAdapter |
| 11 moving hand, 12 moving ball, 13 crossing, 14 separation | 4 HandCCD |
| 19 state restore, 20 replay equivalence, 21 wrap, 22 generation, 23 invalid frame | 2 History; full environment replay repeated in 7 |
| 24 frame comparison, 25 stale snapshot, 28 duplicate inputs, 29 timestamps | 5 Codec/Clock/Transport |
| 26 historical correction, 27 replay target, 32 rejected prediction | 6 Prediction; authoritative rejection repeated in 7 |
| 30 delayed rollback, 31 conflict ordering | 7 Rollback/EventLedger |

### Evidence and tolerances

Use Unreal Automation Tests with pure analytic fixtures and UWorld integration fixtures; functional/real network tests supplement them. Proposed suites: `FourSquareVR.Ball.Core`, `.Physics`, `.CCD`, `.WorldAdapter`, `.HandCCD`, `.History`, `.Codec`, `.Clock`, `.Transport`, `.Prediction`, `.Rollback`, `.Events`, `.Presentation`, `.Forces`, `.Soak`. Test implementation classes are local Automation macro products, not new production architecture.

Record config, dt, platform/compiler, random seed, inputs/delivery schedule, geometry revision and failing frame. Compare positions, velocities, angular state, sleep/support, contact episodes, generation/revision, event identities and finality. Query tests need scale-aware documented tolerances; initial cross-platform analytic-fixture target is <=0.1 cm position, <=1 cm/s velocity, <=0.1 degree rotation over a specified 10 s trace. Chaotic collision ordering may diverge near degenerate boundaries; identify sensitivity and reconcile, never claim universal bitwise determinism.

Required additional tests: immutable committed history; event retraction/replacement transaction; no output on failed replay; config/origin/environment barriers; lost decision delta; unknown collider/material ID; wire-range overflow; missing sample endpoint; invalid gap interpolation; actor destruction during retained history; map/PIE world isolation; sustained malicious replay demand; passive friction/manifold energy; allocation and queue bounds; no reliable-per-tracking-frame traffic.

## 17. Build/test environment and current verification status

Available: UE 5.6.1 at `S:\Program Files\EpicGames\UE_5.6`; Build.bat, UnrealBuildTool, built-in C# UHT, UnrealEditor/UnrealEditor-Cmd and RunUAT; bundled .NET 8.0.300; VS Community 2022 17.14.13; MSVC 14.44.35207; Windows SDK 10.0.26100.0. Existing game/plugin editor module build IDs match 43139311. MSVC is newer than UE's preferred 14.38 and may warn. Plugin descriptor labels 5.5/5.4 are not by themselves proof of a compile failure.

After approval, initial build target: `FourSquareVREditor Win64 Development` with the absolute `FourSquareVR.uproject` path through the installed Build.bat. Run `FourSquareVR.Ball` Automation tests through UnrealEditor-Cmd with unattended logging/report export appropriate to the created tests; verify process exit and actual test report, not merely launch success. Preserve logs/reports in the project's generated Saved test output and summarize exact commands/results in progress. Close/coordinate Live Coding before reflected type changes as necessary.

The planning phase performed no build/gameplay tests. Stage 2 subsequently built successfully with the installed UE 5.6.1 and executed 23 Core/History Automation tests: 23 passed, zero warnings/failures/not-run. Current results and preserved build failures are recorded in progress. Android SDK/NDK, a Quest package, dedicated-server target/build and physical/multiple-machine VR tests remain unverified. Git investigation before Stage 2 gameplay edits established that the repository at `S:/Github/4Square` is usable with approved read-only commands outside the workspace sandbox; sandboxed discovery cannot chdir into that parent. Final read-only status/diff checks found only the expected untracked documents/new source, no tracked-file modifications. No Git mutations were performed.

## 18. Requirement traceability and review boundary

| Specification sections | Plan coverage |
|---|---|
| 1-3 project/principles/types | 1-5, inventory, verified project boundaries |
| 4 custom physics | 5-7, stages 2/3/8 |
| 5 CCD / 6 moving hands / 7 response | 7-8, stages 3/4, explicit dynamic support limits |
| 8 networking / 9 prediction | 9-10, 12, stages 5/6 |
| 10 rollback / 11 validation | 8-9, 11-12, stage 7 |
| 12 visuals / 13 advanced physics / 14 events | 6-7, 13-14, stage 8 |
| 15 lifecycle / 16 performance / 17 debug | 5, 9, 13-14, stages 5/8/9 |
| 18 automated tests / 19 integration / 20 test environment | 2, 4, 16-17, tests 1-32 mapping |
| 21 scenarios A-I | 16 stage 9 and earlier stage-specific acceptance |
| 22 quality / 23 frameworks / 24 order / 25 files | 3-4, 16-17 |
| 26 done / 27 verification / 28 reporting / 29 priorities | 1, 15-18 and progress evidence; no implementation marked done |

Architecture planning and Stage 2 foundations are complete as scoped deliverables, with the evidence/limits above and in progress. No Stage 3 or later implementation is authorized. The next action is user review of Stage 2; only after explicit approval may Stage 3 begin. The existing Pawn, Blueprints, maps, Steam/plugin configuration, Build.cs and target files remain unchanged.
