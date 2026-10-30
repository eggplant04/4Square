# Multiplayer VR Handball: development progress

Last updated: 30 October 2026, Australia/Sydney. Workspace: S:/Github/4Square/FourSquareVR.

## Current stage and approval state

**Stage 3 C++ implementation, UE 5.6.1 Editor build and automated verification are complete. Final result: 52 passed, 0 failed, 0 test warnings, 0 not run. Rendered Editor placement/PIE acceptance remains manual and unverified. Stop here for user review. Stages 4-9 are not approved or started.**

At the beginning of every following session read in full, in order:

1. BALL_SYSTEM_SPEC.md.
2. BALL_IMPLEMENTATION_PLAN.md.
3. BALL_PROGRESS.md.

The specification remains the requirements source of truth; its SHA-256 is unchanged: 3C37DC3C28F1E0474BED79F1A327AB333404A55201797F670238C8707948CE79. The user's explicit stage gates override automatic continuation instructions in the specification.

## Entry verification and version control

Before any Stage 3 source edits, reread all three documents, inspected the Stage 2 source/configuration and verified a clean working tree at user-created commit abda108 (stage 2 checkpoint). The installed UE 5.6.1 Editor target built successfully and all 23 Stage 2 tests passed again.

Baseline evidence: Saved/Logs/BallStage3/BaselineBuild.log, BaselineTests.log and Saved/Automation/BallStage3/Baseline/index.json.

Git is usable. The repository root is S:/Github/4Square, above the writable project directory. Ordinary sandboxed discovery cannot chdir into that parent; approved read-only commands outside the sandbox succeed. Stage 3 used status, log and diff inspection only: no staging, commits, checkout, reset, clean, branch creation or other Git mutations. Current changes are uncommitted tracked edits and new source files, as listed below. Generated Binaries, Intermediate and Saved output remains ignored.

The existing Blueprint VR Pawn, motion controllers, VRTemplateMap, project/target configuration, Steam configuration and session plugins were preserved. The only dependency change is private PhysicsCore in Build.cs for read-only physical-material metadata. Ball simulation never uses a Chaos rigid body.

Final audit: Git diff --check passed with Windows CR line endings recognized; a separate source/document whitespace scan found zero issues. There are 52 Automation declarations matching the final report. Read-only diff/name inspection found no tracked paths outside the documented source/build/planning scope and no retained new Content assets. The specification hash was rechecked unchanged.

## Completed Stage 3 functionality

- Custom sphere CCD against a sealed analytic plane/sphere/rounded-box world and an Unreal static-world query adapter. Sphere/relative-sphere TOI uses a stable quadratic root; rounded boxes test faces, finite edge cylinders and vertices instead of treating the expanded AABB as exact geometry.
- Swept time-of-impact movement, quaternion drift to impact, response, minimum-translation separation, and continued simulation of the entire remaining interval. Acceleration microintervals remain inside the same configured fixed timestep; rendering FPS never changes dt.
- Simultaneous and near-simultaneous manifolds in stable collider/generation/feature order. A proximity query at impact includes previously tangential/supporting faces that become constraining after another impulse.
- A bounded normal-contact active-set solve: at most 92 nonempty subsets with one, two or three independent normals in 3D, plus the empty set. This solves acute corners without relying on slow sequential normal convergence. The same projection computes minimum separation and support reactions.
- Solid-sphere mass/inertia response, restitution, iterative Coulomb friction with angular impulse, final cone/non-penetration feasibility, low-speed restitution suppression, rolling/spin damping on balanced support and support-aware dwell/sleep/wake.
- Passive static collision energy checks. If coupled restitution/friction targets would add kinetic energy or cannot converge feasibly, retry that manifold inelastically, with an exposed RestitutionFallbacks diagnostic; fail transactionally if it still cannot be solved. This deliberately sacrifices corner elasticity to preserve passivity.
- Initial-overlap recovery, separating/tangent filtering, bounded zero-time contact processing, exact failure on impossible gaps, contact/candidate/iteration limits, and unchanged complete state with no partially published events on failure.
- Native placeable AHandballBall: non-simulating, collision-disabled proxy and mesh, configurable physics/history settings, initial velocity/spin, launch/reset/state/config APIs and debug drawing. Scene transforms only display simulator output. Actor scale is normalized; tune Physics.Radius instead.
- UBallSimulationSubsystem with a real TG_PostPhysics tick function, one shared accumulator clock, maximum 16 registered balls, transactional prepare/commit across all registered balls and preflighted history writes. Simulation faults pause the timeline. Explicit environment rebuild creates a new revision/generation/history/timebase boundary.
- Native ABallTestCourt containing a floor, four walls and optional ceiling, with ordinary static engine collision. No binary map or Blueprint was retained.
- FUnrealBallCollisionWorld: static-mobility colliders, response-channel/profile filters, stable canonical-path IDs, geometry/material/filter fingerprints, optional component physical materials, reused query storage and explicit invalidation. Static mobility is supported across object types; movable/simulating objects and BallIgnore-tagged components are excluded.
- Engine queries use multi-object sweeps, which UE 5.6.1 prefilters as touches to gather matching hits without a first-blocker cutoff. Earliest TOI selection and response remain custom. Compound-component corner testing verifies that two subshape normals are returned.
- Complete checkpoint format 2 adds feature IDs and still reads format 1, defaulting feature IDs to zero. Full state/input history, owned-copy semantics, pinned capacity, event ordinals and exact same-process collision replay remain intact.
- Raw collision facts include ball identity, category, frame/fraction, collider/feature, episode/ordinal, contact point/normal, pre/post linear/angular velocities and normal impulse. They are provisional simulation output, not committed scoring/elimination callbacks.
- CPU trace scopes, per-step query/microinterval/collision/penetration/fallback counters, replay counters, actual allocated history/cache sizes, ball/velocity/contact-normal debug drawing and four-speed engine replay benchmarks.

No hand components, Pawn changes, hand pose transport, RPCs, snapshots, client prediction, gameplay event ledger or retrospective server adjudication were implemented. Actor replication remains disabled until Stage 5's complete state protocol. Stage 2's configurable 250 ms finality policy is preserved; actual event finality enforcement is Stage 7.

## Files created or modified

Paths in this table are relative to Source/FourSquareVR unless a root path is stated. Paired files are explicitly named by directory and shared basename.

| Status | Files | Responsibility |
|---|---|---|
| New | Public/Ball/BallCollisionSolver.h; Private/Ball/BallCollisionSolver.cpp | Geometry kernels, contact projection, response, separation, support/cache |
| New | Private/Ball/BallCollisionWorld.cpp | Validated earliest manifold construction/order |
| New | Public/Ball/UnrealBallCollisionWorld.h; Private/Ball/UnrealBallCollisionWorld.cpp | Immutable static environment adapter, metadata, reused query storage |
| New | Public/Ball/HandballBall.h; Private/Ball/HandballBall.cpp | Placeable Actor, complete live state/history, APIs, presentation/debug |
| New | Public/Ball/BallSimulationSubsystem.h; Private/Ball/BallSimulationSubsystem.cpp | Shared fixed clock, native tick, bounded registration, prepare/commit and environment boundaries |
| New | Public/Ball/BallTestCourt.h; Private/Ball/BallTestCourt.cpp | Native test-court fixture |
| New | Private/Tests/Ball/BallCollisionTests.cpp | 17 analytic collision/response/stability cases |
| New | Private/Tests/Ball/BallCollisionReplayTests.cpp | Collision FPS/replay/events/performance plus legacy checkpoint migration |
| New | Private/Tests/Ball/BallWorldCollisionTests.cpp | Seven actual UWorld/Actor/court/compound/performance cases |
| Modified | Public/Ball/BallCollisionWorld.h | Query settings, manifold facts/results and gathering interface |
| Modified | Public/Ball/BallPhysicsConfig.h; Private/Ball/BallPhysicsConfig.cpp | Surface values, CCD/manifold/filter/material/candidate settings and validation |
| Modified | Public/Ball/BallPhysicsTypes.h; Private/Ball/BallPhysicsTypes.cpp | Feature identity, complete events, diagnostics, validation/capacities |
| Modified | Public/Ball/RecordedBallCollisionWorld.h; Private/Ball/RecordedBallCollisionWorld.cpp | Sealed plane/sphere/box reference geometry |
| Modified | Private/Ball/BallSimulator.cpp | Complete transactional collision/rest/remaining-time orchestration |
| Modified | Public/Ball/BallSimulationClock.h; Private/Ball/BallSimulationClock.cpp | Explicit fault pause with retained debt |
| Modified | Public/Ball/BallHistoryBuffer.h; Private/Ball/BallHistoryBuffer.cpp | Non-mutating write preflight for multi-ball transactions |
| Modified | Public/Ball/BallStateCodec.h; Private/Ball/BallStateCodec.cpp | Schema 2 and schema 1 migration |
| Modified | Private/Tests/Ball/BallCoreTests.cpp; Private/Tests/Ball/BallHistoryTests.cpp; Private/Tests/Ball/BallTestFixtures.h | Evolve unsupported-contact assertions into real response/support tests; query accounting, transactional fault fixture and populated feature IDs |
| Modified | FourSquareVR.Build.cs | Private PhysicsCore metadata dependency |
| Modified | Root BALL_IMPLEMENTATION_PLAN.md; BALL_PROGRESS.md | Actual implementation, decisions, measurements, limits and continuation gate |

Total: 14 new source files, 18 modified source/build files and two updated documents. No Content asset remains changed. A temporary failing Editor setup script was removed; its blank map was archived under Saved, not delivered as a usable level.

## Compilation and actual test results

Final target: FourSquareVREditor Win64 Development, installed UE 5.6.1 CL 44394996, compatible module build ID 43139311. Final compile/link succeeded, exit 0, 2.15 s incremental build. Evidence: Saved/Logs/BallStage3/BuildFinal.log. Earlier full Stage 3 builds compiled all new code/UHT types successfully.

Toolchain: bundled .NET 8.0.300; VS 2022/MSVC 14.44.35215, installed family 14.44.35207; Windows SDK 10.0.26100.0. Existing non-blocking warnings remain: MSVC is newer than preferred 14.38, and the targets retain Unreal5_3 include order. No target/toolchain edits were made to suppress them.

Initial Stage 3 compile errors in UHT-generated constructors (a native reference member and an incomplete unique-pointer pointee) were fixed before tests ran. These are resolved, not current build blockers.

| Actual UE Automation run | Passed | Failed | Test warnings | Not run | Notes |
|---|---:|---:|---:|---:|---|
| Stage 3 baseline | 23 | 0 | 0 | 0 | Before source changes |
| Run01 | 49 | 0 | 0 | 0 | Initial complete collision/engine suite |
| Run02 | 50 | 1 | 0 | 0 | Acute-corner test exposed missing tangential constraint/slow normal convergence |
| Run03 | 50 | 1 | 0 | 0 | Frictionless acute corner fixed; frictional closure still failed |
| Run04 | 52 | 0 | 0 | 0 | Coupled feasibility, slope wake and speed profiling passed |
| Final Run05 | 52 | 0 | 0 | 0 | Final source, process exit 0 |

Final suite groups: Core 15, History 9, Collision 21, World 7. Reports are under Saved/Automation/BallStage3/Run01 through Run05/index.json; full logs are Saved/Logs/BallStage3/Tests01.log through Tests05.log. UE's process exit alone did NOT indicate the failed Run02/03 tests; JSON and individual entries were inspected.

Executed coverage verifies analytic positions/velocities/times/normals/points, not merely hit booleans: floor/wall/ceiling at 24,000 cm/s; 0.1 cm thin box at up to 600,000 cm/s; real Unreal thin-wall sweep at 120,000 cm/s; near-tangent sphere roots and glancing/tangent planes; rounded edges/vertices; floor then wall in one 120 Hz step; simultaneous and acute corners; deep/zero-time contacts; impossible gaps and collision budgets; 1,200 frames in a valid 0.2 cm clearance channel with thousands of impacts and conserved speed; restitution/friction/inertia energy equations; 60 s analytic rest and 30 s real-world rest; force wake and a frictional slope that must roll; rendering rates 30/72/90/120/144/240 with identical full collision state; codec migration and corruption; full restore/replay with exact regenerated event identity/timing/response; real world filters, epoch invalidation, compound geometry and Actor launch/reset/config/transform projection.

Tolerances: constant-velocity analytic tests generally use 1e-7 cm or tighter for positions/velocities and 1e-9 normalized TOI or tighter; near-tangent roots use 1e-10. Unreal thin-wall position tolerance is 0.005 cm, velocity 0.01 cm/s, normalized TOI 2e-6, point 0.002 cm and normal 1e-5. These qualify the tested coordinates/simple geometry, not arbitrary mesh cooks or bitwise x64/ARM equivalence.

Exact build command:

```powershell
& 'S:\Program Files\EpicGames\UE_5.6\Engine\Build\BatchFiles\Build.bat' FourSquareVREditor Win64 Development '-Project=S:\Github\4Square\FourSquareVR\FourSquareVR.uproject' -WaitMutex -NoHotReloadFromIDE
```

Exact final test command:

```powershell
& 'S:\Program Files\EpicGames\UE_5.6\Engine\Binaries\Win64\UnrealEditor-Cmd.exe' 'S:\Github\4Square\FourSquareVR\FourSquareVR.uproject' -unattended -nop4 -nosplash -NullRHI -nosound -nohmd -nosteam '-ExecCmds=Automation RunTests FourSquareVR.Ball' '-TestExit=Automation Test Queue Empty' '-ReportExportPath=S:\Github\4Square\FourSquareVR\Saved\Automation\BallStage3\Run05' '-abslog=S:\Github\4Square\FourSquareVR\Saved\Logs\BallStage3\Tests05.log' -stdout -FullStdOutLogOutput
```

These process flags disabled rendering, XR, audio and Steam only for testing; project settings were not changed.

## Measured CPU and memory; no Quest qualification

Final Run05, Intel i5-12600K, Win64 Development Editor, NullRHI. Engine benchmark: six static court components, 512 forward frames, then 256 repetitions of the same 30-frame replay at each speed. Inputs are reproducible, not live VR/network traffic.

| Ball speed cm/s | Forward step median / p95 us | 30-frame replay median / p95 / max us | Impacts in replay |
|---:|---:|---:|---:|
| 1,000 | 10.002 / 10.200 | 374.2 / 395.2 / 555.2 | 2 |
| 3,000 | 9.701 / 15.300 | 358.2 / 392.7 / 552.4 | 2 |
| 10,000 | 9.798 / 16.801 | 374.4 / 413.5 / 1377.1 | 4 |
| 32,310.99 | 10.300 / 19.401 | 422.1 / 470.1 / 629.7 | 11 |

Peak broadphase candidates: two in this court. The fastest workload performed 28,672 scene queries across 256 replays (112 queries/replay). Replay timings include runner preflight/copy/validation and queries. Forward step timings are solver measurements, excluding Actor projection/copies, rendered frames and XR.

Analytic six-plane collision replay, 512 x 30 frames at 16,798 cm/s: median 72.8 us, p95 74.2 us, max 102.1 us, 158 queries/replay. Empty analytic free-flight replay: mean 38.219 us, median 37.7 us, p95 38.9 us, max 86.8 us. The old Stage 2 median of 8.2 us was an earlier contact-free foundation and is not the current simulation cost. Early Run01 real-world median was about 1.4 ms; CPU scheduling/power/startup variation affects results, so these are observed samples, not real-time guarantees.

| Native storage | Measured bytes |
|---|---:|
| Complete FBallState | 1,504 |
| State plus saved inputs, FBallHistoryFrame | 1,760 |
| 256-entry history including owner | 450,648 |
| Fully populated schema-2 checkpoint | 1,400 |
| One step output | 25,144 |
| Two Actor-owned output buffers | 50,288 |
| Native AHandballBall object | 5,216 |
| Shared static adapter cache, six-component fixture | 151,572 |

History/cache allocated sizes did not grow across benchmark writes/replays. One ball's native object/output/history total is 506,152 bytes, plus the shared world cache. This excludes UObject component allocations, shared assets, engine collision/scene-query internals, future hands/network/event ledgers and rendering. Four such balls plus this cache already use about 2.08 MiB of counted storage; maximum 16 registration is a safety cap, not a qualified Quest ball count.

Custom physics/history buffers do not allocate per impact. Engine multi-query internals may allocate; the reused 256-result TArray can grow before an excessive result is rejected. Do not describe the whole engine query path as allocation-free. Fingerprint validation scans registered static components repeatedly and needs larger-scene profiling/optimization before final rollback qualification.

No Quest hardware, Android package, ARM profiling, thermal test, four-player workload, network traffic or replay-burst p99 qualification was performed. The plan's <=1 ms/frame, <=2 ms replay p99 and <=2 MiB targets remain unproven.

## Manual Unreal Editor setup and outstanding acceptance

No usable .umap was retained. A headless Editor Python placement attempt crashed with EXCEPTION_INT_DIVIDE_BY_ZERO in the engine/Editor ActorFactory path after saving only a blank map; the exact native cause is unresolved. A fallback runtime-spawn Python API was not exported by this UE build. Neither attempt proves rendered Editor placement works or fails in an interactive session. Logs: Saved/Logs/BallStage3/TestLevelSetup.log and TestLevelSetup02.log. The blank artifact is preserved in Saved/Automation/BallStage3/FailedEditorSetup/BallStage3TestCourt.empty.umap. The failing script was removed.

Required manual setup:

1. Open the project with UE 5.6.1. Create a separate empty test level; save it under a new name such as /Game/Ball/Tests/BallStage3ManualTest. Preserve VRTemplateMap.
2. Place BallTestCourt at (0,0,0) and HandballBall at (0,0,100). The native court has floor z=0, inside walls x=+/-1000/y=+/-500 and ceiling z=1000 cm. Add lighting or use an unlit Editor viewport.
3. On the ball set InitialLinearVelocity=(1600,300,-500), bDebugDraw=true, or use console ball.DebugDraw 1. Defaults are radius 3 cm and 120 Hz. Use Physics.Radius rather than Actor scaling.
4. Start Simulate or standalone PIE. Verify visible floor/wall/ceiling bounces, normals, spin, rest and frame progression. During simulation select the runtime ball and exercise LaunchConfiguredBall/ResetBall, or call LaunchBall/SetBallState/SetBallPhysicsConfig from Blueprint. Buttons need an initialized playing ball; they are not an edit-mode simulator.
5. Verify physics parameters through the runtime setter. Frequency must match the shared world frequency and cannot be changed live on one ball.
6. Keep collision geometry static during a run. Geometry edits pause simulation. RebuildStaticEnvironment is an explicit subsystem API for a new epoch/history/generation/timebase; it is not historical rewind.
7. Check interactive actor placement (including the prior headless failure), real tick registration/order, visible debug lifetime, material tuning, frame pacing and existing Pawn/teleport/UI/grab regression in the normal VR map.

C++ UWorld/court/Actor integration was automated and passed. Rendered Editor placement/PIE, visible quality and physical headset regression were not performed. No multi-process, multi-machine Steam, VR hand, reconciliation or Quest result is claimed.

## Limits and architectural concerns for the following stages

- Defaults: 12 collision iterations per fixed step, up to 8 acceleration/retry microintervals, 8 simultaneous/persistent contacts, 64 facts/events per step, 64 broadphase components/query, 256 raw engine hits/query, 4,096 registered static components, 16 balls. Excess work or impossible geometry pauses/fails atomically, never discards remaining time or silently ignores a normal.
- Nearly dependent normal systems below pivot tolerance 1e-10, excessive contacts and unsupported MTD may still fail explicitly. Iterative Coulomb friction is not an exact global maximum-dissipation solve; passive fallback can make acute corners less bouncy. Properties are tunable game values, not calibrated rubber measurements.
- Default skin/penetration tolerance is 0.02 cm; support probes extend to 0.04 cm. Nearby constraints within skin may be activated early. Separation changes position but not velocity; potential-energy error under gravity and first-order force integration must be distinguished from the tested collision kinetic-energy inequality.
- Sweeps are continuous along each velocity drift. Stage 2 semi-implicit force integration is retained. CCDTolerance controls acceleration subdivision/chord planning, not a universal bound on global trajectory error. No exact continuous parabolic trajectory is claimed.
- Prefer closed simple collision. Complex triangle/open/one-sided/heightfield MTD, per-triangle physical materials and arbitrary mesh cooks are unqualified. The adapter uses component simple physical-material values and geometric-mean coefficient combination; engine material combine-mode flags are not used.
- Static geometry is a live but immutable epoch, not a copied historical Chaos scene. Component transform/bounds/body GUID/material/filter changes, static spawns and level changes invalidate it. In-place geometry/instance edits that preserve those fingerprints must explicitly invalidate/rebuild before simulation. Old-epoch replay is refused; moving/rotating/deforming geometry requires Stage 4's recorded historical contract.
- Same-process exact restore/event equivalence passed. Engine cooks, float query results, feature hashing near quantization boundaries and x64/ARM differences can change near-degenerate ordering. No cross-platform bitwise determinism is promised.
- Contact cache episodes can evict inactive features. They are not a durable network duplicate-hit ledger. Stage 4/7 must add tracked-hand epochs/sample identities and the specified event revision/deduplication protocol.
- Client clocks, hands, lag compensation, admission/rollback transactions, event commitment and presentation smoothing remain later work. Do not use SetBallState or Actor transforms as rollback; those APIs deliberately create fresh generations and clear history.
- Immediate predicted feedback remains a Stage 6 requirement. Future scoring/elimination, round changes, rewards and irreversible callbacks must wait for the configurable revision window AND the committed frontier with no admitted replay pending. Raw Stage 3 collision events cannot trigger irreversible gameplay.
- CPU/memory feasibility remains open until larger scenes, moving hands, latency/jitter/loss, actual multiplayer and Quest hardware are measured. No accuracy/rate/history requirement was weakened to meet a guessed budget.

## Remaining work and exact next steps

| Stage | Status |
|---|---|
| 1 inspection/architecture | Complete, reviewed |
| 2 foundations | Complete; original 23 tests and Stage 3 regressions passed |
| 3 static physics/Actor/adapter | C++ complete, build and 52 tests passed; manual Editor acceptance above remains |
| 4 VR tracking/hand CCD/historical dynamic proxies/Pawn integration | Not started, requires explicit approval |
| 5 owned transport/clocks/snapshots/origin/late join | Not started |
| 6 prediction/reconciliation/local feedback | Not started |
| 7 historical server adjudication/resimulation/finality ledger | Not started |
| 8 full presentation/events/advanced forces/debug workflow | Not started |
| 9 integrated network and hardware qualification | Not started |

Next session: read all three documents, inspect user/Git changes, review the Stage 3 results and unresolved manual checks. Fix only specifically requested Stage 3 issues until the user approves Stage 4. After that approval, implement recorded VR hand input/historical collider contracts before wiring Grip components, preserve complete state and rerun collision/replay/performance regression tests. Do not automatically continue.

## Session history

- 2026-10-29: read-only inspection, then complete specification/architecture planning; no gameplay edits.
- 2026-10-29/30: Stage 2 approved; Git access diagnosed before changes; 24 foundation/test files added; UE 5.6.1 build and 23 tests passed; stopped for review.
- 2026-10-30: Stage 3 approved; baseline reverified before edits; 14 source files added and 18 source/build files updated; acute/compound/slope issues tested and resolved; final Editor build and 52 tests passed; real static collision benchmarks recorded; headless asset attempt failed, native fixture/manual steps retained; stopped before Stage 4.
