# Multiplayer VR Handball: development progress

Last updated: 30 October 2026, Australia/Sydney.

## Current stage and approval state

**Stage 2 complete within its approved foundation scope. Build succeeded; 23 Automation tests executed and passed. Stopped for user review.**

Architecture and Stage 2 were explicitly approved with configurable finality, early performance evidence, and Git investigation requirements. **Stages 3-9 are not authorized and have not started.**

At the beginning of each following session read, in order:

1. `BALL_SYSTEM_SPEC.md` in full.
2. `BALL_IMPLEMENTATION_PLAN.md`.
3. This progress file.

The specification remains the requirements source of truth. Its SHA-256 remains `3C37DC3C28F1E0474BED79F1A327AB333404A55201797F670238C8707948CE79`. It was not edited. The user's stage approval boundaries override instructions in the specification to continue automatically.

## Completed functionality

- Blueprint-editable, validated physics configuration with documented units and separate configurable history/revision settings. Defaults target 120 Hz and a 250 ms revisable tail; increasing the window must also fit replay/history capacities.
- Boundary state semantics: State(F) is the start of fixed frame F. Full state contains epoch/ball/generation, config/material/environment versions, position/velocity, rotation/angular velocity, sleep/support, contact episodes/impulse caches, force-provider records, deterministic seed and event ordinal.
- Complete state copy/restore and lossless explicit-field checkpoint serialization. Versioned little-endian format, CRC, bounds, count/finite/rotation validation and transactional failure. No raw struct padding or UObject references. All live fields round-trip; inactive fixed-array slots are not meaningful state.
- Fixed accumulator clock with zero/multiple steps, per-render-frame catch-up limit, retained fractional time/debt, frame/timebase indexing and diagnostics. Server overload pauses explicitly; client overload requests a baseline. Recovery records explicitly discarded pause debt and does not fabricate simulated frames.
- Pure C++ numerical foundations: stable exponential linear/angular damping, semi-implicit force kick/drift, quaternion integration, optional designer speed limits, ordered recorded acceleration inputs and acceleration-based bounded microintervals.
- Collision-world interface and a bounded analytic half-space query fixture. Geometry becomes sealed on its first query. Collision facts are separate from movement response.
- Simulator transactions preserve the entire destination on query/numeric/capacity/version failures. Stage 2 explicitly returns ContactSolverRequired for colliding/resting/support/manifold states; it does not pretend Stage 3 collision response exists.
- Preallocated circular state/input history with exact identity/frame checks, contiguous writes, wrap handling, owned copies, generation reset and explicit pin protection against eviction of replay dependencies.
- Bounded, transactional replay runner that restores a complete checkpoint, validates dependencies and resimulates recorded numeric inputs. This is a foundation/feasibility primitive, **not server lag compensation, hit adjudication, networking, or the Stage 7 rollback manager**.
- Step/replay CPU trace scopes, elapsed timing and work counters; history memory/occupancy APIs and benchmark evidence. No automatic hot-path buffer growth.
- Finality eligibility calculation and modular packet-sequence comparison. This is not a final event ledger or scoring implementation.
- 23 real Unreal Automation tests covering the Stage 2 acceptance cases and validation regressions.

The existing VR Pawn, maps, Blueprint assets, Steam configuration, Advanced Sessions source, Build.cs and target files were not modified. No ball Actor or network/hand component was introduced.

## Finality and future scoring/elimination contract

RevisionWindowSeconds is configurable (default 0.25 s), rounded conservatively to fixed frames. History retention is longer than the accepted compensation window. Eligibility calculations avoid unsigned underflow at match startup and reject incompatible capacities.

Immediate predicted motion/audio/haptics remain the approved later-stage design. Irreversible double-bounce, out-of-bounds, last-hitter consequences, scoring, elimination, promotion, serve and round-reset decisions must wait for committed canonical events. An immediate tentative UI must support retraction. Do not eliminate a player or reset the rally from a provisional bounce: a late validated strike can remove or replace it.

The finality eligibility helper alone is insufficient to commit an event. The future coordinator/ledger must also account for admitted pending replay, pauses and boundaries, and keep commitment monotonic. Actual perceived scoring delay can exceed the configured window due to input cushion, pending work or network delivery. Changing the window requires a versioned timeline/config boundary and cannot reopen already committed history. None of the future prediction/event/scoring delivery behavior is claimed implemented in Stage 2.

## Git investigation and change audit

Completed **before gameplay source edits**:

- Ordinary sandboxed Git discovery failed because it cannot chdir into the parent repository.
- Approved read-only commands outside the sandbox succeeded: repository root is S:/Github/4Square; current commit is 46fd8e6 INIT WITH VR PLUGIN.
- Tracked source/configuration were clean at entry; BALL_SYSTEM_SPEC.md, BALL_IMPLEMENTATION_PLAN.md and BALL_PROGRESS.md were untracked.
- Final read-only git status --short --untracked-files=all, diff --check, diff --name-only and diff --stat succeeded.
- Final status showed only those three documents and the 24 new Stage 2 source files. No tracked-file differences were reported. diff --check does not validate untracked file contents; those were separately reviewed.
- **Version control is usable with the required parent-directory access.** No commit, staging, checkout, reset, clean, branch creation or other Git mutation was performed. Files remain untracked/uncommitted.
- Build/test-generated Binaries, Intermediate and Saved output is ignored by the existing repository rules.

## Files created or modified

All source paths below are relative to Source/FourSquareVR. Header/source pairs are listed explicitly by common basename; no future empty skeletons were created.

| Files | Status and responsibility |
|---|---|
| Public/Ball/BallPhysicsConfig.h; Private/Ball/BallPhysicsConfig.cpp | New; reflected settings, validation, configurable finality eligibility and sequence ordering |
| Public/Ball/BallPhysicsTypes.h; Private/Ball/BallPhysicsTypes.cpp | New; identities, boundary/contact/provider state, frame/input/output/query-contact values and validation |
| Public/Ball/BallDiagnostics.h; Private/Ball/BallDiagnostics.cpp | New; dedicated log category and bounded cost/work aggregation |
| Public/Ball/BallSimulationClock.h; Private/Ball/BallSimulationClock.cpp | New; accumulator, limits, pause/baseline recovery and timebase diagnostics |
| Public/Ball/BallCollisionWorld.h | New; pure query interface and explicit query result |
| Public/Ball/RecordedBallCollisionWorld.h; Private/Ball/RecordedBallCollisionWorld.cpp | New; bounded/sealed analytic plane query fixture |
| Public/Ball/BallIntegrator.h; Private/Ball/BallIntegrator.cpp | New; force kick/damping, optional clamps, drift/angular integration |
| Public/Ball/BallSimulator.h; Private/Ball/BallSimulator.cpp | New; transactional fixed-step free-flight orchestration and explicit unsupported-contact result |
| Public/Ball/BallStateCodec.h; Private/Ball/BallStateCodec.cpp | New; bounded/versioned/checksummed complete checkpoint persistence |
| Public/Ball/BallHistoryBuffer.h; Private/Ball/BallHistoryBuffer.cpp | New; complete state/input ring and dependency pinning |
| Public/Ball/BallReplayRunner.h; Private/Ball/BallReplayRunner.cpp | New; bounded preflight/restore/replay and reports |
| Private/Tests/Ball/BallTestFixtures.h | New; rich persistent-state and recorded-input fixtures |
| Private/Tests/Ball/BallCoreTests.cpp | New; 15 Core Automation cases |
| Private/Tests/Ball/BallHistoryTests.cpp | New; 8 History Automation cases |
| BALL_IMPLEMENTATION_PLAN.md (project root) | Updated approval/status, actual Stage 2 type/layout details, scoring consequences, early measurements and Git findings |
| BALL_PROGRESS.md (project root) | Updated actual implementation/build/test/continuation record |

Source total: 24 new files. Project documentation changed: the two planning/progress files. The user-supplied specification is unchanged.

## Compilation status and build results

**Final Editor target compilation/link: succeeded with Unreal Engine 5.6.1, CL 44394996.** UHT generated the reflected configuration successfully.

Environment used: bundled .NET 8.0.300, Visual Studio 2022, MSVC compiler 14.44.35215 (installed family 14.44.35207), Windows SDK 10.0.26100.0.

| Attempt | Actual result | Evidence |
|---|---|---|
| Sandboxed Build.bat launch | Failed before compilation because engine-directory access was denied by sandbox; misleading batch location error | Tool output; resolved by approved unsandboxed execution |
| Build 01 | Failed compilation: test flags declared uint32 instead of UE 5.6 scoped EAutomationTestFlags; corrected in both test sources | Saved/Logs/BallStage2/Build01-Failed.log; exit 1; 106.99 s |
| Build 02 | Succeeded after test flag correction; initial 20-test suite build | Saved/Logs/BallStage2/Build02-Succeeded.log; exit 0; 2.42 s |
| Build 03 | Succeeded after final state/replay validation fixes and three additional regression tests | Saved/Logs/BallStage2/Build03-Succeeded.log; exit 0; 2.08 s |

Remaining non-blocking build warnings: MSVC 14.44 is not UE's preferred 14.38; existing target uses backward-compatible Unreal5_3 include order. Targets/toolchain were not altered to suppress these warnings.

Exact build command (PowerShell, approved outside sandbox):
```powershell
& 'S:\Program Files\EpicGames\UE_5.6\Engine\Build\BatchFiles\Build.bat' FourSquareVREditor Win64 Development '-Project=S:\Github\4Square\FourSquareVR\FourSquareVR.uproject' -WaitMutex -NoHotReloadFromIDE
```

## Automated tests actually executed

Both runs used the compiled UE 5.6.1 Editor module in UnrealEditor-Cmd, not an external substitute or static-only check.

| Run | Passed | Failed | Warnings | Not run | Process exit | Report |
|---|---:|---:|---:|---:|---:|---|
| Run01 | 20 | 0 | 0 | 0 | 0 | Saved/Automation/BallStage2/Run01/index.json |
| Final Run02 | 23 | 0 | 0 | 0 | 0 | Saved/Automation/BallStage2/Run02/index.json |

Final Automation report's aggregate test duration: 0.179 s, excluding editor startup/discovery. Full logs: Saved/Logs/BallStage2/Tests01.log and Tests02.log. Headless testing disabled XR/audio/Steam for the process; no project configuration was changed.

Final executed tests, all under FourSquareVR.Ball:

| Suite | Successful test names |
|---|---|
| Core | AnalyticQueryAndUnsupportedContact; ClockInvalidInput; ClockOverrunRecovery; ClockRemainderAndCatchUp; ConfigurationAndFinality; ConfigurableFixedFrequency; DampingAndAngularIntegration; Gravity; PerformanceCounters; PersistentStateAndQueryFailure; RecordedForceOrderingAndVersions; RenderingRateConsistency; SequenceWrap; SubdivisionAndFailureAtomicity; ZeroAndInvalidState |
| History | BoundedReplayPerformance; CheckpointCorruptionAndBounds; CompleteCheckpointRoundTrip; PinnedCapacityAndGeneration; RecordedReplayAndRestoreContinue; ReplayBudgetsAndAtomicFailure; RingWrapAndInvalidFrames; ZeroLengthReplayValidation |

Coverage includes specification tests 1, 2, 7, 8 and 19-23 within the Stage 2 analytic/free-flight scope. Rendering rates tested: 30/45/60/72/90/120/144/240 Hz over equal elapsed time. Separately configured fixed frequencies 60/100/240 Hz work at 90 Hz rendering. Snapshot/RPC/hand/scoring tests assigned to later stages have not been executed.

Checkpoint tests populate every contact/provider slot and all support/sleep/version/seed/ordinal fields. They verify serialized restore/continue, all truncation lengths, CRC/schema/trailing bytes, semantically invalid data with recomputed CRC, count/size bounds and unchanged destination on failure. History tests cover eviction, owned-copy semantics, pinned-capacity rejection, generations, missing frames and no allocation growth. Replay tests verify recorded results, hard work caps, no partial destination/history updates, incompatible environments and even zero-step validation.

Exact final test command:
```powershell
& 'S:\Program Files\EpicGames\UE_5.6\Engine\Binaries\Win64\UnrealEditor-Cmd.exe' 'S:\Github\4Square\FourSquareVR\FourSquareVR.uproject' -unattended -nop4 -nosplash -NullRHI -nosound -nohmd -nosteam '-ExecCmds=Automation RunTests FourSquareVR.Ball' '-TestExit=Automation Test Queue Empty' '-ReportExportPath=S:\Github\4Square\FourSquareVR\Saved\Automation\BallStage2\Run02' '-abslog=S:\Github\4Square\FourSquareVR\Saved\Logs\BallStage2\Tests02.log' -stdout -FullStdOutLogOutput
```

Run01 used the same command with Run01 and Tests01.log paths. Verify JSON counts and process exit on reruns; process launch/compilation alone is not test success.

## Early performance and memory evidence

Actual final PC fixture, Intel Core i5-12600K, Win64 Development Editor:

| Measurement | Result |
|---|---:|
| sizeof(FBallState) | 1,504 bytes |
| sizeof(FBallHistoryFrame), including copied numeric inputs | 1,760 bytes |
| 256-frame history allocation including owner | 450,648 bytes (~440 KiB) |
| Encoded fully populated checkpoint | 1,368 bytes |
| Replay workload | 512 repetitions of 30 fixed frames (15,360 replay steps) |
| Mean / median 30-frame replay | 8.306 / 8.200 microseconds |
| p95 / maximum observed | 8.400 / 41.600 microseconds |
| History storage growth across writes/replays | None |
| Failed benchmark replay requests | 0 |

CPU scopes: Handball_SimulateStep and Handball_BoundedReplay. Counters distinguish attempted/failed steps, queries, microintervals, replay attempts/failures/executed frames and measured durations. Reports expose replay query/microinterval work. GetAllocatedBytes reports actual ring allocation, not a guessed struct budget.

**Limitations:** this is a free-flight fixture using an empty analytic collision world. It excludes Unreal scene queries, static contact resolution, moving palms/colliders, event revision ledgers, serialization/transport and network processing. It does not measure the complete world's memory, global allocations, real-time multiplayer, Android/ARM or Quest. No CPU/memory budget is assumed achievable on Quest from these results.

Required feasibility checkpoints before Stage 7: repeat bounded replay/memory measurement with real world collision at Stage 3, captured/multiple hands and historical dynamic geometry at Stage 4, and real network/prediction traffic at Stages 5-6. Profile an actual Quest build/device before claiming its budget passes.

## Remaining stages and manual tests

| Stage | Status |
|---|---|
| 1 inspection/architecture | Complete; architecture approved |
| 2 foundations | Implemented, compiled, 23 tests passed; stopped for review |
| 3 static CCD/response/rest, Actor/UWorld adapter/test court | Not started; needs approval |
| 4 tracking/relative hand CCD/dynamic proxies/Pawn integration | Not started |
| 5 owned RPCs/clocks/snapshots/origin/late join | Not started |
| 6 prediction/reconciliation | Not started |
| 7 historical server validation/resimulation/finality ledger | Not started |
| 8 presentation/events/spin/forces/debug workflow | Not started |
| 9 integrated network/hardware qualification | Not started |

Manual verification still required when dependent implementation exists: place/launch/reset the ball; UWorld sweep edge cases and tick order; existing Pawn regression; real palm calibration/strikes/tracking loss/recenter; visual/haptic corrections; real two-process and two-machine Steam/VR; scenarios A-I; dedicated-server and Android/Quest setup, performance and x64/ARM comparison. **None was represented as passed by the Stage 2 headless tests.**

## Known limitations and unresolved decisions

- No known failing Stage 2 tests. This is a standalone numerical/history foundation, not a playable ball or completed multiplayer feature.
- State capacities are eight contacts/four persistent provider records. Histories are initialized with explicit bounded capacities; future contact/hand/event capacity exhaustion must remain explicit and tested.
- Sleep/support/contact state is fully preserved but response/rest evolution awaits Stage 3. Contact-capacity and solver edge cases are not falsely covered by free-flight tests.
- Only analytic planes are implemented in the recorded collision fixture. Unreal geometry, moving sphere/box CCD, historical hand input and server event ordering remain later work.
- The future server event ledger must enforce actual finality in addition to the current eligibility calculation; speculative scoring is not authorized.
- Mutable-tail CPU and total memory may exceed the proposed budgets after real collision/VR/network work; early measurements are a baseline, not a guarantee.
- Existing Pawn origin/teleport authority, clock uncertainty, residual spoofing and remote pose prediction remain documented architecture risks.
- Approximate x64/ARM reproducibility is unverified. Same-platform test equivalence does not promise bitwise determinism across platforms.
- No catch/grab mechanics, ball-ball collision, arbitrary rotating/deforming mesh CCD or two-way Chaos coupling is claimed.
- Source remains untracked/uncommitted. Git is usable outside the sandbox; no commits were requested/performed.
- No major architecture change or specification conflict arose. Complete-state fields and boundaries were preserved. Helper services and consolidated files refine Stage 2 without replacing its architecture.

## Exact next steps

1. Read all three documents in order and check user/spec/source changes.
2. Present/review Stage 2 results. **Do not begin Stage 3 without explicit approval.**
3. If changes are requested to Stage 2, make only those changes, rebuild and execute affected tests, and update this record.
4. After explicit Stage 3 approval, implement static world adapter, actual contact/remaining-time response, robust support/rest and the placeable Actor/coordinator. Replace the explicit ContactSolverRequired boundary with tested behavior; do not silently bypass it.
5. Preserve state/codec version discipline and test full history restoration as contact state becomes active. Update plan/inventory if fields or helper boundaries change.
6. Repeat performance instrumentation with real scene queries/contact-heavy 30-frame replay before treating rollback feasibility as established. Investigate Quest on device; do not infer feasibility from this PC fixture.
7. Continue leaving Pawn, Steam/session plugins and unrelated configuration intact until their specifically approved integration stage.

## Session history

| Date | Result |
|---|---|
| 2026-10-29 | Read-only project inspection; no files changed |
| 2026-10-29 | Full specification analysis and two planning documents; no gameplay implementation/build/tests |
| 2026-10-29 to 2026-10-30 | Architecture/Stage 2 approved; Git access diagnosed before gameplay edits; 24 foundation/test files added; one compile failure fixed; final UE 5.6.1 build succeeded; 23 Automation tests passed; measurements/limits documented; stopped before Stage 3 |
