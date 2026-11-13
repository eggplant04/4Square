# CODEX MASTER IMPLEMENTATION PROMPT — MULTIPLAYER VR HANDBALL PHYSICS SYSTEM

## YOUR ROLE

You are a senior Unreal Engine 5 C++ engineer specialising in:

- Custom real-time physics simulation
- Continuous collision detection
- Multiplayer networking and replication
- Client-side prediction and reconciliation
- Server-authoritative physics
- Rollback and lag compensation
- Virtual reality interaction systems
- Performance optimisation for standalone VR hardware
- Reusable, modular gameplay architecture

Your task is to **design, implement, integrate, debug and test a complete production-oriented multiplayer VR ball physics system** inside my existing Unreal Engine 5 project.

Do not simply give me an explanation, implementation plan, pseudocode, or empty class skeletons.

**I want you to actually create and modify the necessary C++ header and source files in my project, compile them where possible, identify and fix errors, and deliver a working system.**

This is a substantial implementation. Work through it methodically, but do not stop after creating the first functional prototype. Continue through the complete specification, subject to genuine technical or environment limitations.

---

# 1. PROJECT BACKGROUND

I am developing a multiplayer virtual reality game based on Australian school handball, also known as four-square.

Players stand in different squares on a court and physically strike a bouncing rubber ball using their hands.

The ball is the most technically important part of the game.

The entire project depends on making this one ball feel extremely responsive, consistent, accurate and fair in multiplayer.

The project currently uses:

- Unreal Engine 5
- C++
- Unreal Engine's VR framework
- Steam multiplayer sessions
- Advanced Sessions plugin
- Networked VR players and motion controllers

The project is intended to support PCVR initially, with standalone Meta Quest hardware as an important future target.

**Before implementing anything, inspect the repository.**

Determine:

1. Exact Unreal Engine version.
2. Actual C++ module name and API export macro.
3. Existing `.uproject` and `.Build.cs` configuration.
4. Current VR Pawn implementation and Blueprint inheritance.
5. Existing left and right motion controller components.
6. Existing multiplayer architecture.
7. How players are spawned and possess their pawns.
8. Existing collision channels and physics settings.
9. Whether the project uses OpenXR, SteamVR, Meta XR, or another tracking setup.
10. Existing coding conventions and useful reusable functionality.

Adapt the implementation to the project rather than inventing new project names or breaking existing functionality.

Do not replace the existing multiplayer session system, VR pawn, or GameMode unless genuinely necessary.

Do not introduce unnecessary third-party dependencies.

---

# 2. NON-NEGOTIABLE DESIGN PRINCIPLES

The implementation must follow these principles.

## 2.1 Custom physics

Do not use Chaos rigid-body simulation to determine ball movement.

Specifically:

- Do not enable `SimulatePhysics` on the ball.
- Do not rely on Chaos rigid-body velocity replication.
- Do not use Blueprint Tick as the primary physics solver.
- Do not make the ball depend on Unreal's physics substepping settings.

Instead, implement a custom C++ simulation that explicitly calculates the ball's motion and collisions.

Unreal's collision queries may be used for static world geometry.

All critical movement and collision response calculations must be controlled by our implementation.

## 2.2 Fixed timestep

The physics must advance using a fixed delta time.

Default target: **120 Hz**.

```cpp
constexpr double FixedDeltaTime = 1.0 / 120.0;
```

The timestep must be configurable.

Do not use variable rendering-frame DeltaTime directly for ball integration.

Implement an accumulator-based fixed-step scheduler, including:

- Multiple steps per rendering frame when necessary.
- Zero-step frames when appropriate.
- Maximum catch-up steps.
- Frame-stall handling.
- Simulation frame indexing.
- Performance and overrun diagnostics.
- Explicit handling if the simulation cannot keep up.

Do not silently change physics timestep length to compensate for slow frames.

Do not silently discard simulation time in a manner that permanently desynchronises networked clients.

Client recovery and authoritative-server overrun behaviour must be clearly defined.

## 2.3 Server authority

The server is the final authority for:

- Ball position
- Ball linear velocity
- Ball angular velocity
- Accepted hand collisions
- Bounce events
- Simulation resets
- Gameplay-relevant contact history

Clients must never be allowed to authoritatively teleport the ball or set its velocity.

## 2.4 Client-side prediction

Local players must see their own hand strikes affect the ball immediately, without waiting for the network round trip.

Clients run a predicted local simulation, which is corrected against authoritative server states.

## 2.5 Modularity

Physics, collision detection, networking, VR tracking, rendering and gameplay events must be separated.

New features such as spin, Magnus force, alternative ball types and alternative court surfaces must be possible without rewriting the fundamental system.

## 2.6 Reproducibility

The simulation must support:

- Saving state.
- Restoring state.
- Running a single simulation step.
- Replaying recorded inputs.
- Testing predicted versus authoritative outcomes.

Aim for reproducibility within documented numerical tolerances, not an unsupported promise of bit-identical floating-point determinism across PC and ARM hardware.

Use stable ordering, explicit inputs and consistent numerical methods.

---

# 3. REQUIRED CLASS ARCHITECTURE

Use the following architecture as the starting point.

You may refine the class boundaries after examining the project, but preserve the separation of responsibilities.

## 3.1 Ball Actor

Create:

`AHandballBall`

Responsibilities:

- Own the ball components.
- Own the current authoritative or predicted simulation state.
- Coordinate fixed-timestep execution.
- Provide Blueprint-accessible functions.
- Expose configurable settings.
- Coordinate simulation, collision, networking and visual presentation.
- Dispatch collision and gameplay events.
- Support spawning, resetting and late joining.
- Cleanly initialise and destroy simulation resources.

The Actor must work when placed directly into a level.

Its physics movement must be generated by the custom simulator.

Disable default movement replication if it would conflict with custom state replication.

The server must replicate authoritative ball state through the networking system.

### Components

Provide appropriate components for:

- Scene root
- Ball collision representation
- Visual mesh
- Network synchronisation
- Visual smoothing and presentation

Use Unreal reflection and component conventions correctly.

Ensure the ball's collision representation does not create duplicate collision response from Chaos.

The ball's visual mesh must not simulate physics.

Provide a usable default appearance if possible with existing engine assets.

The actual collision sphere radius must be independent of visual mesh scale.

## 3.2 Ball state

Create:

`FBallState`

This is the complete dynamic state needed to restore and continue simulation.

At minimum include:

```cpp
FVector Position;
FVector LinearVelocity;

FQuat Rotation;
FVector AngularVelocity;

uint64 SimulationFrame;
```

Also include appropriate state for:

- Active/resting state.
- Current contact state.
- Simulation generation or reset ID.
- Any persistent physics state required for replay.

Avoid placing unnecessary UObject references inside the inner simulation state.

Use valid Unreal types and reflection only where necessary.

Make the state easy to compare, copy, serialise and restore.

Separate dynamic state from configuration.

## 3.3 Ball configuration

Create:

`FBallPhysicsConfig`

Or a suitable configuration Data Asset containing:

- Ball radius
- Ball mass
- Gravitational acceleration
- Air drag
- Restitution
- Surface friction
- Spin friction
- Angular drag
- Maximum linear speed
- Maximum angular speed
- Sleep threshold
- Wake threshold
- Minimum meaningful collision speed
- CCD tolerance
- Penetration correction tolerance
- Maximum collision iterations
- Fixed simulation frequency

Expose relevant parameters to the Unreal Editor and Blueprints.

Use centimetres, seconds and kilograms consistently with Unreal conventions.

Document the units of all parameters.

Choose sensible default values for a school handball-sized rubber ball.

Do not claim default physics values are experimentally accurate. They should be tunable.

Runtime configuration must be synchronised where needed so server and clients simulate using equivalent values.

## 3.4 Physics simulator

Create:

`FBallSimulator`

This must be a reusable, primarily plain-C++ simulation class.

Its responsibility is to advance a ball state through one fixed simulation interval.

It must not depend directly on the ball Actor for core mathematical calculations.

Provide an interface equivalent to:

```cpp
void SimulateStep(
    FBallState& State,
    const FBallStepInputs& Inputs,
    const FBallPhysicsConfig& Config,
    double FixedDeltaTime,
    FBallStepOutput& Output
);
```

The actual signature may be refined.

`FBallStepInputs` must represent relevant input data, moving colliders and collision environment access.

`FBallStepOutput` should contain contact events and other simulation results.

Avoid UObject allocation, heap allocation and unnecessary dynamic memory during each physics step.

Keep the collision-query integration separate from the physics maths where practical.

---

# 4. CUSTOM BALL PHYSICS

Implement the following.

## 4.1 Gravity

Use configurable gravity.

Default should approximate Earth's gravitational acceleration.

All calculations must respect Unreal units.

## 4.2 Integration

Use a stable integration method, such as semi-implicit Euler, appropriate for fixed-step game physics.

Do not update position using an inconsistent combination of previous and new velocities.

Preserve the ability to introduce a different integrator later.

## 4.3 Linear drag

Provide optional configurable air resistance.

Use a numerically stable model.

Avoid allowing drag to reverse velocity unexpectedly.

## 4.4 Velocity clamping

Provide optional maximum linear and angular velocities.

Do not arbitrarily cap realistic ball behaviour with unexplained constants.

## 4.5 Sleep and resting contact

Implement robust handling for when the ball settles.

Requirements:

- Detect near-rest conditions.
- Avoid endless small bounces.
- Avoid jitter on flat floors.
- Prevent the ball slowly sinking into geometry.
- Wake the ball when struck.
- Handle the ball resting against an angled surface where practical.

Do not implement sleeping solely through a low speed threshold if gravity would immediately cause an invalid result.

## 4.6 Spin readiness

The initial state must support rotation and angular velocity.

Implement basic angular integration and angular damping.

If full angular contact physics is implemented, use the appropriate moment of inertia for a solid sphere or another explicitly documented model.

Do not fake visual rotation independently of the simulated angular state when it affects prediction or replay.

---

# 5. CONTINUOUS COLLISION DETECTION

This is a critical requirement.

**The ball must not rely on discrete overlap checks to detect high-speed collisions.**

Implement a dedicated collision system.

Suggested classes:

- `FBallCollisionSolver`
- `FBallContact`
- `FBallCollisionQuery`
- A collision-world adapter interfacing with Unreal scene queries

## 5.1 Swept sphere movement

Calculate the ball's intended movement during each physics step.

Sweep its collision sphere through the intended path.

Determine the earliest valid collision.

For static Unreal geometry, use suitable sphere sweep queries.

Requirements:

- Correct radius.
- Collision channel filtering.
- Ignore the ball Actor itself.
- Ignore purely cosmetic objects.
- Support court floors and walls.
- Support appropriate world-static geometry.
- Allow configurable collision profiles.

Do not blindly accept the first arbitrary query hit if multiple hits require stable resolution.

## 5.2 Time of impact

Every collision must calculate or receive a meaningful time of impact within the current simulation step.

When collision occurs:

1. Advance the ball to contact time.
2. Determine contact normal and contact point.
3. Resolve collision response.
4. Integrate remaining simulation time.
5. Repeat if another collision occurs.

Do not automatically discard remaining timestep time after a collision.

## 5.3 Multiple collisions per step

Support multiple contacts within the same fixed timestep.

For example:

- Ball hits a hand.
- Ball travels into the floor.
- Ball rebounds before the timestep finishes.

Set a configurable maximum number of contact iterations.

Prevent zero-time collision loops.

Handle corner contacts and near-simultaneous contacts as robustly as possible.

## 5.4 Initial penetration

Support initial overlap and penetration recovery.

The ball must not permanently become stuck in a surface.

Implement:

- Safe separation direction.
- Penetration correction.
- Collision skin tolerance.
- Numerical epsilon handling.
- Sensible recovery from initial overlaps.

## 5.5 Static versus dynamic geometry

Do not assume moving geometry is equivalent to a stationary object.

Support moving hand colliders explicitly.

Other moving world geometry can use an extensible collider interface.

If arbitrary dynamic mesh CCD is outside the implemented scope, keep it isolated and clearly document that limitation rather than falsely treating it as solved.

---

# 6. MOVING VR HAND COLLISION DETECTION

This is one of the most important systems.

Players must be able to physically swing their hands at the ball.

Simply checking whether the hand currently overlaps the ball is unacceptable.

## 6.1 Hand collision component

Create a reusable component such as:

`UVRHandCollisionComponent`

Requirements:

- Attachable to an existing VR pawn.
- Supports left and right hands.
- Supports explicit association with a `UMotionControllerComponent`.
- Can locate suitable existing motion controllers when possible.
- Supports a configurable palm collision offset.
- Exposes hand collision radius.
- Provides tracking pose samples to the ball simulation.
- Works without requiring physical Chaos collision.
- Does not break existing VR movement or grabbing systems.

Do not create a completely new VR pawn if the project already contains one.

If Blueprint integration is necessary, provide public Blueprint-callable registration and configuration functions.

## 6.2 Tracking history

Store hand pose samples containing:

- World-space position.
- World-space orientation.
- Timestamp.
- Sample sequence ID.
- Left/right hand identifier.
- Player identifier.
- Tracking validity.
- Linear velocity where appropriate.
- Angular velocity where appropriate.

Use a bounded history buffer.

Distinguish tracking sample timestamps from arbitrary network arrival times.

## 6.3 Velocity estimation

Calculate hand velocity using differences in timestamped tracking positions.

Avoid assuming the rendering DeltaTime is always equal to the sampling interval.

Handle:

- Small time intervals.
- Zero or invalid DeltaTime.
- Tracking jitter.
- Sudden teleport-like jumps.
- Controller tracking loss.
- Reacquired tracking.
- Rapid direction changes.

Use bounded filtering where needed, but do not introduce excessive latency.

## 6.4 Hand shape

Initial implementation:

**Swept sphere representing the palm.**

The design must allow future replacement with:

- Capsule
- Multiple spheres
- Compound collision proxy
- Hand-tracking skeletal collision

Do not claim these advanced shapes are implemented unless they actually work.

## 6.5 Relative-motion CCD

Implement continuous collision detection between the moving ball and moving hand.

Do not perform two independent static sweeps and assume they completely solve moving-versus-moving collision.

For the initial sphere-versus-sphere implementation, solve relative motion mathematically.

Let:

- Ball start position be B0.
- Ball end position be B1.
- Hand start position be H0.
- Hand end position be H1.
- Combined radius be R.

Calculate relative starting position and relative displacement, then solve for the earliest valid intersection of the swept spheres.

Use the quadratic intersection condition:

```text
|RelativeStart + RelativeDisplacement * t|² = R²
```

Where `t` is normalised to the interval [0,1].

Handle:

- No solution.
- Tangential contact.
- Objects moving apart.
- Initial overlap.
- Extremely small relative displacement.
- Nearly zero discriminants.
- High-speed opposing movement.
- Hand and ball crossing between samples.

Calculate contact time, position and normal consistently.

Where acceleration meaningfully changes the trajectory during a step, use appropriately bounded subdivisions or another documented approximation instead of claiming straight-line sweeps are exact.

## 6.6 Angular hand movement

Track hand orientation and angular velocity.

Support calculating velocity at an offset contact point using the appropriate angular contribution.

For a pure spherical hand proxy, recognise that rotation does not change sphere geometry.

Do not invent rotational contact effects inconsistent with the selected collision model.

Design the interface for future capsule and palm-surface collision handling.

## 6.7 Continuous contact

Prevent the ball from receiving repeated artificial impacts because a hand remains inside its collision volume.

Implement correct contact-state management:

- Distinguish contact from new impact.
- Require appropriate separation or a physically valid new closing impact before another hit.
- Avoid generic long cooldowns that block legitimate rapid interactions.
- Make hit deduplication deterministic.

---

# 7. COLLISION RESPONSE AND BOUNCING

Implement physically coherent collision response.

The collision solver must consider:

- Ball velocity.
- Collision normal.
- Surface velocity.
- Relative closing velocity.
- Restitution.
- Tangential friction.
- Ball mass.
- Angular state where applicable.

For a static wall or floor, surface velocity is zero.

For a kinematic hand, surface velocity comes from tracking data.

Use a consistent normal convention.

For example, with the normal pointing from obstacle toward ball, contact is closing when:

```cpp
FVector RelativeVelocity =
    BallVelocity - SurfaceVelocity;

float NormalSpeed =
    FVector::DotProduct(
        RelativeVelocity,
        ContactNormal
    );
```

A negative normal speed indicates approach under this convention.

Apply collision impulses only when appropriate.

Ensure post-contact velocity is not still penetrating the surface due to sign errors.

## 7.1 Hand impact response

Fast hand movement should generally impart greater velocity to the ball than slow hand movement.

The response must account for impact direction.

Requirements:

- Strong forward swings.
- Gentle taps.
- Glancing strikes.
- Sideways deflections.
- Hand moving away from the ball.
- Ball moving toward a stationary hand.
- Opposing hand and ball movement.

Use a physically coherent response as the default.

Expose optional gameplay modifiers separately.

## 7.2 Energy stability

Prevent artificial energy accumulation from:

- Repeated overlap correction.
- Multiple collision iterations.
- Stationary contact.
- Replayed collisions.
- Tiny numerical errors.

Add automated tests for unrealistic increases in velocity or energy.

## 7.3 Surface materials

Allow different collision properties for different surfaces.

For example:

- Concrete court.
- Rubber-like surfaces.
- Walls.
- Hand colliders.

Provide configurable restitution and friction per surface where appropriate.

Avoid hardcoding level-specific logic inside the physics simulator.

---

# 8. NETWORKING ARCHITECTURE

Implement a custom multiplayer synchronisation system designed for this ball.

Suggested component:

`UBallNetworkComponent`

The system must support:

- Listen servers.
- Remote clients.
- Dedicated servers where practical.
- Multiple players.
- Late joining.
- Respawning/resetting the ball.

Use Unreal's supported replication and RPC systems for the detected engine version.

## 8.1 Authoritative server simulation

The server must run the authoritative ball simulation.

It must independently validate hand collisions from timestamped tracking data.

Never trust a client's claimed ball velocity or impact result.

## 8.2 Client simulation

Clients must maintain a local predicted ball state.

Predicted collisions with local hands must be applied immediately.

Other players' hand interactions should be handled using received tracking information and authoritative corrections.

Do not pretend that a client can accurately know remote players' future hand movement.

## 8.3 Network snapshots

Create a network snapshot structure containing the information needed to synchronise the complete dynamic ball state.

Suggested structure:

`FBallNetworkSnapshot`

Include:

- Simulation frame.
- Simulation generation/reset ID.
- Position.
- Linear velocity.
- Rotation.
- Angular velocity.
- Resting state.
- Relevant contact state where necessary.
- Latest acknowledged event/input information as needed.

Implement efficient replication.

Start with approximately 20–30 snapshots per second, configurable independently of simulation frequency.

Do not send the complete ball state every physics step unless profiling demonstrates that it is necessary and acceptable.

Use quantisation where accuracy and range requirements permit.

Do not blindly quantise timestamps or velocities so aggressively that collision prediction becomes inaccurate.

## 8.4 Custom replication

Do not rely exclusively on `SetReplicateMovement(true)` for the ball.

Implement authoritative snapshot replication and local state application.

Avoid competing systems independently writing the ball transform.

The simulation state, collision proxy and rendered mesh must have clearly defined ownership.

## 8.5 Hand input networking

Each locally controlled player must send timestamped hand movement information to the server.

Requirements:

- Batched hand pose updates.
- Sequence IDs.
- Timestamps.
- Left/right identification.
- Tracking validity.
- Sensible transmission rate.
- Bandwidth-conscious representation.
- Rejection of malformed data.
- Handling out-of-order and duplicate samples.
- Handling packet loss.

Use unreliable transmission for frequent replaceable pose data where appropriate.

If necessary, include overlapping samples from recent history so isolated packet loss does not destroy essential motion information.

Do not use reliable RPCs every tracking frame.

## 8.6 Ownership

Client-to-server RPCs must be invoked through an actor/component the client actually owns.

A replicated world ball Actor may not be owned by the player attempting to send a hit.

Do not incorrectly assume every client can call Server RPCs directly on the ball.

Use an appropriate owning PlayerController, pawn, or replicated player-owned component as a relay.

Preserve security boundaries.

## 8.7 Time synchronisation

Implement a consistent time relationship between clients and server.

Requirements:

- Shared simulation frame reference.
- Server simulation epoch.
- Client/server time offset estimate.
- Fractional frame timestamps where appropriate.
- Handling clock offset adjustments.
- Matching received snapshots to predicted history.
- Recovery after time discontinuities or level travel.

Use Unreal's supported server time facilities where appropriate.

Do not assume local platform clocks are already synchronised.

Do not use raw packet arrival time as the claimed contact time.

---

# 9. CLIENT-SIDE PREDICTION

Implement a functioning prediction and reconciliation pipeline.

## 9.1 Immediate local hits

When a locally controlled hand strikes the ball:

1. Detect the contact locally.
2. Resolve it using the local physics simulator.
3. Apply the result to the client's predicted state.
4. Display the reaction immediately.
5. Record the predicted event.
6. Submit the necessary timestamped information for server validation.

Do not wait for the server before showing local ball reaction.

## 9.2 Input history

Maintain a bounded history of relevant simulation inputs and events.

Index them by simulation frame and timestamp as appropriate.

Keep enough history to replay the period between the last authoritative snapshot and the current predicted frame.

## 9.3 State reconciliation

When a server snapshot arrives:

1. Validate its generation and frame.
2. Find the corresponding historical predicted frame.
3. Compare predicted and authoritative states.
4. Ignore insignificant numerical differences within configurable tolerances.
5. If correction is necessary, restore the authoritative state.
6. Replay the saved inputs and supported collisions.
7. Advance back to the client's current simulation frame.
8. Update visual correction state.

Do not simply overwrite current state with an old server position every time a packet arrives.

## 9.4 Predicted event handling

Predicted impacts must have stable identifiers or an equivalent matching mechanism.

Track:

- Predicted.
- Confirmed.
- Rejected.
- Replaced by authoritative result.

Prevent duplicated audio and haptic events during replay.

Do not let predicted-only events permanently affect authoritative gameplay.

---

# 10. ROLLBACK AND LAG COMPENSATION

Implement historical state storage and bounded rollback.

Suggested classes:

- `FBallHistoryBuffer`
- `FHandPoseHistory`
- `FBallRollbackManager`

## 10.1 Ball history

Maintain a preallocated circular buffer.

Store the necessary state and metadata to restore the ball at historical simulation frames.

Include relevant collision and event data needed for resimulation.

Use generation IDs so snapshots from a previous ball reset cannot corrupt the current simulation.

## 10.2 Hand history

Maintain authoritative server-side pose histories for participating players.

Include enough information to reconstruct hand trajectories over historical simulation intervals.

Do not assume the server's latest hand position represents where the hand was when a delayed collision occurred.

## 10.3 Historical validation

When a delayed potential hit is received:

1. Validate the transmitting player and hand.
2. Validate the claimed timestamp.
3. Reject timestamps outside the permitted lag-compensation window.
4. Retrieve the relevant historical ball and hand data.
5. Reconstruct the hand trajectory.
6. Verify whether a legitimate swept collision occurred.
7. Calculate impact time and response independently.
8. Resolve the hit using the authoritative simulation rules.

The client may submit a collision hint, but that hint is not authoritative proof.

The server must derive the actual contact and impulse.

## 10.4 Server rewind and resimulation

When an accepted historical hit changes the ball's state:

- Restore an appropriate pre-impact state.
- Insert the validated collision into the correct time interval.
- Replay subsequent relevant simulation inputs and collisions.
- Resolve conflicts with other accepted hits.
- Advance to the present server simulation frame.
- Publish the corrected authoritative state.

Do not merely move the present ball to the historical collision point.

## 10.5 Multiple conflicting hits

Create a deterministic policy for hits that occur very close together.

For example:

- Primary ordering by validated time of impact.
- Secondary stable ordering by player/hand identifier.
- Defined tie-breaking rules.
- Revalidation of later contacts after earlier impacts modify the trajectory.

Do not blindly accept every previously reported hit if earlier state changes make a later contact impossible.

## 10.6 Delayed input handling

Explicitly solve the problem that the server may not yet have received the hand samples necessary to reconstruct a historical contact.

Support an appropriate combination of:

- Small interpolation buffer.
- Historical pose updates.
- Collision hints.
- Bounded retrospective collision evaluation.
- Rollback on newly validated information.

The system must define exactly when an input is considered too late.

Avoid making the simulation depend on infinitely delayed data.

## 10.7 Rollback window

Make the history and maximum accepted latency configurable.

Use an initial practical budget such as a few hundred milliseconds, with enough additional storage for jitter and reconciliation.

Do not assume this window guarantees fairness for arbitrary ping.

Track rejected hits and rollback frequency for debugging.

---

# 11. NETWORK SECURITY AND VALIDATION

Treat client tracking data as untrusted.

Implement validation for:

- Impossible hand teleportation.
- Implausible hand velocities.
- Invalid acceleration spikes.
- Invalid or non-finite vectors.
- Invalid rotations.
- Excessive tracking distance from the player's body.
- Invalid timestamps.
- Future-dated collisions.
- Duplicate impact reports.
- Excessive RPC rates.
- Spoofed player identifiers.
- Repeated stale tracking samples.
- Invalid simulation generation IDs.

Validate the ownership of submitted tracking information.

Do not allow a client to report another player's hand movement.

Avoid harsh validation limits that frequently reject real rapid hand movement.

Expose relevant thresholds and provide diagnostics.

The server should calculate accepted collision impulses rather than trusting client-provided impulse values.

---

# 12. VISUAL SMOOTHING AND RENDERING

Separate the physics collision state from the ball's displayed transform.

Suggested component:

`UBallVisualComponent`

The physics position must always come from the simulation.

The visual mesh may be rendered at a corrected or smoothed transform.

Implement:

- Visual position smoothing.
- Visual rotation smoothing.
- Correction offset management.
- Correction decay.
- Configurable snap thresholds.
- Handling for significant state divergence.
- Reset/teleport bypass.
- Impact-aware smoothing.

Avoid introducing unnecessary visual latency for locally predicted hits.

Do not smooth the actual collision state merely to make the mesh look better.

The player's ability to hit the ball must be based on the defined simulation state, not an unrelated interpolated mesh location.

For VR, favour responsiveness over overly aggressive interpolation.

---

# 13. EXTENSIBLE SPIN AND ADVANCED PHYSICS

Implement the necessary infrastructure for advanced ball physics.

Include angular state from the beginning.

Where practical, implement basic:

- Angular integration.
- Angular damping.
- Tangential impulse response.
- Contact-induced spin.
- Rolling resistance.
- Spin transfer during surface collisions.

Design an optional force system to support future effects such as:

- Magnus force.
- Wind.
- Custom ball abilities.
- Alternative gravity.
- Temporary gameplay modifiers.

These optional forces must be applied in an explicitly ordered, replay-compatible manner.

Do not add a complex generic plugin architecture if simple C++ interfaces or composable force providers are sufficient.

All gameplay-affecting physics must use the same authoritative and predicted simulation pipeline.

---

# 14. EVENT SYSTEM

The physics simulation should generate structured collision events.

Suggested:

`FBallCollisionEvent`

Include:

- Stable event ID.
- Simulation frame.
- Fractional impact time.
- Collision category.
- Contact position.
- Contact normal.
- Impact magnitude.
- Incoming and outgoing velocities.
- Player or hand identifier where applicable.
- Collider identifier where applicable.
- Predicted versus authoritative status.

Expose appropriate Blueprint events or delegates for:

- Ball hit by hand.
- Ball hitting court.
- Ball hitting wall.
- Ball entering resting state.
- Ball waking up.
- Authoritative correction.
- Ball reset.

Do not embed four-square scoring rules inside the core physics solver.

The gameplay layer should be able to determine later:

- Which player last hit the ball.
- Which square the ball bounced in.
- Whether the ball bounced twice.
- Whether an out-of-bounds event occurred.
- Whether a hit was legal.

Provide enough information to support these features without requiring major physics changes.

---

# 15. BALL RESET, SPAWNING AND LATE JOINING

Implement Blueprint-callable, server-authoritative functions equivalent to:

```cpp
ResetBall();

SetBallState(...);

LaunchBall(...);

GetBallState();

GetBallLinearVelocity();

GetBallAngularVelocity();

SetBallPhysicsConfig(...);
```

Use appropriately named functions and authority checks.

Requirements:

- Server can reset the ball.
- Clients receive the reset.
- History buffers are invalidated or correctly regenerated.
- Old collision events cannot affect a newly reset ball.
- Late joiners initialise from the current authoritative state.
- Spawned balls initialise correctly.
- Level travel and Actor destruction clean up their resources.
- No invalid access to old pawn/controller references.

Use Blueprint events and parameters where helpful for designers.

Do not expose unrestricted client-authoritative mutation.

---

# 16. PERFORMANCE REQUIREMENTS

Design for future standalone Quest use.

Optimise sensibly without sacrificing correctness.

Requirements:

- Avoid unnecessary per-step heap allocation.
- Prefer reusable arrays and ring buffers.
- Avoid UObject allocation inside the physics loop.
- Avoid repeated expensive Actor lookups.
- Minimise world queries.
- Use simple hand proxies.
- Reuse collision query parameters where possible.
- Avoid unnecessary virtual calls in hot numerical loops.
- Keep simulation state compact.
- Keep network packets bandwidth-conscious.
- Avoid unnecessary replication of unchanged information.
- Avoid unnecessary Blueprint execution in the simulation loop.

Keep critical simulation code on a controlled execution thread initially.

Do not introduce unsafe multithreaded scene queries or concurrent state mutation.

If multithreading is introduced, document the thread ownership and synchronisation requirements.

Provide simple timing instrumentation for the simulator.

Do not add complex SIMD or multithreading solely because it sounds faster.

Correctness, reproducibility and profiling come first.

---

# 17. DEBUGGING TOOLS

Implement useful debug capabilities.

At minimum support toggling:

- Ball collision sphere.
- Current simulation position.
- Previous simulation position.
- Ball velocity vector.
- Angular velocity.
- Intended sweep path.
- Collision contact point.
- Collision normal.
- VR hand collision proxies.
- Hand sweep paths.
- Predicted ball position.
- Server-authoritative ball position.
- Prediction error.
- Simulation frame.
- Rollback occurrence.
- Number of physics steps per rendered frame.
- Simulation step cost.
- Collision iterations per step.

Use appropriate Unreal debug drawing, logging and console variables.

Debug visualisation must be optional.

It should not be required for normal gameplay.

Create a dedicated log category for the system.

Avoid flooding logs every physics step unless explicitly requested.

---

# 18. AUTOMATED TESTING

Create Unreal Automation Tests or equivalent C++ tests for the custom mathematical and state-management systems.

At minimum include tests covering:

## Physics

1. Ball falls under gravity.
2. Physics result is consistent across different rendering frame rates.
3. Ball bounces from a static floor.
4. Ball retains an appropriate amount of energy.
5. Ball eventually settles.
6. Ball wakes when struck.
7. Ball behaves correctly with zero initial velocity.
8. Ball does not generate NaN or infinite values.

## Continuous collision

9. High-speed ball does not tunnel through a floor.
10. High-speed ball does not tunnel through a thin wall.
11. Stationary ball is struck by moving hand.
12. Moving ball is struck by stationary hand.
13. Moving ball and moving hand cross between sampled positions.
14. Two objects moving apart do not generate a false impact.
15. Tangential collision is handled safely.
16. Initial penetration is resolved.
17. Multiple contacts in one timestep are handled.
18. Repeated zero-time contacts cannot cause an infinite loop.

## Simulation history

19. State can be saved and restored.
20. Recorded simulation replay produces equivalent results within tolerances.
21. History buffer wraps correctly.
22. Old simulation generations cannot affect new states.
23. Invalid historical frames are rejected safely.

## Multiplayer logic

24. Snapshot frame comparison works correctly.
25. Stale snapshots are rejected.
26. Prediction correction restores the correct historical frame.
27. Replay advances to the expected present frame.
28. Duplicate input IDs are handled safely.
29. Invalid timestamps are rejected.
30. A delayed validated hit can trigger bounded rollback.
31. Conflicting historical impacts are resolved in a defined order.
32. A rejected prediction does not remain permanently applied.

Where actual multiplayer or XR hardware tests cannot be automated, provide a clearly documented manual test procedure.

Do not describe a test as passing unless it was actually executed successfully.

---

# 19. REQUIRED INTEGRATION WITH EXISTING VR PAWN

This must not be a disconnected example.

Inspect my current VR pawn and integrate the hand system.

Prefer the following approach:

1. Find existing left and right motion controller components.
2. Bind them to the hand collision system.
3. Ensure locally controlled players capture tracking samples.
4. Ensure remote hand data reaches the server through owned RPC pathways.
5. Make the ball simulation aware of active player hands.
6. Register and unregister hands on player spawn/destruction.
7. Support multiple simultaneous players.
8. Avoid duplicate registration.
9. Preserve current locomotion and other VR functionality.
10. Ensure tracking and simulation tick order is defined.

If the pawn is Blueprint-only, provide a reusable C++ component and a minimal, precise Blueprint integration path.

Where technically possible, automatically locate and register compatible motion controllers.

Do not silently assume that a controller component exists under a particular name.

If automatic binding cannot succeed, log a clear actionable message.

Avoid introducing dependencies on Steam session logic into the custom physics classes.

Steam sessions should establish the multiplayer connection; ball physics should use Unreal's network authority and replication facilities.

---

# 20. REQUIRED PLAYABLE TEST ENVIRONMENT

I want to be able to test the ball before implementing the full four-square game.

Create a minimal testing workflow.

Ideally provide:

- A ball Actor that can be placed in a level.
- A floor that the ball can bounce on.
- A small test court or walls.
- A way to reset the ball.
- A way to launch the ball with a chosen velocity.
- VR hand interaction.
- A way to test ball simulation without a VR headset.
- A way to test two networked players.

If creating or modifying `.umap` or Blueprint assets is unavailable from the coding environment, provide native C++ test helpers or actors wherever practical.

Do not claim to have modified Unreal Editor binary assets if you have not.

Provide the minimum exact Editor steps needed for anything that must be connected manually.

---

# 21. NETWORK TEST SCENARIOS

The system must be designed and tested against these scenarios.

### Scenario A — Local player

A single player strikes the ball.

Expected:

- Immediate reaction.
- Correct impact direction.
- No tunnelling.
- Consistent bounce.
- No artificial multiple hits.

### Scenario B — Two players, low ping

Two players share the ball over a multiplayer session.

Expected:

- Both see approximately the same ball state.
- Server owns the authoritative result.
- Client predictions are reconciled.
- Remote hand movement affects the ball.

### Scenario C — Simulated 80 ms RTT

Client strikes the ball with significant network latency.

Expected:

- Local reaction is immediate.
- Server validates historical contact.
- Authoritative state eventually reconciles.
- No uncontrolled position snapping.

### Scenario D — Simulated 150 ms RTT

Repeat the test at higher latency.

Expected:

- System remains functional within its configured support limits.
- Late inputs are handled according to the lag compensation policy.
- Prediction errors are observable through debug tools.

Do not guarantee zero visible corrections at this latency.

### Scenario E — Packet loss

Simulate packet loss and jitter.

Expected:

- No crash.
- No corrupted history.
- Stale pose updates are handled safely.
- Ball eventually converges to authoritative state.

### Scenario F — Simultaneous impacts

Two players hit the ball at nearly the same time.

Expected:

- Server resolves the collision ordering.
- Duplicate or invalidated contacts are handled.
- Clients eventually agree on the outcome.

### Scenario G — Late joining

A player joins while the ball is moving.

Expected:

- Correct current ball state.
- No replay of invalid old events.
- No reliance on the ball having been present since BeginPlay.

### Scenario H — Tracking loss

A controller temporarily loses tracking.

Expected:

- No extreme impact generated by tracking recovery.
- No invalid velocities.
- No permanently broken hand registration.

### Scenario I — Different rendering frame rates

Run simulation at different rendering rates, including lower rates than the physics frequency.

Expected:

- Correct fixed-step execution.
- Equivalent results under the same recorded inputs and collision environment, within numerical tolerances.

---

# 22. IMPLEMENTATION QUALITY REQUIREMENTS

Follow Unreal Engine C++ conventions.

Use:

- Correct `UCLASS`, `USTRUCT`, `UPROPERTY` and `UFUNCTION` declarations.
- Correct generated header placement.
- Correct module API macros.
- Proper header/source separation.
- Appropriate forward declarations.
- Correct `GetLifetimeReplicatedProps` implementation.
- Proper RPC declarations and `_Implementation` definitions.
- Valid object lifetime management.
- Correct Actor/component initialisation.
- Correct tick group and tick prerequisite setup.
- Consistent type and unit usage.
- Safe handling of invalid pointers and destroyed Actors.

Do not invent engine API functions.

Use the APIs supported by the actual project engine version.

When adding include or module dependencies, update `.Build.cs` correctly.

Avoid unnecessary engine-wide settings changes.

Do not restructure unrelated files.

Do not replace existing content without understanding why it exists.

Keep the implementation maintainable and understandable.

Comment complicated algorithms, especially:

- Swept collision maths.
- Time-of-impact calculations.
- Contact normal conventions.
- Fixed-step accumulator behaviour.
- Network time conversion.
- Client prediction reconciliation.
- Historical hit validation.
- Rollback replay.
- Stable collision ordering.

Avoid trivial comments that merely repeat the code.

---

# 23. IMPORTANT ENGINEERING TRADE-OFFS

Before selecting or building the implementation, evaluate the relevant Unreal networking systems for the actual engine version, including built-in Network Physics Resimulation and Network Prediction if available.

However, do not automatically adopt Chaos physics replication because it exists.

Our ball uses a custom sphere simulator, so a custom state-history and prediction implementation may be more appropriate.

Choose an approach that:

- Works with custom physics.
- Provides control over hand collision validation.
- Supports rollback.
- Avoids duplicate simulation authority.
- Does not require Chaos rigid-body movement.
- Fits the existing project.

If using an Unreal framework, explain how it integrates with the custom simulator.

If implementing a custom framework, justify the choice briefly and implement the required functionality.

Do not create two competing prediction systems.

---

# 24. IMPLEMENTATION ORDER

Work through the following stages internally.

Do not stop after completing the early stages unless there is a genuine blocker.

### Stage 1 — Inspect project

Read project configuration and existing relevant code.

Identify integration points and engine version.

### Stage 2 — Core structures

Create:

- Ball state
- Ball configuration
- Simulation input/output structures
- Contact structures
- Simulation clock
- Core simulator

### Stage 3 — Physics and static CCD

Implement gravity, drag, ball movement, swept world collision, bounce response and stable contact handling.

### Stage 4 — VR hand system

Implement hand tracking, timestamped pose history, relative-motion sphere CCD and hand impact response.

### Stage 5 — Networking foundation

Implement authoritative simulation, owning-client hand submission, snapshots, serialisation, frame synchronisation and late joining.

### Stage 6 — Prediction and reconciliation

Implement local predicted hits, history, acknowledgements, correction and replay.

### Stage 7 — Lag compensation

Implement historical validation, bounded server rollback, resimulation and conflict resolution.

### Stage 8 — Presentation and extensibility

Implement visual smoothing, gameplay events, angular behaviour, configuration and Blueprint-facing APIs.

### Stage 9 — Testing

Compile, run available tests, fix implementation issues and document remaining hardware-dependent verification.

The final implementation must work as one integrated system, not as unrelated components.

---

# 25. REQUIRED FILES AND DELIVERABLES

Create all necessary `.h` and `.cpp` files.

Suggested names include:

```text
HandballBall.h
HandballBall.cpp

BallPhysicsTypes.h
BallPhysicsTypes.cpp

BallSimulator.h
BallSimulator.cpp

BallCollisionSolver.h
BallCollisionSolver.cpp

BallPhysicsConfig.h

BallSimulationClock.h
BallSimulationClock.cpp

VRHandCollisionComponent.h
VRHandCollisionComponent.cpp

HandPoseHistory.h
HandPoseHistory.cpp

BallNetworkComponent.h
BallNetworkComponent.cpp

BallNetworkTypes.h
BallNetworkTypes.cpp

BallHistoryBuffer.h
BallHistoryBuffer.cpp

BallRollbackManager.h
BallRollbackManager.cpp

BallVisualComponent.h
BallVisualComponent.cpp

BallSimulationTests.cpp
```

This is a suggested structure, not a mandatory one-to-one file requirement.

Consolidate files if appropriate, provided the system remains modular and maintainable.

In addition:

- Update `.Build.cs` if necessary.
- Update existing pawn/Controller code where necessary.
- Register networked components appropriately.
- Provide Blueprint integration.
- Provide debug/test functionality.
- Provide useful comments and documentation.

Do not generate empty placeholder classes merely to satisfy filenames.

Every implemented class must have a real purpose.

---

# 26. DEFINITION OF DONE

The system is not complete merely because it compiles.

It must satisfy the following:

- [ ] Custom ball simulation is implemented in C++.
- [ ] Ball movement is independent of variable rendering DeltaTime.
- [ ] Ball physics does not depend on Chaos rigid-body simulation.
- [ ] Ball uses swept continuous collision detection against supported geometry.
- [ ] Moving hand versus moving ball CCD works.
- [ ] Hand impacts calculate appropriate collision response.
- [ ] Ball bouncing and resting behaviour works.
- [ ] Simulation state can be saved and restored.
- [ ] Simulation can replay recorded inputs.
- [ ] Server owns authoritative ball state.
- [ ] Existing VR pawn can provide tracking data.
- [ ] Client hand input reaches server through valid network ownership.
- [ ] Local hand strikes are predicted immediately.
- [ ] Authoritative snapshots are received and processed.
- [ ] Client prediction can be reconciled.
- [ ] Bounded historical hand collision validation works.
- [ ] Server rollback and resimulation work for supported cases.
- [ ] Near-simultaneous hits use a defined ordering policy.
- [ ] Ball state replicates to connected clients.
- [ ] Late joining and ball resetting are supported.
- [ ] Visual smoothing does not corrupt collision state.
- [ ] The architecture supports future spin and advanced forces.
- [ ] Useful debug tools exist.
- [ ] Automated tests cover the important algorithms.
- [ ] Source files have been compiled where a usable build environment exists.
- [ ] Integration requirements are documented accurately.

Do not mark an item complete unless it is genuinely implemented.

If a feature is only partially functional, state exactly which part is missing.

---

# 27. BUILD AND VERIFICATION

After implementation:

1. Review all modified files.
2. Check for missing includes.
3. Check Unreal Header Tool compatibility.
4. Check replication declarations.
5. Check server/client ownership.
6. Check memory safety.
7. Check tick ordering.
8. Check physics unit consistency.
9. Run available builds.
10. Run automated tests.
11. Fix errors found.
12. Review major performance issues.
13. Verify there are no unrelated project changes.

If Unreal Engine or a Windows build environment is unavailable, do not falsely claim a successful UE build.

Run any checks available and explicitly identify what requires verification inside Unreal Editor.

---

# 28. FINAL RESPONSE REQUIREMENTS

After making the changes, report:

## A. Implemented architecture

Explain the classes created and what they do.

## B. Modified files

List all created and modified project files.

## C. Physics implementation

Summarise fixed timestep, collision detection, hand impacts and bounce response.

## D. Multiplayer implementation

Explain authority, hand input replication, prediction, reconciliation and lag compensation.

## E. Testing results

Clearly distinguish:

- Passed automated tests.
- Successful compilation.
- Manual validation performed.
- Tests that remain unverified.

## F. Unreal Editor integration

Provide exact instructions for any remaining setup.

Explain:

- How to place the ball.
- How to configure its radius and physics settings.
- How to connect the existing VR hands.
- How to test one player.
- How to test two players.
- How to enable debug visualisations.

## G. Known limitations

List genuinely unsupported cases, numerical approximations and performance limitations.

## H. Recommended next improvements

Explain any important extensions that can be added without redesigning the core system.

---

# 29. FINAL INSTRUCTIONS

This is not an exercise in demonstrating how Unreal physics could work.

**I am asking you to implement the actual foundation of my multiplayer VR game.**

Prioritise, in order:

1. Correct physics and collision behaviour.
2. Reproducible simulation state.
3. Proper multiplayer authority.
4. Immediate local hand interaction.
5. Correct prediction and reconciliation.
6. Lag compensation and fairness.
7. Stable visual presentation.
8. Extensibility and optimisation.

Do not substitute simple transform replication for client prediction.

Do not substitute overlap events for swept hand collision.

Do not call a system deterministic merely because it uses fixed DeltaTime.

Do not call rollback implemented unless historical state restoration and subsequent resimulation actually work.

Do not claim to have solved network latency through interpolation alone.

Do not hide failed tests or missing integration.

When encountering complexity, implement a robust and testable solution rather than silently leaving TODOs in essential paths.

If something cannot be completed in the current environment, implement as much as genuinely possible, provide precise remaining steps, and distinguish missing functionality from working code.

**Start by inspecting my Unreal Engine project, then implement the complete ball system directly in the repository.**
