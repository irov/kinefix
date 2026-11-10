# Bodies, constraints, and ticks

Include `kinefix/kinefix.h` from C or `kinefix/kinefix.hpp` from C++ for the
complete physics API.

A world defaults to a rational `1/25` second tick. `kf_world_step` advances the
integer world tick only when the entire step succeeds. Static bodies remain
fixed even if their stored velocity is nonzero; that velocity is ignored by
contact and constraint response. Kinematic bodies follow their specified
velocity or target pose without receiving impulses; dynamic bodies receive
gravity and impulses according to their mass and inertia. `gravity_scale`
(default 1, range [-16, 16]) multiplies the world gravity per body, so floating,
heavy and rising bodies share one world. A trigger may move but never
participates in contact response. Both category/mask checks must allow a
physical pair.

```c
kf_world_config_t config = kf_world_config_default();
kf_world_t * world = NULL;
kf_status_t status = kf_world_create( &config, NULL, &world );
if( status != KF_OK )
{
    return status;
}

status = kf_world_reserve( world, (kf_capacity_t){128, 256, 128, 0, 0} );
if( status != KF_OK )
{
    kf_world_destroy( world );
    return status;
}

kf_body_desc_t body = kf_body_desc_default( 101 );
body.type = KF_BODY_DYNAMIC;
body.pose.position.y = 2 * KF_FIXED_SCALE;
body.pose.rotation = kf_rotation_from_yaw( KF_ANGLE16_QUARTER / 2 );
body.mass = 3 * KF_FIXED_SCALE;
body.restitution = KF_FIXED_SCALE / 4;
kf_shape_desc_t shape = {0};
shape.id = 1;
shape.shape.type = KF_SHAPE_SPHERE;
shape.shape.data.sphere.radius = KF_FIXED_SCALE / 4;
kf_handle_t handle;
status = kf_world_add_body( world, &body, &shape, 1, &handle );
if( status == KF_OK )
{
    status = kf_world_step( world );
}
kf_world_destroy( world );
return status;
```

Reserve includes solver and contact storage. Stepping, impulses,
constraint operations, and index rebuilding do not allocate. Adding a previously
unseen constraint id consumes one reserved constraint slot. Removing it retains a
tombstone, so its former handle cannot refer to a later constraint. Recreating
the same application id increments its generation. Removing a body also removes
all constraints referencing it.

`kf_world_modify_body` changes a live body's type, mass, material, damping,
gravity scale, rotation locks, filter and trigger flag in place. The pose,
velocities and remainders are kept, except that a body made static stops; the
body and its neighborhood wake. Use it to switch a prop between static and
dynamic, to toggle a trigger, or to change collision masks without re-adding
shapes. `kf_world_set_velocity` replaces the velocities of a dynamic or kinematic
body without moving it and wakes it, which suits AI steering that sets a
velocity every tick.

## Rotation and mass

Poses carry a full 3D rotation: `kf_rotation_t` stores the vector part of a unit
quaternion in Q2.30 and implies a nonnegative scalar part, so a zero rotation is
the identity. Helpers build rotations from a yaw or an axis and angle, compose,
invert, and apply them. Angular velocity is a world-axis vector in radians per
second. Rotations integrate with Q2.30 precision, so very slow rotation is not
lost to quantization.

An axis-angle axis may be any nonzero vector, including one raw unit per
component: its magnitude does not change the angle. Direction is normalized
at Q30 precision independently of the half-angle sine, as in
[Bullet's axis-angle construction](https://github.com/bulletphysics/bullet3/blob/master/src/LinearMath/btQuaternion.h).
A zero axis or zero angle produces identity.

Mass properties are derived from the body mass and its shapes, and cached until
the body or one of its shapes changes. Shapes share the mass by volume;
triangles carry none. Each body rotates around its
center of mass, and its linear velocity is the velocity of that center. The
inertia tensor combines each shape's analytic tensor with the parallel axis
theorem. A small inertia floor of `(1/64 unit)^2` per unit mass keeps thin or
degenerate shapes invertible. Kinematic and static bodies have infinite mass.

Compound mass uses complete volume-weighted first and second moments, then one
division by the total volume for each component. The center is rounded once,
and translated shape tensors are accumulated about that center in Q8.24.
Unsigned 128-bit magnitudes with separate signs retain every volume share;
there is no per-shape normalized weight or per-shape centroid division. This
also supports the volume sum of every shape count expressible by unique 32-bit
shape ids, without an extra compound-size restriction. As in
[Jolt's compound mass calculation](https://github.com/jrouwe/JoltPhysics/blob/master/Jolt/Physics/Collision/Shape/CompoundShape.cpp#L61),
child mass and translated inertia are accumulated before the final mass scale.
Primitive tensor and Q8.24 quantization still apply when geometry is split.

`rotation_locks` combines `KF_LOCK_ROTATION_X`, `_Y` and `_Z`. A locked world
axis gets no angular response from contacts, links or impulses, and its angular
velocity stays zero. Locking X and Z keeps an actor or prop upright while it can
still turn around Y; locking all three gives a body that only translates.

## Integration and rounding

A tick runs `substeps` (default 8) equal substeps. Velocity and position
integrate using integer numerator/denominator arithmetic per substep. Every
division truncates toward zero; its signed remainder is retained per axis in
units of `1 / (step.denominator * substeps)` raw units. Gravity uses
semi-implicit Euler: update velocity first, then advance position using that
velocity. Damping is an implicit rate per second applied every substep:

`velocity /= 1 + damping * substep_seconds`

Restitution and friction are unit-interval coefficients. A pair uses its greater
restitution and smaller friction coefficient. Approaches slower than
`restitution_threshold` do not bounce, so resting bodies with restitution settle
and can sleep. `kf_world_apply_impulse` applies an impulse at the center of mass;
`kf_world_apply_impulse_at` applies it at a world point and also changes angular
velocity. Both wake the body and reject a result beyond the speed limit; applying
an impulse to a static or kinematic body is an argument error.

`kf_world_set_kinematic_target` makes a kinematic body reach a target pose
exactly at the end of the next step. The step derives the linear and angular
velocity of that motion, so dynamic bodies are pushed with the correct speed and
friction. A target whose center-of-mass displacement or shortest rotational
motion would require a component above `KF_SPEED_LIMIT` returns
`KF_OUT_OF_RANGE` without changing the body. State updates and snapshot restores
apply the same check. Afterwards the body keeps that velocity until it receives
another target or velocity. Use targets, not `kf_world_set_body`, to move character
colliders and animated platforms.

The state API and snapshots retain the tick, all integration remainders,
sleeping state, kinematic targets, body velocities, rotation, constraint
impulses, and the contact impulses carried into the next step. The index is
rebuilt from restored authoritative shapes and poses.

## Contacts and solver

Each step builds contact manifolds once, for shape pairs whose swept bounds are
within the speculative distance (four `contact_slop`; a kinematic partner adds
its known motion over the tick). A manifold has up to four points from face
clipping, two for a capsule lying along a face or another capsule, and one
otherwise. Pairs are visited in canonical shape order, so insertion order and
index traversal cannot change results.

The solver is a substepped soft-contact sequential impulse solver. Each substep
integrates velocities, applies the accumulated impulses, runs
`solver_iterations` biased passes over distance links and contacts, integrates
positions and rotations, and runs one relaxing pass without bias. Accumulated
normal impulses are clamped to be nonnegative; friction impulses stay inside the
Coulomb cone of the normal impulse along two tangents. Contacts with static
bodies are solved last in every pass and are twice as stiff, so pushes from other
bodies, including kinematic ones, do not drive bodies through static geometry.
Penetration recovers softly (`contact_hertz`, `contact_damping_ratio`) at no more
than `contact_push_speed`. Gaps up to the speculative distance let bodies close
exactly the remaining distance. After the substeps, restitution is applied from
the approach speed measured before the step.

Converged contact impulses carry into the next step, matched per point by body
frame anchor. They are part of world state and of snapshots, so a restored world
continues identically. Snapshot validation rejects negative normal impulses,
`INT64_MIN` tangential components, and out-of-range anchors. Saturating impulse
accumulation, and torque and inertia products bounded so that their sums cannot
overflow, protect the solver from large accepted cache values without slowing
ordinary steps; internal linear and angular velocity components have headroom up
to 1024 times the public speed limit before the ordinary physical clamp. Physical limits are enforced per body without rejecting
the step: velocities are clamped to the speed limits, and a body reaching the
coordinate range stops at it. Only invalid input, invariant failures, and an
exceeded contact capacity reject a step; they are detected before any body,
constraint or contact state changes, and time does not advance.
The contact capacity includes both active manifolds and retained contacts of
sleeping bodies. Their union is checked before commit, so an overflow preserves
all cached impulses, body states, constraints and the tick. `KF_CAPACITY_EXCEEDED`
can be resolved by reserving more contacts and stepping again.
Retained pairs are rechecked against body types, triggers and filters; a pair
that can no longer receive physical response releases its cache slot.

`kf_world_contact_reports` lists the shape pairs that touched in the last step,
in canonical pair order: a pair touches when one of its contact points ended
within `contact_slop` or carried a normal impulse. Each report has both bodies
and shape ids, the normal from the right shape toward the left one, the contact
centroid at the step start, the closing speed before the step, and `began` when
the pair did not touch in the previous step. Whether a pair touched is carried
with its contact impulses, so `began` is reproduced after a restore. Pairs whose
bodies all sleep are not reported, and waking them does not begin them again.
Reports describe the last step: body, shape and velocity edits keep them, while
adding or removing bodies, reserving and restoring clear them until the next
step.
Contacts that occur only inside continuous collision within one tick are not
reported; use queries for projectiles.

## Trigger events

After every step, each trigger shape whose body has a nonzero mask collects the
shapes it touches (with the tolerance of `kf_shape_contact`) at the new poses.
Both filter directions must accept the pair; other triggers are ignored, and so
are pairs of two static bodies. `kf_world_trigger_events` returns the overlaps
that began or ended in that step in canonical order, each with both handles and
shape ids. A visitor removed between steps ends its overlap in the next step
with its last handle. A body that only needs to stop colliding, such as a
climbing actor or a thrown prop, can use the trigger flag with a zero mask: it
then creates no overlaps. Events stay available until the next step, also
across body edits and structural changes; only a restore clears them.

The current overlaps are world state and part of snapshots, so a restored world
reports the same events. `kf_capacity_t.triggers` bounds them (zero selects one
per shape). A step that finds more keeps the previous overlaps unchanged and
`kf_world_trigger_events` reports `KF_CAPACITY_EXCEEDED` with the overlap count
to reserve; after reserving, the next step reports the missed changes. Detection
rebuilds the query index once per step when an active trigger exists, which
the next query would otherwise do.

## Continuous collision

After the solver, bodies that moved farther than half their thinnest shape
(including rotation) are swept from their start pose along their path. The
sweep is conservative advancement with a bound on rotational point speed. The
center path retains the solver's substep positions, including the effects of
gravity, implicit damping, speed limits and contact impulses. Pairs already within contact
range at the start of a path belong to the solver. This is tracked by the exact
pair of leaf shape ids: a contact on one shape of a compound body does not
exempt its other shapes from CCD against the same obstacle. Shape-level
continuous queries also underlie
[Box2D's continuous solver](https://github.com/erincatto/box2d/blob/main/src/solver.c).
Impacts are processed in time
order, with ties broken by body and shape order. An impact with a static or
kinematic obstacle, or between two fast bodies, resolves restitution and
friction at the impact pose and continues the remaining motion, so a projectile
can bounce several times within one tick. Impacts sample the actual substep
velocity; the remaining part of that substep uses the impact velocity, and the
remaining solver boundaries apply forces and damping once each. Every nearby
static or kinematic contact continues to support the body during this replay,
including contacts from the original solver and earlier impacts. Support pairs
are solved in canonical shape order; restitution applies only to fresh impacts.
A fast body reaching a slow dynamic
body stops at the impact pose; the next step's contact then exchanges momentum
with both masses. `ccd_iterations` limits the impacts per fast body and tick;
when it is exhausted the body stops at its next impact. If either of two fast
dynamic bodies has exhausted its budget, both paths stop at their shared impact
and the next tick's contact solve exchanges momentum. The step never fails
because of it. Angular motion uses the actual rotation vectors of the solver
substeps, including reversals and complete revolutions; the final orientation
alone cannot describe that path. Returning center paths also qualify for a
sweep when their intermediate excursion exceeds the movement threshold.
Each impact rechecks trajectories
whose swept bounds meet a changed path, including those with no previous event.
Stopped paths remain collision obstacles for the rest of the tick.
When prescribed kinematic travel exceeds the bounded speculative margin,
dynamic candidates in its swept bounds also use continuous collision, including
stationary and sleeping bodies. A kinematic target still reaches its prescribed
endpoint exactly.

## Distance constraints

`KF_CONSTRAINT_DISTANCE` enforces a fixed anchor separation;
`KF_CONSTRAINT_MAX_DISTANCE` permits a slack rope and blocks extension beyond its
length. A nonzero `hertz` turns either link into a spring with that natural
frequency and `damping_ratio`: a distance spring pulls and pushes toward its
length, and a rope spring only pulls once stretched. Spring softness is part of
the behavior, so it acts in the relaxing pass too. Anchors are body-local points transformed by each body's rotation, so
off-center anchors produce torque. Length and each anchor coordinate are limited
to 512 world units. A zero-length distance link uses +X as its deterministic
separation axis when both anchors coincide.

Links are soft velocity constraints solved before contacts in every pass and
warm started from their stored impulse. Rigid links hold errors within
`contact_slop` rigidly; larger errors recover softly at twice the contact
stiffness. Static and
kinematic anchors have infinite mass. Use `kf_world_get_constraint` /
`kf_world_set_constraint` to read or restore a link state, and explicit
remove/add operations to change topology.

## Sleep

Dynamic bodies within contact range of each other, or joined by a link, form an
island. A body counts as slow when its speed and its angular speed times its
extent stay below `sleep_speed`; an island sleeps after all its bodies stayed
slow for `sleep_ticks` consecutive ticks. An awake body, an impulse, a constraint
edit, or a moving kinematic body within contact range wakes the whole island.
Adding, removing, or moving bodies and shapes wakes sleeping bodies near the
affected bounds. Set `sleep_ticks` to zero to disable automatic sleeping.

## Scope

The solver supplies motion and contact mechanics with semi-implicit Euler
integration, so a perfectly elastic body under gravity keeps its bounce height
only to within one substep: an impact at fraction f of a substep shifts the next
apex by (1 - 2f) times the substep travel at the impact speed, alternately above
and below the drop height without accumulating (about 0.04 units for an 8 units/s
impact at the default 200 substeps per second; more substeps reduce it). Damage, detonation,
penetration policy, prop destruction, chain release rules, AI, and visual ragdoll
parts remain outside Kinefix. Joints other than distance links and springs,
network formats and storage protocols are not provided.
