# World, queries and saved state

Create a world with `kf_world_create`, reserve it with `kf_world_reserve`, and
release it with `kf_world_destroy`. A null allocator selects malloc/free. A custom
allocator supplies both allocation and free callbacks plus its context; returned
memory must have normal C allocation alignment. Reservation is transactional:
if any new buffer fails, old world storage remains valid. Reservation can grow
capacities but never shrinks them. `kf_capacity_t.contacts` bounds both the contact
manifolds and the carried cache, including sleeping pairs (one per shape pair);
zero selects four per shape.
`kf_capacity_t.triggers` bounds the trigger overlaps tracked after a step; zero
selects one per shape.
There is no implicit growth in object, link, query or step operations. No step,
query or index rebuild allocates after reserve.

## Objects and identity

The application assigns nonzero stable object and per-object shape IDs.
`kf_world_add_body` takes a body descriptor plus an array of local shapes. A
body can have zero or several shapes. `kf_world_update_body` replaces its motion
and material/filter descriptor and resets its integration remainders/sleep state;
use `kf_world_set_body` to restore the complete physical state,
`kf_world_modify_body` to change type, mass, material, filter, trigger flag,
gravity scale or rotation locks while keeping the current motion,
`kf_world_set_velocity` to steer a body, and `kf_world_set_kinematic_target` to
move a kinematic body through the next step with the matching velocity. Shape updates
replace an existing shape ID; changing compound topology uses body remove/add.
Triangles are permitted only on static bodies.

Handles pair the application ID with a generation. Removing a body invalidates
its handle, removes all its shapes and deactivates all attached constraints.
Surviving endpoints of those constraints wake even outside the removed body's
contact neighborhood; their islands wake on the next step.
Recreating its ID increments the generation. ID slots retain tombstones, so
capacity counts distinct IDs ever used, including removed objects/links, rather
than just currently active objects. Reusing an ID reuses its tombstone slot.
Generations cannot wrap: exhaustion returns `KF_OVERFLOW`. World storage is
canonical by object ID and then shape ID. Shape order in an add call is immaterial.
Removing a body discards its carried contact impulses; a body with the reused ID
starts a new contact. Body and shape edits keep the last step's contact reports
and trigger events. Adding or removing bodies, reserving and restoring clear the
contact reports until the next successful step; restoring also clears trigger
events.

The same input command sequence is deterministic. Independent character commands
must be scheduled in a consistent order by the game; insertion-order independence
does not mean different sequences of dependent motion commands are equivalent.
Adding, removing, moving or reshaping a body wakes sleeping dynamic bodies
within contact range of its old and new bounds; the next step wakes the rest of
their islands.

## Index and filters

The world keeps a balanced binary bounding-volume tree over shapes, including
individual triangles of static geometry. Rebuild sorts bounds by X and stable
shape order using an allocation-free heap sort. World bounds of every shape are
cached and recomputed only for bodies that moved or changed. Updates refit
existing tree bounds when possible; structural mutations and steps mark it for
lazy rebuilding by the next query. Restoring
a snapshot explicitly rebuilds the index. Static meshes use multiple triangle
shapes in this same BVH; there is no separate mesh-file reader or content cooker.

Each homogeneous subtree records its owner, so collision and trigger traversal
can skip a body's own subtree without visiting all its leaves. Structural
rebuilds update ownership; pose refits preserve it. This prevents quadratic
self-pair traversal for large compounds and costs about 16 bytes per reserved
shape on a 64-bit ABI, without changing snapshots or allocating during a step.

Both filter directions must allow a pair: `(query.category & body.mask) != 0`
and `(body.category & query.mask) != 0`. The default filter permits all categories
and excludes triggers. Explicit exclusions match both ID and generation; a stale
excluded handle does not suppress a newly created object. Triggers participate
in queries only when requested, and never create a dynamic or character obstacle.
After each step they report overlaps that began or ended through
`kf_world_trigger_events` (see [dynamics.md](dynamics.md)).
A null filter selects the default. Batches store a filter value in each request.

## Query outputs

`kf_world_sweep_first` / `kf_world_raycast_first` return the closest result.
The all-hit variants return one first hit per shape. Hits sort by exact Q32
`fraction_q32`, object ID and shape ID; the Q16 `fraction` is a convenience value. Contact/overlap results sort by object and shape ID.
A region query returns unique objects with at least one shape AABB intersecting
the region, sorted by ID; this is a broad-phase region test. It does not claim
exact geometry intersection. Contact is the exact-shape overlap API, including
tolerance-based touching. Results include current handles.

Caller-provided capacity is measured in result records. Every all-result API
requires a `required` pointer. A zero-capacity/null-buffer query is a sizing
query. A short buffer returns `KF_CAPACITY_EXCEEDED`, reports the full count and
writes the deterministic sorted prefix that fits. Nothing is silently dropped.
`KF_NO_HIT` reports an empty result, independently of input/numeric errors.
On other errors, discard output storage. First-hit queries intentionally need
only one result slot even if more shapes intersect the path.

`kf_world_query_batch` executes every request and returns each request's status
and required count. Its own `KF_OK` means the batch was dispatched; callers must
inspect individual statuses. Output element types follow the request type:
sweep/ray -> `kf_sweep_hit_t`, contacts -> `kf_contact_t`, region -> `kf_handle_t`.
Calls on one world require external serialization; queries can rebuild the index.
No function holds global mutable world state.

Depenetration repeatedly chooses the deepest contact, using stable IDs on ties.
The iteration count and maximum total correction bound its work; unresolved
penetration returns `KF_ITERATION_LIMIT` with its partial corrected pose and
reported correction. Triggers should remain excluded for physical depenetration.

## Continuous queries and continuation

`kf_shape_sweep_relative` accepts two complete displacements over the same
interval. Returned moving position and obstacle contact point are transformed
back into actual world motion. `kf_world_sweep_moving` uses each target's current
linear velocity, rational step and position remainder to predict its motion.
This is a constant-velocity prediction: gravity, future impulses and angular
swept volumes are not included in this query. The dynamic step performs its own
continuous collision, including rotation.

An all-hit query gives the game the ordered obstacle list for a segment. The
caller chooses whether to penetrate, bounce, stop or resweep. To continue from a
hit, use its returned pose/contact, the unconsumed time/distance, and exclusions
or a small separating offset as appropriate. The library never applies damage
or chooses penetration policy.

`kf_world_sweep_curve` approximates a quadratic Bezier reference path from the
pose position through `control` to `end`, with orientation fixed. A deterministic
power-of-two subdivision bounds reference-path chord error by the requested
tolerance (including fixed-point rounding). If the segment limit cannot meet it,
the call returns `KF_ITERATION_LIMIT` before casting. The function casts the
piecewise linear path, without inflating the shape: hits within that error of
the analytic curve can differ. Inflate a spherical query radius by the tolerance
when a conservative application policy is needed. Hits are ordered by global
Bezier parameter, then IDs; distance accumulates piecewise-linear path length.
Repeated crossings or initial penetrations may return the same shape in multiple
segments. Multiple hits are capacity checked across the entire path.

## Numbers and snapshots

Checked world coordinates/bounds are within +/-4096 units, each linear speed or
gravity component within +/-1024, and dynamic mass within `[1/512, 512]` units.
Inverse mass is derived from mass. Each angular velocity component has the same
numeric +/-1024 limit in radians/second. Rotations are unit quaternions in Q2.30;
`kf_rotation_valid` checks that the stored vector part has at most unit length.
During a step these limits are enforced per body: speeds are clamped and a body
reaching the coordinate range stops there, so the world keeps advancing.
Containment checks the stored body origin and each axis of every rotated shape,
including compound shapes with offset centers. Motion along the boundary is
preserved, and a valid resting body stays in place. If a wide compound cannot fit
at any translation after rotating, it keeps its previous admissible orientation
and stops rotating.
Kinematic targets must keep every shape within the coordinate range and require
no more than the allowed center-of-mass linear and angular speed over the tick.
The dedicated target setter, complete-state setter and snapshot restore apply
the same checks. A shape edit with a pending target also validates its new center
of mass; an invalid edit leaves the original shape and target in place.
Friction/restitution lie in `[0,1]`, damping rates in `[0,1024]` per second.
Geometry has additional size limits in [geometry.md](geometry.md).

A rational step requires `0 < numerator <= denominator <= 1000000`, and 1 to 16
substeps divide it. Integration divides signed integers toward zero and retains
the signed remainder. Body remainders are measured in units of
`1 / (denominator * substeps)` and stay below that divisor. `kf_integrate_scalar`
updates its remainder only on success. Scalar checked add/multiply return
`KF_OVERFLOW` without modifying the destination. Other boundary checks return
`KF_OUT_OF_RANGE`, not no-hit.

`kf_world_save` writes a caller-owned snapshot descriptor and arrays. A sizing
call reports object, shape, constraint and contact counts, including tombstones,
even when array capacities are insufficient. The snapshot includes world
config/tick, body descriptors/physical states and kinematic targets, local
shapes, constraint topology/state, generations, all rational remainders, and the
contact impulses carried into the next step together with whether each pair
touched, and the current trigger overlaps. It excludes only derivable
index/scratch storage. Arrays are canonical, and restore validates the complete
snapshot before
mutating a world. Restore requires already reserved capacities and rebuilds the
BVH. Object and shape ownership is checked in one pass over their sorted arrays;
constraint handles use binary lookup. Invalid snapshots leave physical state
unchanged. Carried contacts must reference existing shape pairs, have nonnegative
normal impulses and in-range anchors. Tangent impulses cannot equal `INT64_MIN`,
whose negation is not representable. Extreme accepted warm starts are guarded
by the solver before normal simulation continues.

Snapshots are in-memory C values, not an ABI-independent binary format. Do not
hash or serialize struct padding. File/network encoding, version migration and
command replay are outside the library. Restoring a timeline also restores its
identities; the caller must synchronize external gameplay references with that
timeline. Character states are caller-owned and saved/restored separately as
complete `kf_character_state_t` values.
