# Character controller

`kf_character_move` in `kinefix/kinefix.h` moves a vertical capsule through a
`kf_world_t`. Its position is the bottom of the capsule. Horizontal velocity is
an input in world units per second; the controller integrates vertical gravity
and clips velocity at blocking surfaces. It uses the world's rational time step
(default `1/25` second), with signed division toward zero and persisted
`position_remainder` and `gravity_remainder` fields. Save the entire
`kf_character_state_t` to resume the same simulation.

The caller supplies a persistent character state, settings, query filter, and
contact buffer. Initialize a new state to zero. `kf_character_settings_default`
allows a 45-degree maximum slope, a one-third-unit step, and a one-fifth-unit
downward snap. Settings can disable steps or snap by setting their distance to
zero; a small contact probe still recognizes a touching floor. The skin controls
the contact probe distance. No floor or arena is implicit. Holes are represented
by the absence of supporting shapes. `constrain_to_region` optionally clips the
requested destination to a region containing the entire capsule; it does not
create support or a synthetic contact.

`ledge_drop` guards pits. When it is nonzero, a grounded move that does not jump
casts a ray from the bottom sphere's center at the new position. If no walkable
surface lies within `ledge_drop` below, the move ends at its start position as
if blocked by a wall, horizontal velocity becomes zero, and
`result.held_at_ledge` is set. Drops within `ledge_drop`, such as terraces, are
walked off normally. Jumps, knockback with upward velocity and airborne
characters are not held. The ray is cast at the center, so a character can
stand with half its body over the rim.

Movement first validates and carries an existing support, resolves bounded
initial penetration, then sweeps the complete displacement vector. Each impact
clips the remaining displacement against the accumulated planes. Intersecting
planes constrain movement to their common direction; blocking capsules use this
same procedure, so a collision does not discard the whole requested move.
If collision projection would exceed the velocity component limit, all velocity
components are scaled together, preserving the clipped direction and keeping
the resulting state valid for the next move.
`slide_iterations` and `depenetration_iterations` must be between 1 and 32.
Unresolved movement or penetration reports `KF_ITERATION_LIMIT`; penetration
also respects `maximum_depenetration`.

Walkable surfaces have `normal.y >= minimum_ground_normal_y`. Steeper upward
normals block horizontal movement without converting it into upward movement.
A character supported by walkable ground does not accelerate into it: gravity
starts only after the support is lost, so standing on a walkable slope does not
slide downhill. Walking off an edge therefore keeps the height for the move in
which the support disappears.
An initially grounded character blocked by a side face can try a step: the
controller checks the upward path, traverses with the raised capsule, then
requires a walkable surface on the downward path. It accepts a step only if it
makes more progress in the requested horizontal direction. Both the upward
and raised horizontal sweeps check headroom. The obstacle surface at the
landing contact must be within `step_height` of the starting feet, even when
the capsule first touches with its rounded rim. Ground probes classify the
adjacent obstacle surface normal at a convex lip, so a flat stair top remains
walkable at small time steps. Downward snapping is limited to
`snap_distance` and is available while following ground; an airborne character
uses only the contact probe, preserving falls and jumps.
Internal vertical support, snap, step and ledge probes stop at the coordinate
boundary. A touching floor can still support the capsule at the lower boundary;
the boundary itself creates no support. A step uses only the available upward
clearance. Actual requested movement outside the coordinate range still returns
`KF_OUT_OF_RANGE` and leaves character state unchanged.

The final downward probe determines current support. State records its
generation-checked body handle, shape id, normal, linear plus angular surface
velocity around the support's center of mass, pose, and a fieldwise shape
signature. Static supports report zero surface velocity even if their descriptor
stores nonzero velocities. On the next call, the saved support is checked before
carrying the character through the platform's translation and full rotation.
An invalid saved support rotation returns `KF_INVALID_ARGUMENT` before movement
and leaves the character state unchanged.
Carry movement sweeps against other colliders. Removed,
filtered, nonwalkable, or changed supporting geometry invalidates that support;
the ordinary final probe may acquire any valid surface under the new position.
Character state does not automatically inherit platform velocity on a jump;
game logic can add the reported `support_velocity` when it starts the jump.

The controller never allocates. Add other characters as kinematic capsule
colliders in the world, and exclude the moving character's own body using the
filter. Triggers never obstruct a character, even when the supplied filter
includes triggers for other query types. After each move, pass the new position
to `kf_world_set_kinematic_target` for the character's collider: the next world
step moves it there with the matching velocity, so dynamic bodies are pushed and
dragged consistently. The order of character commands is part of the input
sequence.
The controller does not implement a simultaneous crowd or reciprocal-avoidance
solver.

`result.displacement` includes support carry, depenetration, sliding, stepping,
and snap. `landed`, `stepped`, `hit_ceiling` and `held_at_ledge` report the
performed movement. `landing_speed` is the downward speed with which the move
landed, or zero, and is the input for fall damage or landing stuns.
The contact buffer contains initial contacts and movement contact events in
their deterministic processing order; a surface may appear more than once.
Initial contacts include penetration depth and both surface points. Sweep
events have zero depth and the coincident contact point at impact.
`result.contact_count` is the full required number of records. Insufficient
capacity returns `KF_CAPACITY_EXCEEDED`, writes the available prefix, and leaves
character state unchanged. All errors leave character state unchanged; contact
and result storage should be discarded on errors other than capacity overflow.
Queries may rebuild the world's reserved index, but do not change body state.
