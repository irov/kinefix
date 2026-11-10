# Dante prototype on Kinefix

The Dante web prototype runs 2D matter-js for the ground plane and integrates
height by hand, turning pairs at different heights into sensors. Kinefix
simulates the same scene in 3D, so height-aware collision, standing on props and
falling into pits come from the geometry itself. This page maps each prototype
mechanic to the library and lists what remains game code.

## Units and configuration

| Prototype | Kinefix |
|---|---|
| x, y ground plane; z height | x, z ground plane; y up |
| pixel-like unit `u` | `0.25 * u` world units |
| 60 Hz step | `config.step = {1, 60}` |
| gravity 1500-1800 u/s^2 | `gravity.y = -450` (1800 u/s^2), per-body `gravity_scale` for the rest |

At scale 0.25 the 9216 x 5184 world centered at the origin spans +/-1152 by
+/-648 units, heights from the pit floor (-300) to the highest arcs (about +400)
stay inside +/-4096, and the fastest measured body (2403 u/s, a corpse part)
moves at 601 units/s, below the 1024 limit. Prototype masses of 0.7-35 fit the
[1/512, 512] range unchanged.

Use one world gravity and express the other entities as a ratio: props and
fragments 1700/1800, corpse parts 1600/1800, grenades 1500/1800. matter-js
`frictionAir` f per 60 Hz step becomes `linear_damping = -60 * ln(1 - f)` per
second: 0.13 gives 8.4, 0.08 gives 5.0, 0.045 gives 2.8. Restitution values
carry over; `restitution_threshold` replaces the prototype's per-entity bounce
stop speeds.

The level benchmark uses 2000 static boxes, 300 dynamic props, 80 steered actors
and 400 rays per tick. Run it with the intended tick rate and substep count to
measure the target platform; the README includes both 25 Hz and 60 Hz commands.

## Mechanics

| Mechanic | Prototype | Kinefix |
|---|---|---|
| Player | Hand-written z/vz, matter circle in the plane | `kf_character_move` capsule with `ledge_drop` for the pit guard; its kinematic collider follows with `kf_world_set_kinematic_target` |
| Enemies | Dynamic circles steered by `setVelocity`, infinite inertia | Dynamic capsules with `rotation_locks = X | Y | Z`, steered every tick by `kf_world_set_velocity` (keep the current `velocity.y`) |
| Props | Chamfered rectangles and barrels, yaw only, static or movable | Box or prism bodies (chamfers as an 8-vertex prism, barrels as a 12-16 vertex prism) with `rotation_locks = X | Z`, or full rotation for tumbling debris |
| Static/dynamic switch, sensor toggles | `setStatic`, `isSensor` | `kf_world_modify_body` with a new type, `trigger` flag or mask; pose and motion are kept |
| Height-aware pairs, one-way prop lids | Pairs become sensors when height intervals do not overlap | Real 3D contact; the controller stands on props through its support |
| Collision categories | 8 categories, asymmetric masks | Same `category`/`mask` on bodies and query filters |
| Terrain | Analytic heightfield: terraces, ramps, basement, pits, berms | Static boxes for terraces, rotated boxes or prisms for ramps and berms, triangles for curved pit rims; pits are the absence of floor |
| Floor height | `floorHeight(x, y)` | `kf_world_raycast_first` straight down |
| Walking off ledges, pit guard | Center sampling, 18 u step, 34 u pit padding | `step_height`, `snap_distance`, `ledge_drop` (walk-offs within it are allowed, deeper drops hold the player) |
| Fall damage, landing stun | Peak tracking | `result.landing_speed` for the player; `approach_speed` of contact reports for bodies |
| Dangerous props touching the player | Scan of active pairs | `kf_world_contact_reports` pairs between a prop and the player collider; `began` for one-shot effects |
| Bullets, spit, rockets | Parametric lines and arcs, terrain sampling | `kf_world_sweep_curve` or `kf_world_sweep_all` with exclusions; an arc `lerp(a, b, t) + 4 h t (1 - t)` is the quadratic Bezier with control `(a + b) / 2 + (0, 2 h, 0)`, and the downward variant `lerp(a, b, t^2)` uses control `a + (0, 2 h, 0)` |
| Line of sight, fog, abyss spirits | `clearRay`, sampled lines | `kf_world_raycast_first` with a mask, `kf_world_sweep_first` for swept spheres |
| Explosions, knockback, blast lift | Radius checks, `applyForce`, hand vz | `kf_world_contacts` with a sphere or `kf_world_query_region`, game-side falloff, `kf_world_apply_impulse_at` with a vertical component |
| Leaps, climbs, throws, pickups, emergence | `setPosition` along lerp plus parabola as a sensor | Kinematic body moved by `kf_world_set_kinematic_target` every tick while its `trigger` flag is set and its mask is zero |
| Zones: objective areas, hazards, pickup radius | Distance checks in game code | Trigger bodies with a mask for the visitors of interest; `kf_world_trigger_events` reports entering and leaving |
| Chains | 6-node constraint chains, stiffness and damping | Distance links between small spheres; set `hertz` and `damping_ratio` for elastic chains, remove the constraint to release |
| Corpses, ragdolls | Rectangles joined by constraints | Dynamic boxes joined by zero-length distance links at the joints; categories keep limbs out of actors |
| Fragments, pit debris | Small rectangles, removed below a height | Small dynamic boxes; remove bodies below the kill height in game code |

## Not provided

These remain game code or later library work:

- Contacts that exist only inside continuous collision within one tick are not
  in contact reports. Fast projectiles should stay queries, as in the prototype.
- A heightfield shape. Terrain is built from boxes, prisms and triangles; a
  dedicated heightfield would reduce shape count for large curved terrain.
- A per-pair contact callback. Categories, masks, triggers and 3D geometry
  cover the prototype's uses.
- Navigation, damage, stuns, chain rules and removal policies.
