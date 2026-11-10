# Checked geometry

Include `kinefix/kinefix.h` for checked geometry and math. All calculations in
the narrow phase use integer fixed point; floating point conversions are not
used during queries.

## Shape matrix

Every cell below supports `kf_shape_contact`, `kf_shape_manifold`, translational
`kf_shape_sweep`, and bounds. `kf_shape_raycast` supports every column. Contact
is also the overlap
query: `KF_OK` means contact, `KF_NO_HIT` means separation. A triangle is a double
sided surface with zero thickness; triangle bodies in a world must be static.

| Moving / left | AABB | Sphere | Capsule | Box | Convex prism | Triangle |
|---|---|---|---|---|---|---|
| AABB | yes | yes | yes | yes | yes | yes |
| Sphere | yes | yes | yes | yes | yes | yes |
| Capsule | yes | yes | yes | yes | yes | yes |
| Box | yes | yes | yes | yes | yes | yes |
| Convex prism | yes | yes | yes | yes | yes | yes |
| Triangle (geometry query only) | yes | yes | yes | yes | yes | yes |

The body pose translates and rotates local geometry by a full 3D rotation. A
local AABB therefore becomes an oriented box when its body is rotated. A box's
own yaw is applied in the body frame before the pose rotation. Capsules are
vertical in their body frame and follow its rotation. Prisms accept either
consistent winding; each edge must place every other vertex strictly on the
same side. Concave, self intersecting, duplicate, and collinear vertex lists are
rejected. Convex decomposition is a content preparation operation.

Box geometry retains one Q30 rigid frame for its local yaw and body rotation.
Forward vertices, inverse closest-point queries, surface witnesses, and local
yaw inertia use that rotation consistently. Q16 output normals are never used
as an inverse rotation. AABB local minimum and maximum remain separate, including
odd raw dimensions; averaging them does not shrink an endpoint.

Triangles may have arbitrary orientation, including slopes. Separate triangles
must actually leave out the floor under a hole; a visual hole inside a solid
triangle does not remove physical support. Build walls with thickness or use
double sided triangle surfaces as appropriate for the content.

## Contact and motion conventions

A contact normal points from the right shape toward the left shape. Depth is
nonnegative. Each witness point is on its respective shape. The narrow phase
uses `KF_GEOMETRY_TOLERANCE` (8 raw Q16.16 units, approximately 0.000122 units)
for touching and sweep convergence. A separation within this tolerance returns
contact with zero depth. Normals and witnesses have fixed point rounding error.
The contact API returns the deepest point of the contact manifold.

`kf_shape_manifold` returns every point within a caller margin, at most four.
When a face separates the shapes, the incident face (or a capsule core parallel
to it) is clipped against the reference face, and clipped point sets larger
than four keep the deepest point, its farthest partner, and the widest points
on opposite sides of their diagonal. Area ranking uses raw 64-bit determinants
in a nonsingular coordinate-plane projection of the incident polygon, avoiding
Q16 area truncation or wraparound as face size changes. Parallel
capsules, and cores within about one degree of parallel whose endpoint closest
directions agree with the shared normal, touch at both ends of their overlap
when both ends lie within the margin. Witnesses are projected onto their own
rounded surfaces. Other crossing cores use their single closest pair. Other
feature pairs produce one point. Each point has a witness on each shape and
their signed separation along the shared normal.

Rounded edge and vertex contacts retain their closest feature witnesses. When
a preferred face normal lies near an edge, the polygon witness is clamped to
the face boundary. Closest core points and clipping retain Q32 parameters until
their coordinates are evaluated, including long thin capsules. Coincident or
nearly coincident crossing capsule cores use a normal perpendicular to both
cores; parallel and point cores use deterministic perpendicular fallbacks.
Segment pairs retain exact wide normal equations before computing Q32 fractions,
so short cores and nearly parallel crossings are not erased by common scaling.

Sweeps translate the entire displacement vector. They return the first
approached boundary. Tangential or separating initial contact returns
`KF_NO_HIT`, including initial penetration when the displacement points along
its selected escape normal. An inward displacement from penetration returns a
zero fraction hit with `started_inside = KF_TRUE`. An exact hit on a
zero-thickness plane retains the normal from the approached side. Use the contact query for
stationary penetration and the world's bounded depenetration API to remove it.

`kf_sweep_hit_t.position` is the moving **pose position** at impact.
`contact_point` is the obstacle surface witness. `fraction` is Q16.16 in [0, 1],
`distance` is the distance traveled by the moving reference position. The `fraction_q32` ordering key and internal
sweep fractions use 32 fractional bits, so high speed motion is not quantized to
the public fraction resolution during collision finding. Reconstructing the
impact position from the public fraction can lose precision; use the returned
position. Object and shape IDs are filled by world queries; direct geometry
queries leave them zero.

Polyhedron contacts use separating axes from face normals and every pair of
edge directions; a triangle also tests its in-plane edge normals. Internal
support directions and face planes retain Q30 ratios through projection and
clipping; only the final public normal is narrowed to Q16. Face planes are
derived from transformed features; an extrusion retains its limiting side and
vertical directions at zero height or when tiny world edges round to zero.
Rounded nearest-feature axes come from the
feature lines before witness coordinates are rounded, so a small tangential
residual cannot grow with a long edge. Face axes are
preferred over edge axes within a small tolerance while penetrating. A positive
separating gap always takes precedence over that preference. A sphere, or a capsule
parallel to a box axis, is measured exactly in the box frame. Other rounded
pairs add exact nearest vertex and edge feature axes for the sphere or capsule
core, using closest points between arbitrary segments. Sweeps use bounded
conservative advancement on these supporting axes. The hard limit is 96
iterations; nonconvergence returns `KF_ITERATION_LIMIT`, never a successful
no-hit result. Orientation is constant during a translational query sweep. The
world's own continuous collision also bounds the speed of rotating shape points,
so spinning bodies are swept including their rotation. For any nonzero angular
path, CCD follows the recorded solver substeps and their actual rotation
vectors, retaining full revolutions and changes of rotation axis. Its speed
bound covers every recorded segment and the remaining path after an earlier
impact.
Internal CCD also evaluates rotations between contained substep poses when the
intermediate shape briefly crosses a world plane. Distant virtual shape pairs
use an exact axis gap before feature arithmetic to keep coordinate differences
representable. Public geometry queries continue to enforce the world bounds.

## Ranges and failures

World geometry including its radius must fit within +/-4096 units on each axis.
Sphere radius, capsule radius/height, and box half extents are at most 512 units.
AABB dimensions and prism extrusion spans are at most 1024 units. Prism XZ
vertices are within +/-512 local units. Local centers, triangle vertices, and
poses are within +/-4096 units; transformed bounds must still fit the world
range. The endpoint of a sweep must also satisfy these bounds. Each displacement
component is at most 8192 units. Ray maximum distance is at most 8192 units and
ray direction components are within +/-4096 units.
Mass extents are measured in the body frame without imposing the world bounds
on untranslated local vertices. A local center at a coordinate limit is valid
when its body pose places the complete shape inside the world.

`KF_INVALID_ARGUMENT` reports null required outputs, invalid shape types,
inverted bounds, degenerate triangles, invalid capsules, and invalid polygons.
`KF_OUT_OF_RANGE` reports unsupported coordinates or sizes. Invalid data is
checked before feature arithmetic and is not converted into `KF_NO_HIT`.
No query allocates memory. The narrow phase uses bounded stack feature arrays.

The rigid-frame closest-point design follows the separation between transforms
and output normals used by [Bullet transforms](https://github.com/bulletphysics/bullet3/blob/master/src/LinearMath/btTransform.h)
and [Jolt box queries](https://github.com/jrouwe/JoltPhysics/blob/master/Jolt/Physics/Collision/Shape/BoxShape.cpp).
The contact reduction preserves depth and support area, as in
[Bullet persistent manifolds](https://github.com/bulletphysics/bullet3/blob/master/src/BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp)
and [Jolt face manifolds](https://github.com/jrouwe/JoltPhysics/blob/master/Jolt/Physics/Collision/ManifoldBetweenTwoFaces.cpp).
Kinefix implements those geometric principles with bounded integer arithmetic.

## Relative motion and curved paths

`kf_shape_sweep_relative` validates both actual trajectories, then casts using
`moving_displacement - target_displacement`. Its virtual relative endpoint may
leave the coordinate range while both actual trajectories remain valid. The
internal Q32 impact fraction is retained when converting position and obstacle
witness back to world space; this conversion does not use the rounded public
fraction. `kf_world_sweep_moving` predicts each nonstatic target's translation
from its current velocity and the world's rational timestep, including its
position remainder. It does not predict gravity acceleration or rotation.

`kf_world_sweep_curve` casts a piecewise linear approximation to a quadratic
Bezier pose path. Segment counts are powers of two and satisfy a chord deviation
bound `|P0 - 2*P1 + P2| / (4*n*n)`, with four additional raw units reserved for
fixed point evaluation. The caller's segment limit is strict: insufficient
segments return `KF_ITERATION_LIMIT`. Returned fractions describe the Bezier
parameter and distances describe the polyline distance traveled. Later returns
to an obstacle are retained; contiguous segments can report the same obstacle
more than once while initially penetrating it.

The curve query casts its chords, without inflating the shape. Consequently the
specified approximation error also bounds a possible missed near-grazing
contact; this is not an analytic or conservative cast of the exact curve.
Callers requiring conservative detection can enlarge a sphere's or capsule's
radius by the chosen tolerance (and increase capsule height by twice that
amount while moving its bottom down by the tolerance). The enlarged shape must
still satisfy the size and coordinate limits. Gameplay decides whether to use
that extra margin and how to continue after each hit.
