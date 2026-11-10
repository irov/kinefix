#include "kinefix/kinefix.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK( Condition )                                                \
    do                                                                    \
    {                                                                     \
        if( ( Condition ) == 0 )                                          \
        {                                                                 \
            fprintf( stderr, "geometry:%d: %s\n", __LINE__, #Condition ); \
            return EXIT_FAILURE;                                          \
        }                                                                 \
    } while( 0 )
#define FIX( Value ) ( (kf_fixed_t)( ( Value ) * KF_FIXED_SCALE ) )

static kf_shape_t make_box( kf_vec3_t _minimum, kf_vec3_t _maximum )
{
    kf_shape_t shape;
    memset( &shape, 0, sizeof( shape ) );
    shape.type = KF_SHAPE_AABB;
    shape.data.aabb.minimum = _minimum;
    shape.data.aabb.maximum = _maximum;
    return shape;
}

static kf_shape_t make_sphere( kf_fixed_t _radius )
{
    kf_shape_t shape;
    memset( &shape, 0, sizeof( shape ) );
    shape.type = KF_SHAPE_SPHERE;
    shape.data.sphere.radius = _radius;
    return shape;
}

static int test_capsule_edge_witnesses( void )
{
    kf_shape_t capsule;
    kf_shape_t box = make_box( (kf_vec3_t){ FIX( -1 ), FIX( -1 ), FIX( -1 ) }, (kf_vec3_t){ FIX( 1 ), FIX( 1 ), FIX( 1 ) } );
    kf_pose_t identity = { { 0, 0, 0 }, { 0, 0, 0 } };
    kf_pose_t fixtures[2] = {
        { { 46883, -51896, 107972 }, { 421417963, -233962416, 380743175 } },
        { { 86764, -127089, 129300 }, { 896274800, 256101163, 240927265 } }
    };
    uint32_t fixture;
    kf_contact_t contact;
    kf_contact_t reverse;
    kf_sweep_hit_t hit;
    kf_vec3_t offset;
    memset( &capsule, 0, sizeof( capsule ) );
    capsule.type = KF_SHAPE_CAPSULE;
    capsule.data.capsule.position.y = FIX( -2 );
    capsule.data.capsule.radius = FIX( 0.4 );
    capsule.data.capsule.height = FIX( 4 );
    for( fixture = 0; fixture != 2u; ++fixture )
    {
        kf_pose_t pose = fixtures[fixture];
        CHECK( kf_shape_contact( &capsule, pose, &box, identity, &contact ) == KF_OK );
        CHECK( kf_fixed_abs( contact.point_right.x ) <= FIX( 1 ) + KF_GEOMETRY_TOLERANCE );
        CHECK( kf_fixed_abs( contact.point_right.y ) <= FIX( 1 ) + KF_GEOMETRY_TOLERANCE );
        CHECK( kf_fixed_abs( contact.point_right.z ) <= FIX( 1 ) + KF_GEOMETRY_TOLERANCE );
        CHECK( kf_shape_contact( &box, identity, &capsule, pose, &reverse ) == KF_OK );
        CHECK( kf_vec3_equal( reverse.point_left, contact.point_right ) == KF_TRUE );
        CHECK( kf_vec3_equal( reverse.point_right, contact.point_left ) == KF_TRUE );
        CHECK( kf_vec3_equal( reverse.normal, kf_vec3_neg( contact.normal ) ) == KF_TRUE );
        offset = contact.normal;
        pose.position = kf_vec3_add( pose.position, offset );
        CHECK( kf_shape_sweep( &capsule, pose, kf_vec3_neg( offset ), &box, identity, &hit ) == KF_OK );
        CHECK( kf_fixed_abs( hit.contact_point.x ) <= FIX( 1 ) + KF_GEOMETRY_TOLERANCE );
        CHECK( kf_fixed_abs( hit.contact_point.y ) <= FIX( 1 ) + KF_GEOMETRY_TOLERANCE );
        CHECK( kf_fixed_abs( hit.contact_point.z ) <= FIX( 1 ) + KF_GEOMETRY_TOLERANCE );
        CHECK( kf_shape_sweep( &box, identity, offset, &capsule, pose, &hit ) == KF_OK );
        {
            kf_vec3_t local = kf_rotation_apply_inverse( pose.rotation, kf_vec3_sub( hit.contact_point, pose.position ) );
            kf_vec3_t core = { 0, kf_fixed_clamp( local.y, FIX( -1.6 ), FIX( 1.6 ) ), 0 };
            kf_fixed_t radius = kf_vec3_length( kf_vec3_sub( local, core ) );
            CHECK( kf_fixed_abs( radius - capsule.data.capsule.radius ) <= KF_GEOMETRY_TOLERANCE * 4 );
        }
    }
    return EXIT_SUCCESS;
}

static int test_coincident_capsule_cores( void )
{
    kf_shape_t capsule;
    kf_pose_t identity = { { 0, 0, 0 }, { 0, 0, 0 } };
    kf_pose_t across = identity;
    kf_pose_t escaped = identity;
    kf_contact_t contact;
    memset( &capsule, 0, sizeof( capsule ) );
    capsule.type = KF_SHAPE_CAPSULE;
    capsule.data.capsule.position.y = FIX( -2 );
    capsule.data.capsule.radius = FIX( 0.25 );
    capsule.data.capsule.height = FIX( 4 );
    across.rotation = kf_rotation_from_axis_angle( (kf_vec3_t){ 0, 0, KF_FIXED_SCALE }, FIX( 1.5707963 ) );
    CHECK( kf_shape_contact( &capsule, identity, &capsule, across, &contact ) == KF_OK );
    escaped.position = kf_vec3_mul( contact.normal, contact.depth + KF_GEOMETRY_TOLERANCE * 2 );
    CHECK( kf_shape_contact( &capsule, escaped, &capsule, across, &contact ) == KF_NO_HIT );
    CHECK( kf_shape_contact( &capsule, identity, &capsule, identity, &contact ) == KF_OK );
    escaped.position = kf_vec3_mul( contact.normal, contact.depth + KF_GEOMETRY_TOLERANCE * 2 );
    CHECK( kf_shape_contact( &capsule, escaped, &capsule, identity, &contact ) == KF_NO_HIT );
    {
        kf_shape_t point = make_sphere( 0 );
        CHECK( kf_shape_contact( &point, identity, &capsule, identity, &contact ) == KF_OK );
        escaped.position = kf_vec3_mul( contact.normal, contact.depth + KF_GEOMETRY_TOLERANCE * 2 );
        CHECK( kf_shape_contact( &point, escaped, &capsule, identity, &contact ) == KF_NO_HIT );
        CHECK( kf_shape_contact( &point, identity, &point, identity, &contact ) == KF_OK );
    }
    return EXIT_SUCCESS;
}

static int test_long_thin_capsule( void )
{
    kf_shape_t capsule;
    kf_shape_t sphere = make_sphere( FIX( 0.001 ) );
    kf_pose_t pose = { { 0, 0, 0 }, { 0, 0, 0 } };
    kf_pose_t spherePose = pose;
    kf_contact_t contact;
    memset( &capsule, 0, sizeof( capsule ) );
    capsule.type = KF_SHAPE_CAPSULE;
    capsule.data.capsule.radius = FIX( 0.001 );
    capsule.data.capsule.height = FIX( 500 );
    pose.rotation = kf_rotation_from_axis_angle( (kf_vec3_t){ 0, 0, KF_FIXED_SCALE }, FIX( 0.785398 ) );
    spherePose.position = kf_rotation_apply( pose.rotation, (kf_vec3_t){ 0, FIX( 123.4567 ), 0 } );
    CHECK( kf_shape_contact( &sphere, spherePose, &capsule, pose, &contact ) == KF_OK );
    CHECK( contact.depth >= FIX( 0.0019 ) );
    CHECK( kf_shape_contact( &capsule, pose, &sphere, spherePose, &contact ) == KF_OK );
    {
        kf_pose_t across = { { 0, 0, 0 }, { 0, 0, 0 } };
        across.rotation = kf_rotation_from_axis_angle( (kf_vec3_t){ KF_FIXED_SCALE, 0, 0 }, FIX( 0.785398 ) );
        across.position = kf_vec3_sub( spherePose.position, kf_rotation_apply( across.rotation, (kf_vec3_t){ 0, FIX( 300.1234 ), 0 } ) );
        CHECK( kf_shape_contact( &capsule, across, &capsule, pose, &contact ) == KF_OK );
        CHECK( contact.depth >= FIX( 0.0019 ) );
        {
            kf_pose_t escaped = across;
            escaped.position = kf_vec3_add( escaped.position, kf_vec3_mul( contact.normal, contact.depth + KF_GEOMETRY_TOLERANCE * 2 ) );
            CHECK( kf_shape_contact( &capsule, escaped, &capsule, pose, &contact ) == KF_NO_HIT );
        }
        CHECK( kf_shape_contact( &capsule, pose, &capsule, across, &contact ) == KF_OK );
    }
    return EXIT_SUCCESS;
}

static int test_capsule_core_scales( void )
{
    kf_fixed_t heights[3][2] = { { FIX( 0.01 ), FIX( 500 ) }, { FIX( 500 ), FIX( 500 ) }, { FIX( 0.003 ), FIX( 500 ) } };
    kf_fixed_t angles[3] = { FIX( 0.785398 ), FIX( 0.0001 ), FIX( 0.785398 ) };
    uint32_t index;
    for( index = 0; index != 3u; ++index )
    {
        kf_shape_t left;
        kf_shape_t right;
        kf_pose_t pose = { { FIX( 3 ), FIX( -7 ), FIX( 11 ) }, { 0, 0, 0 } };
        kf_pose_t turned = pose;
        kf_contact_t contact;
        memset( &left, 0, sizeof( left ) );
        left.type = KF_SHAPE_CAPSULE;
        left.data.capsule.radius = FIX( 0.001 );
        left.data.capsule.height = heights[index][0];
        left.data.capsule.position.y = -heights[index][0] / 2;
        right = left;
        right.data.capsule.height = heights[index][1];
        right.data.capsule.position.y = -heights[index][1] / 2;
        turned.rotation = kf_rotation_from_axis_angle( (kf_vec3_t){ 0, 0, KF_FIXED_SCALE }, angles[index] );
        CHECK( kf_shape_contact( &left, pose, &right, turned, &contact ) == KF_OK );
        CHECK( contact.depth >= FIX( 0.0018 ) );
        CHECK( kf_shape_contact( &right, turned, &left, pose, &contact ) == KF_OK );
        CHECK( contact.depth >= FIX( 0.0018 ) );
        turned.position.z += left.data.capsule.radius + right.data.capsule.radius + KF_GEOMETRY_TOLERANCE * 2;
        CHECK( kf_shape_contact( &left, pose, &right, turned, &contact ) == KF_NO_HIT );
    }
    return EXIT_SUCCESS;
}

static kf_fixed_t capsule_core_distance( const kf_shape_t * _shape, kf_pose_t _pose, kf_vec3_t _point )
{
    kf_vec3_t local = kf_rotation_apply_inverse( _pose.rotation, kf_vec3_sub( _point, _pose.position ) );
    kf_vec3_t core = _shape->data.capsule.position;
    core.y = kf_fixed_clamp( local.y, core.y + _shape->data.capsule.radius, core.y + _shape->data.capsule.height - _shape->data.capsule.radius );
    return kf_vec3_length( kf_vec3_sub( local, core ) );
}

static int test_near_parallel_capsule_manifold( void )
{
    kf_shape_t capsule;
    kf_pose_t identity = { { 0, 0, 0 }, { 0, 0, 0 } };
    kf_pose_t turned = identity;
    kf_manifold_t manifold;
    uint32_t order;
    memset( &capsule, 0, sizeof( capsule ) );
    capsule.type = KF_SHAPE_CAPSULE;
    capsule.data.capsule.radius = FIX( 0.4 );
    capsule.data.capsule.height = FIX( 500 );
    capsule.data.capsule.position.y = FIX( -250 );
    turned.rotation = kf_rotation_from_axis_angle( (kf_vec3_t){ 0, 0, KF_FIXED_SCALE }, FIX( 0.01 ) );
    for( order = 0; order != 2u; ++order )
    {
        kf_pose_t left = order == 0 ? identity : turned;
        kf_pose_t right = order == 0 ? turned : identity;
        uint32_t index;
        CHECK( kf_shape_manifold( &capsule, left, &capsule, right, 0, &manifold ) == KF_OK );
        CHECK( manifold.count != 0 );
        for( index = 0; index != manifold.count; ++index )
        {
            kf_manifold_point_t point = manifold.points[index];
            CHECK( kf_fixed_abs( capsule_core_distance( &capsule, left, point.point_left ) - capsule.data.capsule.radius ) <= 16 );
            CHECK( kf_fixed_abs( capsule_core_distance( &capsule, right, point.point_right ) - capsule.data.capsule.radius ) <= 16 );
            CHECK( capsule_core_distance( &capsule, right, point.point_left ) <= capsule.data.capsule.radius + 16 );
            CHECK( capsule_core_distance( &capsule, left, point.point_right ) <= capsule.data.capsule.radius + 16 );
        }
    }
    turned = identity;
    turned.position.x = FIX( 0.5 );
    CHECK( kf_shape_manifold( &capsule, identity, &capsule, turned, 0, &manifold ) == KF_OK );
    CHECK( manifold.count == 2 );
    return EXIT_SUCCESS;
}

static int test_capsule_manifold_normal( void )
{
    kf_fixed_t radii[4] = { FIX( 0.01 ), FIX( 0.4 ), FIX( 8 ), FIX( 128 ) };
    kf_fixed_t heights[4] = { FIX( 0.1 ), FIX( 2 ), FIX( 20 ), FIX( 512 ) };
    uint32_t scale;
    for( scale = 0; scale != 4u; ++scale )
    {
        kf_shape_t capsule;
        kf_pose_t left = { { FIX( 3 ), FIX( -7 ), FIX( 11 ) }, { 0, 0, 0 } };
        uint32_t orientation;
        memset( &capsule, 0, sizeof( capsule ) );
        capsule.type = KF_SHAPE_CAPSULE;
        capsule.data.capsule.radius = radii[scale];
        capsule.data.capsule.height = heights[scale];
        capsule.data.capsule.position.y = -heights[scale] / 2;
        for( orientation = 0; orientation != 4u; ++orientation )
        {
            kf_pose_t right = left;
            kf_manifold_t forward;
            kf_manifold_t reverse;
            if( orientation == 0 || orientation == 3u )
            {
                right.rotation = kf_rotation_from_axis_angle( (kf_vec3_t){ 0, 0, KF_FIXED_SCALE }, FIX( 0.01 ) );
                if( orientation == 3u )
                {
                    right.position.z += radii[scale];
                }
            }
            else
            {
                right.position.x += radii[scale];
                if( orientation == 2u )
                {
                    right.rotation.z = KF_ROTATION_SCALE;
                }
            }
            CHECK( kf_shape_manifold( &capsule, left, &capsule, right, 0, &forward ) == KF_OK );
            CHECK( kf_shape_manifold( &capsule, right, &capsule, left, 0, &reverse ) == KF_OK );
            CHECK( forward.count != 0 );
            if( orientation == 1u || orientation == 2u )
            {
                CHECK( forward.count == 2u );
            }
            if( orientation == 3u && scale != 0 )
            {
                CHECK( forward.count == 2u );
            }
            CHECK( reverse.count == forward.count );
            CHECK( kf_vec3_equal( forward.normal, kf_vec3_neg( reverse.normal ) ) == KF_TRUE );
            for( uint32_t order = 0; order != 2u; ++order )
            {
                const kf_manifold_t * manifold = order == 0 ? &forward : &reverse;
                const kf_manifold_t * other = order == 0 ? &reverse : &forward;
                kf_pose_t leftPose = order == 0 ? left : right;
                kf_pose_t rightPose = order == 0 ? right : left;
                uint32_t index;
                for( index = 0; index != manifold->count; ++index )
                {
                    kf_manifold_point_t point = manifold->points[index];
                    kf_vec3_t difference = kf_vec3_sub( point.point_left, point.point_right );
                    kf_fixed_t separation = kf_vec3_dot( difference, manifold->normal );
                    kf_bool_t matched = KF_FALSE;
                    uint32_t otherIndex;
                    CHECK( kf_fixed_abs( separation - point.separation ) <= 16 );
                    CHECK( kf_fixed_abs( capsule_core_distance( &capsule, leftPose, point.point_left ) - radii[scale] ) <= 16 );
                    CHECK( kf_fixed_abs( capsule_core_distance( &capsule, rightPose, point.point_right ) - radii[scale] ) <= 16 );
                    for( otherIndex = 0; otherIndex != other->count; ++otherIndex )
                    {
                        kf_manifold_point_t otherPoint = other->points[otherIndex];
                        kf_fixed_t leftError = kf_vec3_length( kf_vec3_sub( point.point_left, otherPoint.point_right ) );
                        kf_fixed_t rightError = kf_vec3_length( kf_vec3_sub( point.point_right, otherPoint.point_left ) );
                        if( leftError <= 16 && rightError <= 16 )
                        {
                            matched = KF_TRUE;
                        }
                    }
                    if( orientation != 3u )
                    {
                        CHECK( matched == KF_TRUE );
                    }
                }
            }
        }
    }
    return EXIT_SUCCESS;
}

static int test_separating_feature_preference( void )
{
    kf_shape_t prism;
    kf_shape_t box = make_box( (kf_vec3_t){ FIX( -1 ), FIX( -1 ), FIX( -1 ) }, (kf_vec3_t){ FIX( 1 ), FIX( 1 ), FIX( 1 ) } );
    kf_pose_t identity = { { 0, 0, 0 }, { 0, 0, 0 } };
    kf_fixed_t radii[3] = { FIX( 1 ), FIX( 8 ), FIX( 128 ) };
    kf_fixed_t offsets[3] = { 2000, 5660, 22624 };
    uint32_t scale;
    memset( &prism, 0, sizeof( prism ) );
    prism.type = KF_SHAPE_PRISM;
    prism.data.prism.count = 4;
    prism.data.prism.minimum_y = FIX( -1 );
    prism.data.prism.maximum_y = FIX( 1 );
    prism.data.prism.vertices[0] = (kf_vec3_t){ FIX( -1 ), 0, FIX( -1 ) };
    prism.data.prism.vertices[1] = (kf_vec3_t){ FIX( 1 ), 0, FIX( -1 ) };
    prism.data.prism.vertices[2] = (kf_vec3_t){ FIX( 1 ), 0, FIX( 1 ) };
    prism.data.prism.vertices[3] = (kf_vec3_t){ FIX( -1 ), 0, FIX( 1 ) };
    for( scale = 0; scale != 3u; ++scale )
    {
        kf_shape_t sphere = make_sphere( radii[scale] );
        kf_pose_t pose = { { FIX( 1 ) + radii[scale] - 1, 0, FIX( 1 ) + offsets[scale] }, { 0, 0, 0 } };
        kf_contact_t contact;
        kf_manifold_t manifold;
        kf_sweep_hit_t hit;
        int64_t cornerX = (int64_t)radii[scale] - 1;
        int64_t cornerZ = offsets[scale];
        int64_t touchingRadius = (int64_t)radii[scale] + KF_GEOMETRY_TOLERANCE;
        /* The edge separates beyond the touching tolerance while the X face
           alone overlaps by one raw unit. A zero margin must also reject it. */
        CHECK( cornerX * cornerX + cornerZ * cornerZ > touchingRadius * touchingRadius );
        CHECK( kf_shape_contact( &sphere, pose, &box, identity, &contact ) == KF_NO_HIT );
        CHECK( kf_shape_contact( &sphere, pose, &prism, identity, &contact ) == KF_NO_HIT );
        CHECK( kf_shape_contact( &prism, identity, &sphere, pose, &contact ) == KF_NO_HIT );
        CHECK( kf_shape_manifold( &sphere, pose, &prism, identity, 0, &manifold ) == KF_NO_HIT );
        CHECK( kf_shape_manifold( &prism, identity, &sphere, pose, 0, &manifold ) == KF_NO_HIT );
        pose.position.x += FIX( 1 );
        CHECK( kf_shape_sweep( &sphere, pose, (kf_vec3_t){ FIX( -2 ), 0, 0 }, &prism, identity, &hit ) == KF_OK );
        pose.position = hit.position;
        CHECK( kf_shape_contact( &sphere, pose, &box, identity, &contact ) == KF_OK );
        pose.position.x -= 64;
        CHECK( kf_shape_contact( &sphere, pose, &prism, identity, &contact ) == KF_OK );
    }
    return EXIT_SUCCESS;
}

static int test_plane_endpoint_ray_normal( void )
{
    kf_fixed_t heights[4] = { 0, 0, -KF_COORDINATE_LIMIT, KF_COORDINATE_LIMIT };
    int32_t sides[4] = { 1, -1, 1, -1 };
    kf_pose_t identity = { { 0, 0, 0 }, { 0, 0, 0 } };
    uint32_t fixture;
    for( fixture = 0; fixture != 4u; ++fixture )
    {
        kf_shape_t shapes[2];
        kf_shape_t point = make_sphere( 0 );
        kf_pose_t touching = identity;
        kf_contact_t contact;
        kf_ray_t ray = { { 0, heights[fixture] + sides[fixture] * FIX( 0.25 ), 0 }, { 0, -sides[fixture] * KF_FIXED_SCALE, 0 }, FIX( 0.25 ) };
        uint32_t shapeIndex;
        shapes[0] = make_box( (kf_vec3_t){ FIX( -1 ), heights[fixture], FIX( -1 ) }, (kf_vec3_t){ FIX( 1 ), heights[fixture], FIX( 1 ) } );
        memset( shapes + 1, 0, sizeof( shapes[1] ) );
        shapes[1].type = KF_SHAPE_TRIANGLE;
        shapes[1].data.triangle.vertices[0] = (kf_vec3_t){ FIX( -1 ), heights[fixture], FIX( -1 ) };
        shapes[1].data.triangle.vertices[1] = (kf_vec3_t){ FIX( 1 ), heights[fixture], FIX( -1 ) };
        shapes[1].data.triangle.vertices[2] = (kf_vec3_t){ 0, heights[fixture], FIX( 1 ) };
        for( shapeIndex = 0; shapeIndex != 2u; ++shapeIndex )
        {
            kf_sweep_hit_t hit;
            CHECK( kf_shape_raycast( &ray, shapes + shapeIndex, identity, &hit ) == KF_OK );
            CHECK( hit.fraction_q32 == ( UINT64_C(1) << 32 ) );
            CHECK( hit.position.y == heights[fixture] );
            CHECK( hit.contact_point.y == heights[fixture] );
            CHECK( hit.normal.y == sides[fixture] * KF_FIXED_SCALE );
            CHECK( hit.started_inside == KF_FALSE );
        }
        /* Stationary overlap retains its deterministic escape tie. */
        touching.position.y = heights[fixture];
        CHECK( kf_shape_contact( &point, touching, shapes, identity, &contact ) == KF_OK );
        CHECK( contact.normal.y == -KF_FIXED_SCALE );
    }
    return EXIT_SUCCESS;
}

static int test_translated_local_extent( void )
{
    kf_world_t * world = NULL;
    kf_world_config_t config = kf_world_config_default();
    kf_body_desc_t body = kf_body_desc_default( 1 );
    kf_shape_desc_t shape;
    kf_handle_t handle;
    kf_body_state_t state;
    memset( &shape, 0, sizeof( shape ) );
    config.gravity = (kf_vec3_t){ 0, 0, 0 };
    body.type = KF_BODY_DYNAMIC;
    body.pose.position.x = -KF_COORDINATE_LIMIT;
    shape.id = 1;
    shape.shape = make_sphere( FIX( 1 ) );
    shape.shape.data.sphere.position.x = KF_COORDINATE_LIMIT;
    CHECK( kf_world_create( &config, NULL, &world ) == KF_OK );
    CHECK( kf_world_reserve( world, (kf_capacity_t){ 1, 1, 0, 0, 0 } ) == KF_OK );
    CHECK( kf_world_add_body( world, &body, &shape, 1, &handle ) == KF_OK );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
    CHECK( kf_vec3_equal( state.pose.position, body.pose.position ) == KF_TRUE );
    kf_world_destroy( world );
    return EXIT_SUCCESS;
}

static int test_box_rigid_frame( void )
{
    kf_rotation_t rotations[4] = {
        { 0, 0, 0 },
        kf_rotation_from_axis_angle( (kf_vec3_t){ FIX( 1 ), FIX( 2 ), FIX( 3 ) }, 80656 ),
        kf_rotation_from_axis_angle( (kf_vec3_t){ FIX( -3 ), FIX( 2 ), FIX( 1 ) }, FIX( 2.3 ) ),
        kf_rotation_from_axis_angle( (kf_vec3_t){ FIX( 1 ), FIX( -1 ), FIX( 2 ) }, FIX( 0.77 ) )
    };
    const kf_angle16_t yaws[3] = { 0, 4096, 19283 };
    const kf_fixed_t scales[3] = { FIX( 1 ), FIX( 128 ), KF_SIZE_LIMIT };
    const kf_vec3_t directions[9] = {
        { KF_FIXED_SCALE, 0, 0 }, { -KF_FIXED_SCALE, 0, 0 },
        { 0, KF_FIXED_SCALE, 0 }, { 0, -KF_FIXED_SCALE, 0 },
        { 0, 0, KF_FIXED_SCALE }, { 0, 0, -KF_FIXED_SCALE },
        { FIX( 0.6 ), FIX( 0.8 ), 0 }, { FIX( -0.6 ), 0, FIX( 0.8 ) },
        { 21845, 43691, 43691 }
    };
    uint32_t rotation;
    uint32_t yaw;
    uint32_t scale;
    uint32_t feature;
    uint32_t inside;
    uint32_t representation;
    for( rotation = 0; rotation != 4u; ++rotation )
    {
        for( yaw = 0; yaw != 3u; ++yaw )
        {
            for( scale = 0; scale != 3u; ++scale )
            {
                kf_shape_t box;
                kf_shape_t sphere = make_sphere( FIX( 0.25 ) );
                kf_shape_t equivalents[3];
                kf_pose_t equivalentPoses[3];
                kf_pose_t boxPose = { { FIX( 17 ), FIX( -23 ), FIX( 11 ) }, { 0, 0, 0 } };
                kf_rotation_t localYaw = kf_rotation_from_yaw( yaws[yaw] );
                memset( &box, 0, sizeof( box ) );
                box.type = KF_SHAPE_BOX;
                box.data.box.half_extents = (kf_vec3_t){ scales[scale], scales[scale] / 2, scales[scale] / 4 };
                box.data.box.center = (kf_vec3_t){ FIX( 3 ), FIX( -2 ), FIX( 1 ) };
                box.data.box.yaw = yaws[yaw];
                boxPose.rotation = rotations[rotation];
                equivalents[0] = box;
                equivalentPoses[0] = boxPose;
                equivalentPoses[1] = boxPose;
                equivalentPoses[1].position = kf_vec3_add( boxPose.position, kf_rotation_apply( boxPose.rotation, box.data.box.center ) );
                equivalentPoses[1].rotation = kf_rotation_multiply( boxPose.rotation, localYaw );
                equivalentPoses[2] = equivalentPoses[1];
                equivalents[1] = make_box( kf_vec3_neg( box.data.box.half_extents ), box.data.box.half_extents );
                memset( equivalents + 2, 0, sizeof( equivalents[2] ) );
                equivalents[2].type = KF_SHAPE_PRISM;
                equivalents[2].data.prism.count = 4;
                equivalents[2].data.prism.minimum_y = -box.data.box.half_extents.y;
                equivalents[2].data.prism.maximum_y = box.data.box.half_extents.y;
                equivalents[2].data.prism.vertices[0] = (kf_vec3_t){ -box.data.box.half_extents.x, 0, -box.data.box.half_extents.z };
                equivalents[2].data.prism.vertices[1] = (kf_vec3_t){ box.data.box.half_extents.x, 0, -box.data.box.half_extents.z };
                equivalents[2].data.prism.vertices[2] = (kf_vec3_t){ box.data.box.half_extents.x, 0, box.data.box.half_extents.z };
                equivalents[2].data.prism.vertices[3] = (kf_vec3_t){ -box.data.box.half_extents.x, 0, box.data.box.half_extents.z };
                for( inside = 0; inside != 2u; ++inside )
                {
                    kf_shape_t capsule;
                    kf_pose_t capsulePose = equivalentPoses[1];
                    kf_vec3_t bottom = { box.data.box.half_extents.x + FIX( 0.25 ) + ( inside == 0 ? 512 : -512 ), FIX( -0.375 ), 0 };
                    memset( &capsule, 0, sizeof( capsule ) );
                    capsule.type = KF_SHAPE_CAPSULE;
                    capsule.data.capsule.radius = FIX( 0.25 );
                    capsule.data.capsule.height = FIX( 0.75 );
                    capsulePose.position = kf_vec3_add( capsulePose.position, kf_rotation_apply( capsulePose.rotation, bottom ) );
                    for( representation = 0; representation != 3u; ++representation )
                    {
                        kf_contact_t contact;
                        kf_manifold_t manifold;
                        if( inside == 0 )
                        {
                            CHECK( kf_shape_contact( &capsule, capsulePose, equivalents + representation, equivalentPoses[representation], &contact ) == KF_NO_HIT );
                            continue;
                        }
                        CHECK( kf_shape_contact( &capsule, capsulePose, equivalents + representation, equivalentPoses[representation], &contact ) == KF_OK );
                        CHECK( kf_shape_manifold( &capsule, capsulePose, equivalents + representation, equivalentPoses[representation], 0, &manifold ) == KF_OK );
                        CHECK( contact.depth >= 504 && contact.depth <= 520 );
                        CHECK( manifold.count == 2 );
                    }
                }
                for( feature = 0; feature != 9u; ++feature )
                {
                    for( inside = 0; inside != 2u; ++inside )
                    {
                        kf_vec3_t direction = directions[feature];
                        kf_fixed_t distance = sphere.data.sphere.radius + ( inside == 0 ? 512 : -512 );
                        kf_vec3_t half = box.data.box.half_extents;
                        kf_vec3_t local = {
                            ( direction.x < 0 ? -half.x : ( direction.x > 0 ? half.x : 0 ) ) + (kf_fixed_t)( (int64_t)direction.x * distance / KF_FIXED_SCALE ),
                            ( direction.y < 0 ? -half.y : ( direction.y > 0 ? half.y : 0 ) ) + (kf_fixed_t)( (int64_t)direction.y * distance / KF_FIXED_SCALE ),
                            ( direction.z < 0 ? -half.z : ( direction.z > 0 ? half.z : 0 ) ) + (kf_fixed_t)( (int64_t)direction.z * distance / KF_FIXED_SCALE )
                        };
                        kf_pose_t spherePose = { { 0, 0, 0 }, { 0, 0, 0 } };
                        kf_contact_t contact;
                        kf_contact_t reverse;
                        kf_manifold_t manifold;
                        local = kf_vec3_add( kf_rotation_apply( localYaw, local ), box.data.box.center );
                        spherePose.position = kf_vec3_add( boxPose.position, kf_rotation_apply( boxPose.rotation, local ) );
                        for( representation = 0; representation != 3u; ++representation )
                        {
                            if( inside == 0 )
                            {
                                CHECK( kf_shape_contact( &sphere, spherePose, equivalents + representation, equivalentPoses[representation], &contact ) == KF_NO_HIT );
                                CHECK( kf_shape_manifold( &sphere, spherePose, equivalents + representation, equivalentPoses[representation], 0, &manifold ) == KF_NO_HIT );
                                CHECK( kf_shape_contact( equivalents + representation, equivalentPoses[representation], &sphere, spherePose, &reverse ) == KF_NO_HIT );
                                continue;
                            }
                            CHECK( kf_shape_contact( &sphere, spherePose, equivalents + representation, equivalentPoses[representation], &contact ) == KF_OK );
                            CHECK( kf_shape_contact( equivalents + representation, equivalentPoses[representation], &sphere, spherePose, &reverse ) == KF_OK );
                            CHECK( kf_vec3_equal( contact.normal, kf_vec3_neg( reverse.normal ) ) == KF_TRUE );
                            CHECK( kf_vec3_equal( contact.point_right, reverse.point_left ) == KF_TRUE );
                            CHECK( kf_vec3_equal( contact.point_left, reverse.point_right ) == KF_TRUE );
                            CHECK( contact.depth >= 504 && contact.depth <= 520 );
                            local = kf_rotation_apply_inverse( boxPose.rotation, kf_vec3_sub( contact.point_right, boxPose.position ) );
                            local = kf_rotation_apply_inverse( localYaw, kf_vec3_sub( local, box.data.box.center ) );
                            CHECK( kf_fixed_abs( local.x ) <= half.x + 4 );
                            CHECK( kf_fixed_abs( local.y ) <= half.y + 4 );
                            CHECK( kf_fixed_abs( local.z ) <= half.z + 4 );
                            CHECK( kf_fixed_abs( kf_fixed_abs( local.x ) - half.x ) <= 4 || kf_fixed_abs( kf_fixed_abs( local.y ) - half.y ) <= 4 || kf_fixed_abs( kf_fixed_abs( local.z ) - half.z ) <= 4 );
                        }
                    }
                }
            }
        }
    }
    {
        /* Odd AABB dimensions retain both exact integer endpoints in the frame. */
        kf_shape_t box = make_box( (kf_vec3_t){ -7, -9, -11 }, (kf_vec3_t){ 4, 8, 10 } );
        kf_shape_t sphere = make_sphere( 32 );
        kf_pose_t identity = { { 0, 0, 0 }, { 0, 0, 0 } };
        kf_pose_t spherePose = { { 37, 0, 0 }, { 0, 0, 0 } };
        kf_manifold_t manifold;
        CHECK( kf_shape_manifold( &sphere, spherePose, &box, identity, 0, &manifold ) == KF_NO_HIT );
        spherePose.position.x = 36;
        CHECK( kf_shape_manifold( &sphere, spherePose, &box, identity, 0, &manifold ) == KF_OK );
        CHECK( manifold.points[0].point_right.x == 4 && manifold.points[0].separation == 0 );
    }
    return EXIT_SUCCESS;
}

static int test_manifold_area_scales( void )
{
    const int32_t x[8] = { 2, 1, -1, -2, -2, -1, 1, 2 };
    const int32_t z[8] = { 1, 2, 2, 1, -1, -2, -2, -1 };
    const int32_t scales[4] = { 1, 16, 128, 256 };
    const kf_rotation_t rotations[4] = {
        { 0, 0, 0 },
        kf_rotation_from_axis_angle( (kf_vec3_t){ FIX( 1 ), FIX( 2 ), FIX( 3 ) }, FIX( 0.8 ) ),
        kf_rotation_from_axis_angle( (kf_vec3_t){ FIX( -3 ), FIX( 1 ), FIX( 2 ) }, FIX( 1.7 ) ),
        kf_rotation_from_axis_angle( (kf_vec3_t){ FIX( 3 ), FIX( -2 ), FIX( 1 ) }, FIX( 2.6 ) )
    };
    uint32_t rotation;
    uint32_t scale;
    uint32_t winding;
    for( rotation = 0; rotation != 4u; ++rotation )
    {
        for( scale = 0; scale != 4u; ++scale )
        {
            for( winding = 0; winding != 16u; ++winding )
            {
                kf_shape_t prism;
                kf_pose_t lower = { { FIX( 31 ), FIX( -29 ), FIX( 17 ) }, { 0, 0, 0 } };
                kf_pose_t upper;
                kf_manifold_t manifold;
                kf_manifold_t reverse;
                uint32_t index;
                uint32_t previous;
                kf_vec3_t locals[KF_MANIFOLD_MAX_POINTS];
                memset( &prism, 0, sizeof( prism ) );
                prism.type = KF_SHAPE_PRISM;
                prism.data.prism.count = 8;
                prism.data.prism.minimum_y = FIX( -1 );
                prism.data.prism.maximum_y = FIX( 1 );
                for( index = 0; index != 8u; ++index )
                {
                    uint32_t vertex = winding < 8u ? ( index + winding ) % 8u : ( winding - 8u + 8u - index ) % 8u;
                    prism.data.prism.vertices[index] = (kf_vec3_t){ FIX( x[vertex] * scales[scale] ), 0, FIX( z[vertex] * scales[scale] ) };
                }
                lower.rotation = rotations[rotation];
                upper = lower;
                upper.position = kf_vec3_add( lower.position, kf_rotation_apply( lower.rotation, (kf_vec3_t){ 0, FIX( 2 ) - 4, 0 } ) );
                CHECK( kf_shape_manifold( &prism, upper, &prism, lower, 16, &manifold ) == KF_OK );
                CHECK( kf_shape_manifold( &prism, lower, &prism, upper, 16, &reverse ) == KF_OK );
                CHECK( manifold.count == 4 && reverse.count == 4 );
                for( index = 0; index != manifold.count; ++index )
                {
                    kf_vec3_t local = kf_rotation_apply_inverse( lower.rotation, kf_vec3_sub( manifold.points[index].point_left, upper.position ) );
                    CHECK( kf_fixed_abs( local.y - FIX( -1 ) ) <= KF_GEOMETRY_TOLERANCE );
                    locals[index] = local;
                    for( previous = 0; previous != index; ++previous )
                    {
                        CHECK( kf_vec3_equal( manifold.points[index].point_left, manifold.points[previous].point_left ) == KF_FALSE );
                    }
                }
                {
                    kf_vec3_t diagonal = kf_vec3_sub( locals[1], locals[0] );
                    kf_vec3_t first = kf_vec3_sub( locals[2], locals[0] );
                    kf_vec3_t second = kf_vec3_sub( locals[3], locals[0] );
                    int64_t firstArea = (int64_t)diagonal.x * first.z - (int64_t)diagonal.z * first.x;
                    int64_t secondArea = (int64_t)diagonal.x * second.z - (int64_t)diagonal.z * second.x;
                    uint64_t minimum = (uint64_t)scales[scale] * scales[scale] * KF_FIXED_SCALE * KF_FIXED_SCALE;
                    CHECK( ( firstArea < 0 ) != ( secondArea < 0 ) );
                    CHECK( (uint64_t)llabs( firstArea ) > minimum );
                    CHECK( (uint64_t)llabs( secondArea ) > minimum );
                }
            }
        }
    }
    return EXIT_SUCCESS;
}

static int test_prism_limiting_axes( void )
{
    const kf_rotation_t rotations[4] = {
        { 0, 0, 0 },
        kf_rotation_from_axis_angle( (kf_vec3_t){ FIX( 1 ), FIX( 2 ), FIX( 3 ) }, FIX( 0.8 ) ),
        kf_rotation_from_axis_angle( (kf_vec3_t){ FIX( -3 ), FIX( 1 ), FIX( 2 ) }, FIX( 1.7 ) ),
        kf_rotation_from_axis_angle( (kf_vec3_t){ FIX( 3 ), FIX( -2 ), FIX( 1 ) }, FIX( 2.6 ) )
    };
    uint32_t rotation;
    uint32_t size;
    uint32_t height;
    for( rotation = 0; rotation != 4u; ++rotation )
    {
        for( size = 0; size != 2u; ++size )
        {
            for( height = 0; height != 4u; ++height )
            {
                kf_fixed_t half = size == 0 ? 1 : FIX( 1 );
                kf_fixed_t offset = size == 0 ? 64 : FIX( 10 );
                kf_shape_t prism;
                kf_pose_t first = { { FIX( 19 ), FIX( -13 ), FIX( 11 ) }, { 0, 0, 0 } };
                kf_pose_t second;
                kf_manifold_t manifold;
                kf_contact_t contact;
                memset( &prism, 0, sizeof( prism ) );
                prism.type = KF_SHAPE_PRISM;
                prism.data.prism.count = 4;
                prism.data.prism.maximum_y = (kf_fixed_t)height;
                prism.data.prism.vertices[0] = (kf_vec3_t){ -half, 0, -half };
                prism.data.prism.vertices[1] = (kf_vec3_t){ half, 0, -half };
                prism.data.prism.vertices[2] = (kf_vec3_t){ half, 0, half };
                prism.data.prism.vertices[3] = (kf_vec3_t){ -half, 0, half };
                first.rotation = rotations[rotation];
                second = first;
                second.position = kf_vec3_add( first.position, kf_rotation_apply( first.rotation, (kf_vec3_t){ offset, 0, 0 } ) );
                CHECK( kf_shape_contact( &prism, first, &prism, second, &contact ) == KF_NO_HIT );
                CHECK( kf_shape_manifold( &prism, first, &prism, second, 0, &manifold ) == KF_NO_HIT );
                CHECK( kf_shape_contact( &prism, second, &prism, first, &contact ) == KF_NO_HIT );
                CHECK( kf_shape_contact( &prism, first, &prism, first, &contact ) == KF_OK );
                CHECK( kf_shape_manifold( &prism, first, &prism, first, KF_GEOMETRY_TOLERANCE, &manifold ) == KF_OK );
                if( size != 0 )
                {
                    CHECK( manifold.count == 4 );
                }
            }
        }
    }
    {
        /* A valid thin triangle can have collinear rounded world vertices. Its
           limiting extrusion direction still separates distant copies. */
        kf_shape_t prism;
        kf_pose_t first = { { 0, 0, 0 }, { -142435266, 751741682, 261131321 } };
        kf_pose_t second = { { -99453, 166432, 32624 }, { -142435266, 751741682, 261131321 } };
        kf_contact_t contact;
        kf_manifold_t manifold;
        memset( &prism, 0, sizeof( prism ) );
        prism.type = KF_SHAPE_PRISM;
        prism.data.prism.count = 3;
        prism.data.prism.maximum_y = FIX( 1 );
        prism.data.prism.vertices[0] = (kf_vec3_t){ FIX( -1 ), 0, 0 };
        prism.data.prism.vertices[1] = (kf_vec3_t){ FIX( 1 ), 0, 0 };
        prism.data.prism.vertices[2] = (kf_vec3_t){ FIX( 1 ) - 1, 0, 1 };
        CHECK( kf_shape_contact( &prism, first, &prism, second, &contact ) == KF_NO_HIT );
        CHECK( kf_shape_manifold( &prism, first, &prism, second, 0, &manifold ) == KF_NO_HIT );
        CHECK( kf_shape_contact( &prism, second, &prism, first, &contact ) == KF_NO_HIT );
    }
    return EXIT_SUCCESS;
}

int main( void )
{
    int regressions = test_capsule_edge_witnesses() + test_coincident_capsule_cores() + test_long_thin_capsule()
        + test_capsule_core_scales() + test_near_parallel_capsule_manifold() + test_capsule_manifold_normal()
        + test_separating_feature_preference() + test_plane_endpoint_ray_normal() + test_translated_local_extent()
        + test_box_rigid_frame() + test_manifold_area_scales() + test_prism_limiting_axes();
    kf_pose_t identity = { { 0, 0, 0 }, { 0, 0, 0 } };
    kf_shape_t wall = make_box( (kf_vec3_t){ 0, FIX( -10 ), FIX( -10 ) }, (kf_vec3_t){ FIX( 1 ), FIX( 10 ), FIX( 10 ) } );
    kf_shape_t sphere = make_sphere( FIX( 1 ) );
    kf_contact_t contact;
    kf_sweep_hit_t hit;
    kf_pose_t pose = { { FIX( -10 ), FIX( 2 ), FIX( 3 ) }, { 0, 0, 0 } };
    CHECK( regressions == EXIT_SUCCESS );
    CHECK( kf_shape_sweep( &sphere, pose, (kf_vec3_t){ FIX( 20 ), 0, 0 }, &wall, identity, &hit ) == KF_OK );
    CHECK( kf_fixed_abs( hit.position.x - FIX( -1 ) ) <= KF_GEOMETRY_TOLERANCE );
    CHECK( kf_fixed_abs( hit.contact_point.x ) <= KF_GEOMETRY_TOLERANCE );
    CHECK( hit.contact_point.y == FIX( 2 ) && hit.contact_point.z == FIX( 3 ) );
    CHECK( hit.normal.x == -KF_FIXED_SCALE && hit.normal.y == 0 );
    pose.position.x = FIX( -0.5 );
    CHECK( kf_shape_contact( &sphere, pose, &wall, identity, &contact ) == KF_OK );
    CHECK( kf_fixed_abs( contact.depth - FIX( 0.5 ) ) <= KF_GEOMETRY_TOLERANCE );
    CHECK( kf_shape_sweep( &sphere, pose, (kf_vec3_t){ FIX( -2 ), 0, 0 }, &wall, identity, &hit ) == KF_NO_HIT );
    CHECK( kf_shape_sweep( &sphere, pose, (kf_vec3_t){ FIX( 2 ), 0, 0 }, &wall, identity, &hit ) == KF_OK );
    CHECK( hit.started_inside == KF_TRUE );
    {
        kf_shape_t corner = make_box( (kf_vec3_t){ 0, 0, 0 }, (kf_vec3_t){ FIX( 2 ), FIX( 2 ), FIX( 2 ) } );
        kf_pose_t cornerPose = { { FIX( -0.9 ), FIX( -0.9 ), 0 }, { 0, 0, 0 } };
        CHECK( kf_shape_contact( &sphere, cornerPose, &corner, identity, &contact ) == KF_NO_HIT );
        CHECK( kf_shape_sweep( &sphere, cornerPose, (kf_vec3_t){ FIX( -1 ), 0, 0 }, &corner, identity, &hit ) == KF_NO_HIT );
        CHECK( kf_shape_sweep( &sphere, cornerPose, (kf_vec3_t){ FIX( 2 ), FIX( 2 ), 0 }, &corner, identity, &hit ) == KF_OK );
        CHECK( hit.started_inside == KF_FALSE );
        CHECK( hit.normal.x < FIX( -0.70 ) && hit.normal.y < FIX( -0.70 ) );
        CHECK( kf_fixed_abs( hit.contact_point.x ) <= 32 && kf_fixed_abs( hit.contact_point.y ) <= 32 );
    }
    {
        kf_shape_t capsule;
        kf_shape_t ramp;
        kf_pose_t above = { { 0, FIX( 3 ), 0 }, { 0, 0, 0 } };
        memset( &capsule, 0, sizeof( capsule ) );
        capsule.type = KF_SHAPE_CAPSULE;
        capsule.data.capsule.radius = FIX( 0.5 );
        capsule.data.capsule.height = FIX( 2 );
        memset( &ramp, 0, sizeof( ramp ) );
        ramp.type = KF_SHAPE_TRIANGLE;
        ramp.data.triangle.vertices[0] = (kf_vec3_t){ FIX( -5 ), FIX( -2.5 ), FIX( -5 ) };
        ramp.data.triangle.vertices[1] = (kf_vec3_t){ FIX( 5 ), FIX( 2.5 ), FIX( -5 ) };
        ramp.data.triangle.vertices[2] = (kf_vec3_t){ 0, 0, FIX( 5 ) };
        CHECK( kf_shape_sweep( &capsule, above, (kf_vec3_t){ 0, FIX( -5 ), 0 }, &ramp, identity, &hit ) == KF_OK );
        CHECK( hit.normal.y > FIX( 0.89 ) && hit.normal.y < FIX( 0.90 ) );
        CHECK( hit.normal.x < FIX( -0.44 ) && hit.normal.x > FIX( -0.46 ) );
        CHECK( kf_fixed_abs( hit.contact_point.y - hit.contact_point.x / 2 ) <= 32 );
        CHECK( hit.position.y > FIX( 0.05 ) && hit.position.y < FIX( 0.07 ) );
        above.position.z = FIX( 7 );
        CHECK( kf_shape_sweep( &capsule, above, (kf_vec3_t){ 0, FIX( -5 ), 0 }, &ramp, identity, &hit ) == KF_NO_HIT );
    }
    {
        kf_shape_t box;
        kf_shape_t prism;
        kf_aabb_t bounds;
        memset( &box, 0, sizeof( box ) );
        box.type = KF_SHAPE_BOX;
        box.data.box.half_extents = (kf_vec3_t){ FIX( 2 ), FIX( 1 ), FIX( 0.5 ) };
        box.data.box.yaw = (kf_angle16_t)( KF_ANGLE16_QUARTER / 2 );
        CHECK( kf_shape_bounds( &box, identity, &bounds ) == KF_OK );
        CHECK( bounds.maximum.x > FIX( 1.7 ) && bounds.maximum.z > FIX( 1.7 ) );
        pose.position = (kf_vec3_t){ FIX( 2 ), 0, 0 };
        CHECK( kf_shape_contact( &sphere, pose, &box, identity, &contact ) == KF_OK );
        memset( &prism, 0, sizeof( prism ) );
        prism.type = KF_SHAPE_PRISM;
        prism.data.prism.count = 4;
        prism.data.prism.minimum_y = FIX( -1 );
        prism.data.prism.maximum_y = FIX( 1 );
        prism.data.prism.vertices[0] = (kf_vec3_t){ FIX( -1 ), 0, FIX( -1 ) };
        prism.data.prism.vertices[1] = (kf_vec3_t){ FIX( 1 ), 0, FIX( -1 ) };
        prism.data.prism.vertices[2] = (kf_vec3_t){ FIX( 1 ), 0, FIX( 1 ) };
        prism.data.prism.vertices[3] = (kf_vec3_t){ FIX( -1 ), 0, FIX( 1 ) };
        CHECK( kf_shape_validate( &prism ) == KF_OK );
        CHECK( kf_shape_contact( &box, identity, &prism, identity, &contact ) == KF_OK );
        pose.position = (kf_vec3_t){ FIX( -20 ), 0, 0 };
        CHECK( kf_shape_sweep( &box, pose, (kf_vec3_t){ FIX( 40 ), 0, 0 }, &prism, identity, &hit ) == KF_OK );
        prism.data.prism.vertices[2] = (kf_vec3_t){ 0, 0, 0 };
        CHECK( kf_shape_validate( &prism ) == KF_INVALID_ARGUMENT );
    }
    {
        kf_shape_t thin = make_box( (kf_vec3_t){ 0, FIX( -1 ), FIX( -1 ) }, (kf_vec3_t){ 32, FIX( 1 ), FIX( 1 ) } );
        kf_shape_t projectile = make_sphere( FIX( 0.02 ) );
        kf_pose_t far = { { FIX( -1000 ), 0, 0 }, { 0, 0, 0 } };
        CHECK( kf_shape_sweep( &projectile, far, (kf_vec3_t){ FIX( 2000 ), 0, 0 }, &thin, identity, &hit ) == KF_OK );
        CHECK( kf_fixed_abs( hit.contact_point.x ) <= KF_GEOMETRY_TOLERANCE );
        far.position.x = KF_COORDINATE_LIMIT;
        CHECK( kf_shape_bounds( &projectile, far, &thin.data.aabb ) == KF_OUT_OF_RANGE );
        CHECK( kf_shape_contact( NULL, identity, &wall, identity, &contact ) == KF_INVALID_ARGUMENT );
    }
    {
        kf_shape_t distantWall = make_box( (kf_vec3_t){FIX(5), FIX(-20), FIX(-1)}, (kf_vec3_t){FIX(6), FIX(20), FIX(1)} );
        kf_ray_t shortDiagonal = {{FIX(-10), FIX(-10), 0}, {1, 1, 0}, FIX(20)};
        kf_fixed_t fraction;
        CHECK( kf_shape_raycast( &shortDiagonal, &distantWall, identity, &hit ) == KF_NO_HIT );
        shortDiagonal.direction = (kf_vec3_t){KF_FIXED_SCALE, KF_FIXED_SCALE, 0};
        CHECK( kf_shape_raycast( &shortDiagonal, &distantWall, identity, &hit ) == KF_NO_HIT );
        shortDiagonal.maximum_distance = FIX(24);
        CHECK( kf_shape_raycast( &shortDiagonal, &distantWall, identity, &hit ) == KF_OK );
        fraction = hit.fraction;
        shortDiagonal.direction = (kf_vec3_t){1, 1, 0};
        CHECK( kf_shape_raycast( &shortDiagonal, &distantWall, identity, &hit ) == KF_OK );
        CHECK( hit.fraction == fraction );
    }
    {
        kf_pcg32_t random;
        kf_shape_t corner = make_box( (kf_vec3_t){ 0, 0, 0 }, (kf_vec3_t){ FIX( 2 ), FIX( 2 ), FIX( 2 ) } );
        uint32_t index;
        kf_pcg32_seed( &random, UINT64_C(77), UINT64_C(19) );
        for( index = 0; index != 2048; ++index )
        {
            kf_vec3_t displacement;
            kf_status_t status;
            pose.position.x = (kf_fixed_t)( kf_pcg32_next( &random ) % UINT32_C(1048576) ) - FIX( 8 );
            pose.position.y = (kf_fixed_t)( kf_pcg32_next( &random ) % UINT32_C(1048576) ) - FIX( 8 );
            pose.position.z = (kf_fixed_t)( kf_pcg32_next( &random ) % UINT32_C(1048576) ) - FIX( 8 );
            displacement.x = (kf_fixed_t)( kf_pcg32_next( &random ) % UINT32_C(1048576) ) - FIX( 8 );
            displacement.y = (kf_fixed_t)( kf_pcg32_next( &random ) % UINT32_C(1048576) ) - FIX( 8 );
            displacement.z = (kf_fixed_t)( kf_pcg32_next( &random ) % UINT32_C(1048576) ) - FIX( 8 );
            status = kf_shape_sweep( &sphere, pose, displacement, &corner, identity, &hit );
            CHECK( status == KF_OK || status == KF_NO_HIT );
            if( status == KF_NO_HIT )
            {
                continue;
            }
            CHECK( hit.fraction_q32 <= (UINT64_C(1) << 32) );
            CHECK( hit.fraction >= 0 && hit.fraction <= KF_FIXED_SCALE );
            CHECK( hit.normal.x != 0 || hit.normal.y != 0 || hit.normal.z != 0 );
        }
    }
    {
        kf_shape_t shapes[6];
        uint32_t left;
        uint32_t right;
        kf_pose_t start = { { FIX( -6 ), 0, 0 }, { 0, 0, 0 } };
        memset( shapes, 0, sizeof( shapes ) );
        shapes[0] = make_box( (kf_vec3_t){ FIX( -1 ), FIX( -1 ), FIX( -1 ) }, (kf_vec3_t){ FIX( 1 ), FIX( 1 ), FIX( 1 ) } );
        shapes[1] = make_sphere( FIX( 1 ) );
        shapes[2].type = KF_SHAPE_CAPSULE;
        shapes[2].data.capsule.position.y = FIX( -1 );
        shapes[2].data.capsule.radius = FIX( 0.5 );
        shapes[2].data.capsule.height = FIX( 2 );
        shapes[3].type = KF_SHAPE_BOX;
        shapes[3].data.box.half_extents = (kf_vec3_t){ FIX( 1 ), FIX( 1 ), FIX( 1 ) };
        shapes[3].data.box.yaw = 4096;
        shapes[4].type = KF_SHAPE_PRISM;
        shapes[4].data.prism.count = 3;
        shapes[4].data.prism.minimum_y = FIX( -1 );
        shapes[4].data.prism.maximum_y = FIX( 1 );
        shapes[4].data.prism.vertices[0] = (kf_vec3_t){ FIX( -2 ), 0, FIX( -2 ) };
        shapes[4].data.prism.vertices[1] = (kf_vec3_t){ FIX( 2 ), 0, FIX( -2 ) };
        shapes[4].data.prism.vertices[2] = (kf_vec3_t){ 0, 0, FIX( 2 ) };
        shapes[5].type = KF_SHAPE_TRIANGLE;
        shapes[5].data.triangle.vertices[0] = (kf_vec3_t){ FIX( -2 ), 0, FIX( -2 ) };
        shapes[5].data.triangle.vertices[1] = (kf_vec3_t){ FIX( 2 ), 0, FIX( -2 ) };
        shapes[5].data.triangle.vertices[2] = (kf_vec3_t){ 0, 0, FIX( 2 ) };
        for( left = 0; left != 6; ++left )
        {
            for( right = 0; right != 6; ++right )
            {
                CHECK( kf_shape_contact( &shapes[left], identity, &shapes[right], identity, &contact ) == KF_OK );
                CHECK( kf_shape_contact( &shapes[left], start, &shapes[right], identity, &contact ) == KF_NO_HIT );
                CHECK( kf_shape_sweep( &shapes[left], start, (kf_vec3_t){ FIX( 12 ), 0, 0 }, &shapes[right], identity, &hit ) == KF_OK );
                CHECK( hit.fraction > 0 && hit.fraction < KF_FIXED_SCALE );
                if( left != 5 || right != 5 )
                {
                    CHECK( hit.normal.x < 0 );
                }
            }
        }
        {
            kf_shape_t invalid = shapes[4];
            invalid.data.prism.vertices[1].x = INT32_MAX;
            invalid.data.prism.vertices[2].z = INT32_MIN;
            CHECK( kf_shape_validate( &invalid ) == KF_OUT_OF_RANGE );
        }
    }
    {
        kf_shape_t first;
        kf_shape_t second;
        kf_aabb_t firstBounds;
        kf_aabb_t secondBounds;
        memset( &first, 0, sizeof( first ) );
        memset( &second, 0, sizeof( second ) );
        first.type = KF_SHAPE_TRIANGLE;
        second.type = KF_SHAPE_TRIANGLE;
        first.data.triangle.vertices[0] = (kf_vec3_t){0, 0, 0};
        first.data.triangle.vertices[1] = (kf_vec3_t){FIX(2), 0, 0};
        first.data.triangle.vertices[2] = (kf_vec3_t){0, 0, FIX(2)};
        second.data.triangle.vertices[0] = (kf_vec3_t){FIX(2), 0, FIX(2)};
        second.data.triangle.vertices[1] = (kf_vec3_t){FIX(2), 0, FIX(0.5)};
        second.data.triangle.vertices[2] = (kf_vec3_t){FIX(0.5), 0, FIX(2)};
        CHECK( kf_shape_bounds( &first, identity, &firstBounds ) == KF_OK );
        CHECK( kf_shape_bounds( &second, identity, &secondBounds ) == KF_OK );
        CHECK( firstBounds.maximum.x > secondBounds.minimum.x && firstBounds.maximum.z > secondBounds.minimum.z );
        CHECK( kf_shape_contact( &first, identity, &second, identity, &contact ) == KF_NO_HIT );
        CHECK( kf_shape_sweep( &first, identity, (kf_vec3_t){FIX(-1), 0, FIX(-1)}, &second, identity, &hit ) == KF_NO_HIT );
        CHECK( kf_shape_sweep( &first, identity, (kf_vec3_t){FIX(0.5), 0, FIX(0.5)}, &second, identity, &hit ) == KF_OK );
        CHECK( hit.fraction > FIX(0.49) && hit.fraction < FIX(0.51) );
    }
    {
        kf_shape_t rotatedBox;
        kf_shape_t triangle;
        memset( &rotatedBox, 0, sizeof( rotatedBox ) );
        memset( &triangle, 0, sizeof( triangle ) );
        rotatedBox.type = KF_SHAPE_BOX;
        rotatedBox.data.box.half_extents = (kf_vec3_t){FIX(1), FIX(1), FIX(1)};
        rotatedBox.data.box.yaw = 4096;
        triangle.type = KF_SHAPE_TRIANGLE;
        triangle.data.triangle.vertices[0] = (kf_vec3_t){120143, 20739, 206013};
        triangle.data.triangle.vertices[1] = (kf_vec3_t){127023, 27402, 230739};
        triangle.data.triangle.vertices[2] = (kf_vec3_t){23106, -106572, 38679};
        /* Face axes overlap; an edge cross axis separates the tilted surface. */
        CHECK( kf_shape_contact( &rotatedBox, identity, &triangle, identity, &contact ) == KF_NO_HIT );
        CHECK( kf_shape_sweep( &rotatedBox, identity, (kf_vec3_t){FIX(-0.125), 0, FIX(-0.125)}, &triangle, identity, &hit ) == KF_NO_HIT );
    }
    {
        /* Rotations keep the yaw convention x' = x cos + z sin and compose exactly. */
        kf_rotation_t quarter = kf_rotation_from_yaw( KF_ANGLE16_QUARTER );
        kf_rotation_t half = kf_rotation_multiply( quarter, quarter );
        kf_rotation_t expected = kf_rotation_from_yaw( KF_ANGLE16_HALF );
        kf_vec3_t turned = kf_rotation_apply( quarter, (kf_vec3_t){ FIX( 1 ), 0, 0 } );
        kf_vec3_t back = kf_rotation_apply( kf_rotation_inverse( quarter ), turned );
        kf_vec3_t returned = kf_rotation_apply_inverse( quarter, turned );
        CHECK( kf_rotation_valid( quarter ) == KF_TRUE );
        CHECK( kf_rotation_valid( (kf_rotation_t){ KF_ROTATION_SCALE, 1, 0 } ) == KF_FALSE );
        CHECK( turned.x >= -4 && turned.x <= 4 && turned.y == 0 && turned.z >= FIX( -1 ) - 4 && turned.z <= FIX( -1 ) + 4 );
        CHECK( back.x >= FIX( 1 ) - 4 && back.x <= FIX( 1 ) + 4 && back.z >= -4 && back.z <= 4 );
        CHECK( returned.x == back.x && returned.z == back.z );
        CHECK( half.x == 0 && half.z == 0 && half.y >= expected.y - 64 && half.y <= expected.y + 64 );
    }
    {
        /* A yawed box on a floor gets a clipped four point face manifold. */
        kf_shape_t box;
        kf_shape_t floor = make_box( (kf_vec3_t){ FIX( -10 ), FIX( -1 ), FIX( -10 ) }, (kf_vec3_t){ FIX( 10 ), 0, FIX( 10 ) } );
        kf_pose_t pose = { { 0, FIX( 0.5 ) - FIX( 0.01 ), 0 }, { 0, 0, 0 } };
        kf_manifold_t manifold;
        uint32_t index;
        memset( &box, 0, sizeof( box ) );
        box.type = KF_SHAPE_BOX;
        box.data.box.half_extents = (kf_vec3_t){ FIX( 0.5 ), FIX( 0.5 ), FIX( 0.5 ) };
        pose.rotation = kf_rotation_from_yaw( 3000 );
        CHECK( kf_shape_manifold( &box, pose, &floor, identity, FIX( 0.02 ), &manifold ) == KF_OK );
        CHECK( manifold.count == 4 && manifold.normal.y == KF_FIXED_SCALE );
        for( index = 0; index != manifold.count; ++index )
        {
            CHECK( manifold.points[index].separation >= -FIX( 0.01 ) - 8 && manifold.points[index].separation <= -FIX( 0.01 ) + 8 );
            CHECK( manifold.points[index].point_right.y == 0 );
        }
        /* Pitched 45 degrees about X, the box rests on one edge: two points. */
        pose.position.y = (kf_fixed_t)( FIX( 0.7071068 ) - FIX( 0.01 ) );
        pose.rotation = kf_rotation_from_axis_angle( (kf_vec3_t){ FIX( 1 ), 0, 0 }, FIX( 0.7853982 ) );
        CHECK( kf_shape_manifold( &box, pose, &floor, identity, FIX( 0.02 ), &manifold ) == KF_OK );
        CHECK( manifold.count == 2 && manifold.normal.y > FIX( 0.99 ) );
        CHECK( kf_shape_manifold( &box, pose, &floor, identity, 0, &manifold ) == KF_OK );
        CHECK( kf_shape_manifold( &box, pose, &floor, identity, -1, &manifold ) == KF_OUT_OF_RANGE );
    }
    {
        /* A capsule lying on its side touches along its core; tilted, at its lower end. */
        kf_shape_t capsule;
        kf_shape_t floor = make_box( (kf_vec3_t){ FIX( -10 ), FIX( -1 ), FIX( -10 ) }, (kf_vec3_t){ FIX( 10 ), 0, FIX( 10 ) } );
        kf_pose_t pose = { { FIX( 1 ), FIX( 0.24 ), 0 }, { 0, 0, 0 } };
        kf_manifold_t manifold;
        memset( &capsule, 0, sizeof( capsule ) );
        capsule.type = KF_SHAPE_CAPSULE;
        capsule.data.capsule.radius = FIX( 0.25 );
        capsule.data.capsule.height = FIX( 2 );
        pose.rotation = kf_rotation_from_axis_angle( (kf_vec3_t){ 0, 0, FIX( 1 ) }, FIX( 1.5707963 ) );
        CHECK( kf_shape_manifold( &capsule, pose, &floor, identity, FIX( 0.02 ), &manifold ) == KF_OK );
        CHECK( manifold.count == 2 );
        CHECK( manifold.points[0].separation >= -FIX( 0.01 ) - 64 && manifold.points[0].separation <= -FIX( 0.01 ) + 64 );
        CHECK( manifold.points[0].point_left.x != manifold.points[1].point_left.x );
        /* Tilted 0.5 rad, the lower cap touches at x = 1 - 0.25 sin 0.5 when lifted by 0.25 (1 - cos 0.5). */
        pose.rotation = kf_rotation_from_axis_angle( (kf_vec3_t){ 0, 0, FIX( 1 ) }, FIX( 0.5 ) );
        pose.position.y = FIX( 0.0306044 );
        CHECK( kf_shape_manifold( &capsule, pose, &floor, identity, FIX( 0.02 ), &manifold ) == KF_OK );
        CHECK( manifold.count == 1 && manifold.points[0].point_left.y >= -64 && manifold.points[0].point_left.y <= 64 );
        CHECK( manifold.points[0].point_left.x >= FIX( 0.8801 ) - 64 && manifold.points[0].point_left.x <= FIX( 0.8801 ) + 64 );
    }
    puts( "geometry tests passed" );
    return EXIT_SUCCESS;
}
