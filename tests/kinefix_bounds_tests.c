#include "kinefix/kinefix.h"

#include <stdio.h>
#include <string.h>

#define F(Value) kf_fixed_from_ratio( (Value), 1 )
#define R(Numerator, Denominator) kf_fixed_from_ratio( (Numerator), (Denominator) )
#define CHECK(Expression) do { if( (Expression) == 0 ) { fprintf( stderr, "bounds:%d: %s\n", __LINE__, #Expression ); return 1; } } while( 0 )

static kf_world_t * make_world( void )
{
    kf_world_config_t config = kf_world_config_default();
    kf_world_t * world = NULL;
    config.gravity = (kf_vec3_t){0, 0, 0};
    config.sleep_ticks = 0;
    if( kf_world_create( &config, NULL, &world ) != KF_OK )
    {
        return NULL;
    }
    if( kf_world_reserve( world, (kf_capacity_t){1, 4, 0, 0, 0} ) != KF_OK )
    {
        kf_world_destroy( world );
        return NULL;
    }
    return world;
}

static kf_shape_desc_t sphere( uint32_t id, kf_vec3_t center )
{
    kf_shape_desc_t shape = {0};
    shape.id = id;
    shape.shape.type = KF_SHAPE_SPHERE;
    shape.shape.data.sphere.position = center;
    shape.shape.data.sphere.radius = R(1, 4);
    return shape;
}

static kf_bool_t same_state( const kf_body_state_t * left, const kf_body_state_t * right )
{
    uint32_t axis;
    if( left->handle.id != right->handle.id || left->handle.generation != right->handle.generation ) return KF_FALSE;
    if( kf_vec3_equal( left->pose.position, right->pose.position ) == KF_FALSE || memcmp( &left->pose.rotation, &right->pose.rotation, sizeof(left->pose.rotation) ) != 0 ) return KF_FALSE;
    if( kf_vec3_equal( left->target.position, right->target.position ) == KF_FALSE || memcmp( &left->target.rotation, &right->target.rotation, sizeof(left->target.rotation) ) != 0 ) return KF_FALSE;
    if( kf_vec3_equal( left->velocity, right->velocity ) == KF_FALSE || kf_vec3_equal( left->angular_velocity, right->angular_velocity ) == KF_FALSE ) return KF_FALSE;
    if( left->inverse_mass != right->inverse_mass || left->has_target != right->has_target || left->sleep_counter != right->sleep_counter || left->sleeping != right->sleeping ) return KF_FALSE;
    for( axis = 0; axis != 3; ++axis )
    {
        if( left->position_remainder[axis] != right->position_remainder[axis] || left->velocity_remainder[axis] != right->velocity_remainder[axis] ) return KF_FALSE;
    }
    return KF_TRUE;
}

static int check_state( kf_world_t * world, kf_world_t * restored, kf_handle_t handle, const kf_shape_desc_t * shapes, size_t count )
{
    kf_body_state_t state;
    kf_body_state_t restored_state;
    kf_object_snapshot_t objects[1];
    kf_shape_snapshot_t saved_shapes[4];
    kf_snapshot_t saved = {0};
    size_t index;
    CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
    CHECK( state.pose.position.x >= -KF_COORDINATE_LIMIT && state.pose.position.x <= KF_COORDINATE_LIMIT );
    CHECK( state.pose.position.y >= -KF_COORDINATE_LIMIT && state.pose.position.y <= KF_COORDINATE_LIMIT );
    CHECK( state.pose.position.z >= -KF_COORDINATE_LIMIT && state.pose.position.z <= KF_COORDINATE_LIMIT );
    CHECK( kf_rotation_valid( state.pose.rotation ) == KF_TRUE );
    for( index = 0; index != count; ++index )
    {
        kf_aabb_t bounds;
        CHECK( kf_shape_bounds( &shapes[index].shape, state.pose, &bounds ) == KF_OK );
        CHECK( bounds.minimum.x >= -KF_COORDINATE_LIMIT && bounds.maximum.x <= KF_COORDINATE_LIMIT );
        CHECK( bounds.minimum.y >= -KF_COORDINATE_LIMIT && bounds.maximum.y <= KF_COORDINATE_LIMIT );
        CHECK( bounds.minimum.z >= -KF_COORDINATE_LIMIT && bounds.maximum.z <= KF_COORDINATE_LIMIT );
    }
    saved.objects = objects;
    saved.object_capacity = 1;
    saved.shapes = saved_shapes;
    saved.shape_capacity = 4;
    CHECK( kf_world_save( world, &saved ) == KF_OK );
    CHECK( kf_world_restore( restored, &saved ) == KF_OK );
    CHECK( kf_world_get_body( restored, handle, &restored_state ) == KF_OK );
    CHECK( same_state( &state, &restored_state ) == KF_TRUE );
    CHECK( kf_world_step( restored ) == KF_OK );
    return 0;
}

static int test_origin_limits( void )
{
    uint32_t axis;
    int32_t sign;
    for( axis = 0; axis != 3; ++axis )
    {
        for( sign = -1; sign <= 1; sign += 2 )
        {
            kf_world_t * world = make_world();
            kf_world_t * restored = make_world();
            kf_body_desc_t desc = kf_body_desc_default( 1 );
            kf_shape_desc_t shape = sphere( 1, (kf_vec3_t){0, 0, 0} );
            kf_fixed_t * position[3] = {&desc.pose.position.x, &desc.pose.position.y, &desc.pose.position.z};
            kf_fixed_t * velocity[3] = {&desc.velocity.x, &desc.velocity.y, &desc.velocity.z};
            kf_fixed_t * local[3] = {&shape.shape.data.sphere.position.x, &shape.shape.data.sphere.position.y, &shape.shape.data.sphere.position.z};
            kf_handle_t handle;
            kf_body_state_t state;
            uint32_t tick;
            CHECK( world != NULL && restored != NULL );
            desc.type = KF_BODY_DYNAMIC;
            *position[axis] = sign * (KF_COORDINATE_LIMIT - R(1, 2));
            *velocity[axis] = sign * F(100);
            *local[axis] = -sign * F(4000);
            CHECK( kf_world_add_body( world, &desc, &shape, 1, &handle ) == KF_OK );
            for( tick = 0; tick != 8; ++tick )
            {
                CHECK( kf_world_step( world ) == KF_OK );
                CHECK( check_state( world, restored, handle, &shape, 1 ) == 0 );
            }
            CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
            CHECK( kf_vec3_equal( state.velocity, (kf_vec3_t){0, 0, 0} ) == KF_TRUE );
            CHECK( axis != 0 || state.pose.position.x == sign * KF_COORDINATE_LIMIT );
            CHECK( axis != 1 || state.pose.position.y == sign * KF_COORDINATE_LIMIT );
            CHECK( axis != 2 || state.pose.position.z == sign * KF_COORDINATE_LIMIT );
            kf_world_destroy( world );
            kf_world_destroy( restored );
        }
    }
    return 0;
}

static int test_resting_boxes( void )
{
    uint32_t rotated;
    for( rotated = 0; rotated != 2; ++rotated )
    {
        kf_world_t * world = make_world();
        kf_world_t * restored = make_world();
        kf_body_desc_t desc = kf_body_desc_default( 1 );
        kf_shape_desc_t shape = {0};
        kf_handle_t handle;
        uint32_t tick;
        CHECK( world != NULL && restored != NULL );
        desc.type = KF_BODY_DYNAMIC;
        desc.pose.position.y = KF_COORDINATE_LIMIT - (rotated == 0 ? R(1, 10) : F(2));
        if( rotated != 0 )
        {
            desc.pose.rotation = kf_rotation_from_axis_angle( (kf_vec3_t){0, 0, KF_FIXED_SCALE}, R(785398, 1000000) );
        }
        shape.id = 1;
        shape.shape.type = KF_SHAPE_BOX;
        shape.shape.data.box.half_extents = (kf_vec3_t){F(2), R(1, 10), R(1, 10)};
        CHECK( kf_world_add_body( world, &desc, &shape, 1, &handle ) == KF_OK );
        for( tick = 0; tick != 8; ++tick )
        {
            kf_body_state_t state;
            CHECK( kf_world_step( world ) == KF_OK );
            CHECK( check_state( world, restored, handle, &shape, 1 ) == 0 );
            CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
            CHECK( kf_vec3_equal( state.pose.position, desc.pose.position ) == KF_TRUE );
            CHECK( memcmp( &state.pose.rotation, &desc.pose.rotation, sizeof(desc.pose.rotation) ) == 0 );
        }
        kf_world_destroy( world );
        kf_world_destroy( restored );
    }
    return 0;
}

static int test_compound_sliding( void )
{
    kf_world_t * world = make_world();
    kf_world_t * restored = make_world();
    kf_body_desc_t desc = kf_body_desc_default( 1 );
    kf_shape_desc_t shapes[2] = {sphere( 1, (kf_vec3_t){F(3), 0, 0} ), sphere( 2, (kf_vec3_t){-F(2), 0, 0} )};
    kf_handle_t handle;
    kf_body_state_t state;
    uint32_t tick;
    CHECK( world != NULL && restored != NULL );
    desc.type = KF_BODY_DYNAMIC;
    desc.pose.position.x = KF_COORDINATE_LIMIT - R(7, 2);
    desc.velocity = (kf_vec3_t){F(100), F(1), 0};
    CHECK( kf_world_add_body( world, &desc, shapes, 2, &handle ) == KF_OK );
    for( tick = 0; tick != 8; ++tick )
    {
        CHECK( kf_world_step( world ) == KF_OK );
        CHECK( check_state( world, restored, handle, shapes, 2 ) == 0 );
    }
    CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
    CHECK( state.pose.position.x == KF_COORDINATE_LIMIT - R(13, 4) );
    CHECK( state.pose.position.y > 0 );
    CHECK( state.velocity.x == 0 && state.velocity.y == F(1) );
    kf_world_destroy( world );
    kf_world_destroy( restored );
    return 0;
}

static int test_rotated_offset_and_target( void )
{
    uint32_t targeted;
    for( targeted = 0; targeted != 2; ++targeted )
    {
        kf_world_t * world = make_world();
        kf_world_t * restored = make_world();
        kf_body_desc_t desc = kf_body_desc_default( 1 );
        kf_shape_desc_t shapes[2] = {sphere( 1, (kf_vec3_t){F(4000), 0, 0} ), sphere( 2, (kf_vec3_t){F(3998), 0, 0} )};
        kf_pose_t target;
        kf_handle_t handle;
        uint32_t tick;
        CHECK( world != NULL && restored != NULL );
        desc.type = targeted == 0 ? KF_BODY_DYNAMIC : KF_BODY_KINEMATIC;
        desc.pose.rotation = kf_rotation_from_axis_angle( (kf_vec3_t){0, 0, KF_FIXED_SCALE}, R(1570796, 1000000) );
        desc.pose.position.y = R(191, 2);
        desc.velocity.y = F(100);
        CHECK( kf_world_add_body( world, &desc, shapes, 2, &handle ) == KF_OK );
        target = desc.pose;
        target.position.y += R(1, 8);
        if( targeted != 0 )
        {
            CHECK( kf_world_set_kinematic_target( world, handle, target ) == KF_OK );
        }
        for( tick = 0; tick != 8; ++tick )
        {
            CHECK( kf_world_step( world ) == KF_OK );
            CHECK( check_state( world, restored, handle, shapes, 2 ) == 0 );
            if( targeted != 0 && tick == 0 )
            {
                kf_body_state_t state;
                CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
                CHECK( kf_vec3_equal( state.pose.position, target.position ) == KF_TRUE );
            }
        }
        kf_world_destroy( world );
        kf_world_destroy( restored );
    }
    return 0;
}

static int test_compound_rotation_fit( void )
{
    kf_world_t * world = make_world();
    kf_world_t * restored = make_world();
    kf_body_desc_t desc = kf_body_desc_default( 1 );
    kf_shape_desc_t shapes[2] = {sphere( 1, (kf_vec3_t){F(4000), F(4000), 0} ), sphere( 2, (kf_vec3_t){-F(4000), -F(4000), 0} )};
    kf_handle_t handle;
    uint32_t tick;
    CHECK( world != NULL && restored != NULL );
    desc.type = KF_BODY_DYNAMIC;
    desc.angular_velocity.z = F(1);
    CHECK( kf_world_add_body( world, &desc, shapes, 2, &handle ) == KF_OK );
    for( tick = 0; tick != 8; ++tick )
    {
        CHECK( kf_world_step( world ) == KF_OK );
        CHECK( check_state( world, restored, handle, shapes, 2 ) == 0 );
    }
    kf_world_destroy( world );
    kf_world_destroy( restored );
    return 0;
}

static int test_rotating_offset_roundtrips( void )
{
    uint32_t variant;
    for( variant = 0; variant != 8; ++variant )
    {
        kf_world_t * world = make_world();
        kf_world_t * restored = make_world();
        kf_body_desc_t desc = kf_body_desc_default( 1 );
        kf_shape_desc_t shape = sphere( 1, (kf_vec3_t){F(4000), 0, 0} );
        kf_aabb_t initial_bounds;
        kf_handle_t handle;
        uint32_t tick;
        CHECK( world != NULL && restored != NULL );
        desc.type = KF_BODY_DYNAMIC;
        desc.pose.rotation = kf_rotation_from_axis_angle( (kf_vec3_t){F(1), F(2), F(-3)}, R(variant * 7, 8) );
        CHECK( kf_shape_bounds( &shape.shape, desc.pose, &initial_bounds ) == KF_OK );
        desc.pose.position.x = -(kf_fixed_t)(((int64_t)initial_bounds.minimum.x + initial_bounds.maximum.x) / 2);
        desc.pose.position.y = -(kf_fixed_t)(((int64_t)initial_bounds.minimum.y + initial_bounds.maximum.y) / 2);
        desc.pose.position.z = -(kf_fixed_t)(((int64_t)initial_bounds.minimum.z + initial_bounds.maximum.z) / 2);
        desc.velocity = (kf_vec3_t){-F(20), F(10), F(30)};
        desc.angular_velocity = (kf_vec3_t){R(5, 2), R(7, 3), R(-11, 4)};
        CHECK( kf_world_add_body( world, &desc, &shape, 1, &handle ) == KF_OK );
        for( tick = 0; tick != 64; ++tick )
        {
            CHECK( kf_world_step( world ) == KF_OK );
            CHECK( check_state( world, restored, handle, &shape, 1 ) == 0 );
        }
        kf_world_destroy( world );
        kf_world_destroy( restored );
    }
    return 0;
}

static int test_boundary_rotation_with_ccd_candidate( void )
{
    kf_world_config_t config = kf_world_config_default();
    kf_world_t * world = NULL;
    kf_world_t * restored = NULL;
    kf_body_desc_t desc = kf_body_desc_default( 1 );
    kf_body_desc_t obstacle = kf_body_desc_default( 2 );
    kf_shape_desc_t box = {0};
    kf_shape_desc_t candidate = sphere( 1, (kf_vec3_t){0, 0, 0} );
    kf_handle_t handle;
    kf_handle_t other;
    kf_object_snapshot_t objects[2];
    kf_shape_snapshot_t shapes[2];
    kf_contact_snapshot_t contacts[16];
    kf_snapshot_t saved = {0};
    uint32_t tick;
    config.gravity = (kf_vec3_t){0, 0, 0};
    config.sleep_ticks = 0;
    config.contact_slop = R(1, 1000);
    CHECK( kf_world_create( &config, NULL, &world ) == KF_OK );
    CHECK( kf_world_create( &config, NULL, &restored ) == KF_OK );
    CHECK( kf_world_reserve( world, (kf_capacity_t){2, 2, 16, 0, 0} ) == KF_OK );
    CHECK( kf_world_reserve( restored, (kf_capacity_t){2, 2, 16, 0, 0} ) == KF_OK );
    desc.type = KF_BODY_DYNAMIC;
    desc.pose.position.y = KF_COORDINATE_LIMIT - R(3, 10);
    desc.angular_velocity.z = F(1);
    box.id = 1;
    box.shape.type = KF_SHAPE_BOX;
    box.shape.data.box.half_extents = (kf_vec3_t){F(2), R(1, 10), R(1, 10)};
    candidate.shape.data.sphere.radius = R(1, 1000);
    /* This shape is close enough for CCD broadphase, but is separated in Z. */
    obstacle.pose.position = (kf_vec3_t){F(1), KF_COORDINATE_LIMIT - R(1, 10), R(11, 100)};
    CHECK( kf_world_add_body( world, &desc, &box, 1, &handle ) == KF_OK );
    CHECK( kf_world_add_body( world, &obstacle, &candidate, 1, &other ) == KF_OK );
    saved.objects = objects;
    saved.object_capacity = 2;
    saved.shapes = shapes;
    saved.shape_capacity = 2;
    saved.contacts = contacts;
    saved.contact_capacity = 16;
    for( tick = 0; tick != 64; ++tick )
    {
        kf_body_state_t state;
        kf_aabb_t bounds;
        CHECK( kf_world_step( world ) == KF_OK );
        CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
        CHECK( kf_shape_bounds( &box.shape, state.pose, &bounds ) == KF_OK );
        CHECK( kf_world_save( world, &saved ) == KF_OK );
        CHECK( kf_world_restore( restored, &saved ) == KF_OK );
        CHECK( kf_world_step( restored ) == KF_OK );
    }
    kf_world_destroy( world );
    kf_world_destroy( restored );
    return 0;
}

static int test_extreme_compound_full_turns( void )
{
    kf_world_config_t config = kf_world_config_default();
    kf_world_t * world = NULL;
    kf_world_t * restored = NULL;
    kf_shape_desc_t shapes[2][2] = {{{0}, {0}}, {{0}, {0}}};
    kf_handle_t handles[2];
    kf_object_snapshot_t objects[2];
    kf_shape_snapshot_t saved_shapes[4];
    kf_contact_snapshot_t contacts[64];
    kf_snapshot_t saved = {0};
    uint32_t index;
    uint32_t tick;
    config.gravity = (kf_vec3_t){0, 0, 0};
    config.sleep_ticks = 0;
    config.substeps = 1;
    CHECK( kf_world_create( &config, NULL, &world ) == KF_OK );
    CHECK( kf_world_create( &config, NULL, &restored ) == KF_OK );
    CHECK( kf_world_reserve( world, (kf_capacity_t){2, 4, 64, 0, 0} ) == KF_OK );
    CHECK( kf_world_reserve( restored, (kf_capacity_t){2, 4, 64, 0, 0} ) == KF_OK );
    for( index = 0; index != 2; ++index )
    {
        int32_t sign = index == 0 ? 1 : -1;
        kf_body_desc_t desc = kf_body_desc_default( index + 1 );
        desc.type = KF_BODY_DYNAMIC;
        desc.angular_velocity = (kf_vec3_t){0, R(-111072073, 1000000), R(111072073, 1000000)};
        shapes[index][0] = sphere( 1, (kf_vec3_t){sign * F(3583), sign * F(3583), sign * F(3583)} );
        shapes[index][0].shape.data.sphere.radius = KF_SIZE_LIMIT;
        shapes[index][1] = sphere( 2, (kf_vec3_t){-sign * F(4095), -sign * F(4095), -sign * F(4095)} );
        shapes[index][1].shape.data.sphere.radius = R(1, 1000);
        CHECK( kf_world_add_body( world, &desc, shapes[index], 2, handles + index ) == KF_OK );
    }
    saved.objects = objects;
    saved.object_capacity = 2;
    saved.shapes = saved_shapes;
    saved.shape_capacity = 4;
    saved.contacts = contacts;
    saved.contact_capacity = 64;
    for( tick = 0; tick != 4; ++tick )
    {
        CHECK( kf_world_step( world ) == KF_OK );
        for( index = 0; index != 2; ++index )
        {
            kf_body_state_t state;
            size_t shape_index;
            CHECK( kf_world_get_body( world, handles[index], &state ) == KF_OK );
            CHECK( state.pose.position.x >= -KF_COORDINATE_LIMIT && state.pose.position.x <= KF_COORDINATE_LIMIT );
            CHECK( state.pose.position.y >= -KF_COORDINATE_LIMIT && state.pose.position.y <= KF_COORDINATE_LIMIT );
            CHECK( state.pose.position.z >= -KF_COORDINATE_LIMIT && state.pose.position.z <= KF_COORDINATE_LIMIT );
            for( shape_index = 0; shape_index != 2; ++shape_index )
            {
                kf_aabb_t bounds;
                CHECK( kf_shape_bounds( &shapes[index][shape_index].shape, state.pose, &bounds ) == KF_OK );
            }
        }
        CHECK( kf_world_save( world, &saved ) == KF_OK );
        CHECK( kf_world_restore( restored, &saved ) == KF_OK );
        CHECK( kf_world_step( restored ) == KF_OK );
    }
    kf_world_destroy( world );
    kf_world_destroy( restored );
    return 0;
}

int main( void )
{
    CHECK( test_origin_limits() == 0 );
    CHECK( test_resting_boxes() == 0 );
    CHECK( test_compound_sliding() == 0 );
    CHECK( test_rotated_offset_and_target() == 0 );
    CHECK( test_compound_rotation_fit() == 0 );
    CHECK( test_rotating_offset_roundtrips() == 0 );
    CHECK( test_boundary_rotation_with_ccd_candidate() == 0 );
    CHECK( test_extreme_compound_full_turns() == 0 );
    puts( "kinefix bounds tests passed" );
    return 0;
}
