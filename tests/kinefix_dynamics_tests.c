#include "kinefix/kinefix.h"
#include "../src/kinefix_internal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define F(Value) kf_fixed_from_ratio( (Value), 1 )
#define R(Numerator, Denominator) kf_fixed_from_ratio( (Numerator), (Denominator) )
static int failures;
#define CHECK(Expression) do { if( (Expression) == 0 ) { fprintf( stderr, "dynamics:%d: %s\n", __LINE__, #Expression ); ++failures; } } while( 0 )

static kf_shape_desc_t sphere_shape( kf_fixed_t radius )
{
    kf_shape_desc_t shape = {0};
    shape.id = 1;
    shape.shape.type = KF_SHAPE_SPHERE;
    shape.shape.data.sphere.radius = radius;
    return shape;
}

static kf_world_t * make_world( kf_world_config_t config )
{
    kf_world_t * world = NULL;
    CHECK( kf_world_create( &config, NULL, &world ) == KF_OK );
    if( world == NULL ) return NULL;
    CHECK( kf_world_reserve( world, (kf_capacity_t){32, 32, 32, 0, 0} ) == KF_OK );
    return world;
}

static kf_handle_t add_sphere( kf_world_t * world, uint32_t id, kf_body_type_t type, kf_vec3_t position, kf_vec3_t velocity, kf_fixed_t restitution )
{
    kf_body_desc_t desc = kf_body_desc_default( id );
    kf_shape_desc_t shape = sphere_shape( R(1, 4) );
    kf_handle_t handle = {0};
    desc.type = type;
    desc.pose.position = position;
    desc.velocity = velocity;
    desc.restitution = restitution;
    desc.friction = 0;
    CHECK( kf_world_add_body( world, &desc, &shape, 1, &handle ) == KF_OK );
    return handle;
}

static kf_bool_t same_pose( kf_pose_t left, kf_pose_t right )
{
    return (kf_bool_t)(kf_vec3_equal( left.position, right.position ) == KF_TRUE && left.rotation.x == right.rotation.x && left.rotation.y == right.rotation.y && left.rotation.z == right.rotation.z);
}

/* Fieldwise, so struct padding never takes part in the comparison. */
static kf_bool_t same_body_state( const kf_body_state_t * left, const kf_body_state_t * right )
{
    uint32_t axis;
    if( left->handle.id != right->handle.id || left->handle.generation != right->handle.generation ) return KF_FALSE;
    if( same_pose( left->pose, right->pose ) == KF_FALSE || same_pose( left->target, right->target ) == KF_FALSE ) return KF_FALSE;
    if( kf_vec3_equal( left->velocity, right->velocity ) == KF_FALSE || kf_vec3_equal( left->angular_velocity, right->angular_velocity ) == KF_FALSE ) return KF_FALSE;
    for( axis = 0; axis != 3u; ++axis )
    {
        if( left->position_remainder[axis] != right->position_remainder[axis] || left->velocity_remainder[axis] != right->velocity_remainder[axis] ) return KF_FALSE;
    }
    return (kf_bool_t)(left->inverse_mass == right->inverse_mass && left->has_target == right->has_target && left->sleep_counter == right->sleep_counter && left->sleeping == right->sleeping);
}

static kf_bool_t near( kf_fixed_t left, kf_fixed_t right, kf_fixed_t tolerance )
{
    int64_t difference = (int64_t)left - right;
    return (kf_bool_t)(difference >= -tolerance && difference <= tolerance);
}

static void test_fast_bounce( void )
{
    kf_world_config_t config = kf_world_config_default();
    kf_world_t * world;
    kf_body_desc_t wall = kf_body_desc_default( 2 );
    kf_shape_desc_t wall_shape = {0};
    kf_handle_t wall_handle;
    kf_handle_t projectile;
    kf_body_state_t state;
    config.gravity = (kf_vec3_t){0, 0, 0};
    config.ccd_iterations = 1;
    world = make_world( config );
    if( world == NULL ) return;
    wall_shape.id = 1;
    wall_shape.shape.type = KF_SHAPE_AABB;
    wall_shape.shape.data.aabb = (kf_aabb_t){{0, -F(4), -F(4)}, {R(1, 100), F(4), F(4)}};
    wall.restitution = KF_FIXED_SCALE;
    CHECK( kf_world_add_body( world, &wall, &wall_shape, 1, &wall_handle ) == KF_OK );
    projectile = add_sphere( world, 1, KF_BODY_DYNAMIC, (kf_vec3_t){-F(3), 0, 0}, (kf_vec3_t){F(100), 0, 0}, KF_FIXED_SCALE );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_get_body( world, projectile, &state ) == KF_OK );
    CHECK( near( state.pose.position.x, R(-3, 2), R(1, 100) ) == KF_TRUE );
    CHECK( near( state.velocity.x, -F(100), R(1, 100) ) == KF_TRUE );
    CHECK( kf_world_tick( world ) == 1 );
    kf_world_destroy( world );
}

static void test_relative_dynamic_motion( void )
{
    kf_world_config_t config = kf_world_config_default();
    kf_world_t * world;
    kf_handle_t left;
    kf_handle_t right;
    kf_body_state_t state;
    config.gravity = (kf_vec3_t){0, 0, 0};
    world = make_world( config );
    if( world == NULL ) return;
    left = add_sphere( world, 4, KF_BODY_DYNAMIC, (kf_vec3_t){-F(3), 0, 0}, (kf_vec3_t){F(100), 0, 0}, KF_FIXED_SCALE );
    right = add_sphere( world, 7, KF_BODY_DYNAMIC, (kf_vec3_t){F(3), 0, 0}, (kf_vec3_t){-F(100), 0, 0}, KF_FIXED_SCALE );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_get_body( world, left, &state ) == KF_OK );
    CHECK( near( state.pose.position.x, R(-3, 2), R(1, 100) ) == KF_TRUE );
    CHECK( near( state.velocity.x, -F(100), R(1, 100) ) == KF_TRUE );
    CHECK( kf_world_get_body( world, right, &state ) == KF_OK );
    CHECK( near( state.pose.position.x, R(3, 2), R(1, 100) ) == KF_TRUE );
    CHECK( near( state.velocity.x, F(100), R(1, 100) ) == KF_TRUE );
    kf_world_destroy( world );
}

static void test_rational_time_sleep_and_impulse( void )
{
    kf_world_config_t config = kf_world_config_default();
    kf_world_t * world;
    kf_handle_t handle;
    kf_body_state_t state;
    unsigned index;
    config.gravity = (kf_vec3_t){0, 0, 0};
    config.sleep_ticks = 0;
    world = make_world( config );
    if( world == NULL ) return;
    handle = add_sphere( world, 1, KF_BODY_DYNAMIC, (kf_vec3_t){0, 0, 0}, (kf_vec3_t){1, 0, 0}, 0 );
    for( index = 0; index != 25; ++index ) CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
    CHECK( state.pose.position.x == 1 );
    CHECK( state.position_remainder[0] == 0 );
    kf_world_destroy( world );

    config.sleep_ticks = 3;
    world = make_world( config );
    if( world == NULL ) return;
    handle = add_sphere( world, 1, KF_BODY_DYNAMIC, (kf_vec3_t){0, 0, 0}, (kf_vec3_t){0, 0, 0}, 0 );
    for( index = 0; index != 3; ++index ) CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
    CHECK( state.sleeping == KF_TRUE );
    CHECK( kf_world_apply_impulse( world, handle, (kf_vec3_t){F(2), 0, 0} ) == KF_OK );
    CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
    CHECK( state.sleeping == KF_FALSE );
    CHECK( state.velocity.x == F(2) );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
    CHECK( state.pose.position.x > 0 );
    kf_world_destroy( world );
}

static void test_chain_and_lifecycle( void )
{
    kf_world_config_t config = kf_world_config_default();
    kf_world_t * world;
    kf_handle_t handles[5];
    kf_handle_t joints[4];
    kf_constraint_state_t joint_state;
    unsigned index;
    config.solver_iterations = 32;
    world = make_world( config );
    if( world == NULL ) return;
    for( index = 0; index != 5; ++index )
    {
        handles[index] = add_sphere( world, index + 1, index == 0 ? KF_BODY_STATIC : KF_BODY_DYNAMIC, (kf_vec3_t){(kf_fixed_t)(index * KF_FIXED_SCALE), F(4), 0}, (kf_vec3_t){0, 0, 0}, 0 );
    }
    for( index = 0; index != 4; ++index )
    {
        kf_constraint_desc_t desc = {0};
        desc.id = index + 1;
        desc.left = handles[index];
        desc.right = handles[index + 1];
        desc.length = KF_FIXED_SCALE;
        CHECK( kf_world_add_constraint( world, &desc, joints + index ) == KF_OK );
    }
    for( index = 0; index != 80; ++index ) CHECK( kf_world_step( world ) == KF_OK );
    for( index = 0; index != 4; ++index )
    {
        kf_body_state_t left;
        kf_body_state_t right;
        kf_vec3_t delta;
        kf_fixed_t length;
        CHECK( kf_world_get_body( world, handles[index], &left ) == KF_OK );
        CHECK( kf_world_get_body( world, handles[index + 1], &right ) == KF_OK );
        delta = kf_vec3_sub( left.pose.position, right.pose.position );
        length = kf_vec3_length( delta );
        CHECK( near( length, KF_FIXED_SCALE, R(1, 100) ) == KF_TRUE );
    }
    CHECK( kf_world_remove_body( world, handles[4] ) == KF_OK );
    CHECK( kf_world_get_constraint( world, joints[3], &joint_state ) == KF_STALE_HANDLE );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_remove_constraint( world, joints[0] ) == KF_OK );
    CHECK( kf_world_get_constraint( world, joints[0], &joint_state ) == KF_STALE_HANDLE );
    kf_world_destroy( world );
}

static void test_max_distance_and_generation( void )
{
    kf_world_config_t config = kf_world_config_default();
    kf_world_t * world;
    kf_handle_t left;
    kf_handle_t right;
    kf_handle_t joint;
    kf_handle_t old_joint;
    kf_constraint_desc_t desc = {0};
    kf_constraint_state_t state;
    kf_body_state_t body;
    config.gravity = (kf_vec3_t){0, 0, 0};
    world = make_world( config );
    if( world == NULL ) return;
    left = add_sphere( world, 1, KF_BODY_STATIC, (kf_vec3_t){0, 0, 0}, (kf_vec3_t){0, 0, 0}, 0 );
    right = add_sphere( world, 2, KF_BODY_DYNAMIC, (kf_vec3_t){F(1), 0, 0}, (kf_vec3_t){F(100), 0, 0}, 0 );
    desc.id = 3;
    desc.type = KF_CONSTRAINT_MAX_DISTANCE;
    desc.left = left;
    desc.right = right;
    desc.length = F(2);
    CHECK( kf_world_add_constraint( world, &desc, &joint ) == KF_OK );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_get_body( world, right, &body ) == KF_OK );
    CHECK( near( body.pose.position.x, F(2), R(1, 1000) ) == KF_TRUE );
    CHECK( body.velocity.x == 0 );
    CHECK( kf_world_get_constraint( world, joint, &state ) == KF_OK );
    CHECK( kf_world_set_constraint( world, &state ) == KF_OK );
    old_joint = joint;
    CHECK( kf_world_remove_constraint( world, joint ) == KF_OK );
    CHECK( kf_world_add_constraint( world, &desc, &joint ) == KF_OK );
    CHECK( joint.generation != old_joint.generation );
    CHECK( kf_world_remove_constraint( world, old_joint ) == KF_STALE_HANDLE );
    kf_world_destroy( world );
}

typedef struct allocation_count_t { size_t allocations; size_t frees; } allocation_count_t;
static void * count_allocate( void * context, size_t bytes )
{
    allocation_count_t * count = context;
    ++count->allocations;
    return malloc( bytes );
}
static void count_free( void * context, void * pointer )
{
    allocation_count_t * count = context;
    if( pointer != NULL ) ++count->frees;
    free( pointer );
}

static void test_no_allocations_and_rollback( void )
{
    kf_world_config_t config = kf_world_config_default();
    kf_world_t * world = NULL;
    allocation_count_t count = {0};
    kf_allocator_t allocator = {count_allocate, count_free, &count};
    kf_handle_t handle;
    kf_body_state_t before;
    kf_body_state_t after;
    size_t allocations;
    unsigned index;
    config.gravity = (kf_vec3_t){0, 0, 0};
    CHECK( kf_world_create( &config, &allocator, &world ) == KF_OK );
    if( world == NULL ) return;
    CHECK( kf_world_reserve( world, (kf_capacity_t){4, 4, 4, 0, 0} ) == KF_OK );
    handle = add_sphere( world, 1, KF_BODY_DYNAMIC, (kf_vec3_t){0, 0, 0}, (kf_vec3_t){F(1), 0, 0}, 0 );
    allocations = count.allocations;
    for( index = 0; index != 100; ++index ) CHECK( kf_world_step( world ) == KF_OK );
    CHECK( count.allocations == allocations );
    kf_world_destroy( world );
    CHECK( count.allocations == count.frees );

    /* Physical limits are enforced per body: the step still advances. */
    config.gravity.x = KF_SPEED_LIMIT;
    world = make_world( config );
    if( world == NULL ) return;
    handle = add_sphere( world, 1, KF_BODY_DYNAMIC, (kf_vec3_t){0, 0, 0}, (kf_vec3_t){KF_SPEED_LIMIT, 0, 0}, 0 );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_tick( world ) == 1 );
    CHECK( kf_world_get_body( world, handle, &after ) == KF_OK );
    CHECK( after.velocity.x == KF_SPEED_LIMIT );
    kf_world_destroy( world );

    /* Contact capacity is the remaining transactional failure; reserving more resumes. */
    config.gravity = (kf_vec3_t){0, -F(20), 0};
    CHECK( kf_world_create( &config, NULL, &world ) == KF_OK );
    if( world == NULL ) return;
    CHECK( kf_world_reserve( world, (kf_capacity_t){4, 4, 0, 1, 0} ) == KF_OK );
    (void)add_sphere( world, 1, KF_BODY_DYNAMIC, (kf_vec3_t){0, R(1, 4), 0}, (kf_vec3_t){0, 0, 0}, 0 );
    handle = add_sphere( world, 2, KF_BODY_DYNAMIC, (kf_vec3_t){0, R(3, 4), 0}, (kf_vec3_t){0, 0, 0}, 0 );
    (void)add_sphere( world, 3, KF_BODY_STATIC, (kf_vec3_t){0, -R(1, 4), 0}, (kf_vec3_t){0, 0, 0}, 0 );
    CHECK( kf_world_get_body( world, handle, &before ) == KF_OK );
    CHECK( kf_world_step( world ) == KF_CAPACITY_EXCEEDED );
    CHECK( kf_world_tick( world ) == 0 );
    CHECK( kf_world_get_body( world, handle, &after ) == KF_OK );
    CHECK( same_body_state( &before, &after ) == KF_TRUE );
    CHECK( kf_world_reserve( world, (kf_capacity_t){4, 4, 0, 8, 0} ) == KF_OK );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_tick( world ) == 1 );
    kf_world_destroy( world );
}

static void test_mass_damping_yaw_and_friction( void )
{
    kf_world_config_t config = kf_world_config_default();
    kf_world_t * world;
    kf_handle_t handle;
    kf_body_desc_t desc = kf_body_desc_default( 1 );
    kf_shape_desc_t shape = sphere_shape( R(1, 4) );
    kf_body_state_t state;
    config.gravity = (kf_vec3_t){0, 0, 0};
    world = make_world( config );
    if( world == NULL ) return;
    desc.type = KF_BODY_DYNAMIC;
    desc.mass = F(2);
    desc.linear_damping = KF_FIXED_SCALE;
    desc.angular_damping = R(1, 2);
    desc.angular_velocity.y = KF_FIXED_PI;
    CHECK( kf_world_add_body( world, &desc, &shape, 1, &handle ) == KF_OK );
    CHECK( kf_world_apply_impulse( world, handle, (kf_vec3_t){F(4), 0, 0} ) == KF_OK );
    CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
    CHECK( state.velocity.x == F(2) );
    CHECK( state.inverse_mass == R(1, 2) );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
    /* Implicit damping per substep: 2 / (1 + 1/100)^4. */
    CHECK( near( state.velocity.x, 125958, 8 ) == KF_TRUE );
    CHECK( state.angular_velocity.y < KF_FIXED_PI );
    CHECK( state.pose.rotation.y > 0 );
    kf_world_destroy( world );

    config.gravity.y = -F(20);
    world = make_world( config );
    if( world == NULL ) return;
    {
        kf_body_desc_t floor = kf_body_desc_default( 2 );
        kf_shape_desc_t floor_shape = {0};
        kf_handle_t floor_handle;
        floor_shape.id = 1;
        floor_shape.shape.type = KF_SHAPE_AABB;
        floor_shape.shape.data.aabb = (kf_aabb_t){{-F(10), -F(1), -F(10)}, {F(10), 0, F(10)}};
        floor.friction = KF_FIXED_SCALE;
        CHECK( kf_world_add_body( world, &floor, &floor_shape, 1, &floor_handle ) == KF_OK );
        desc = kf_body_desc_default( 1 );
        desc.type = KF_BODY_DYNAMIC;
        desc.pose.position.y = R(1, 4);
        desc.velocity.x = F(2);
        desc.friction = KF_FIXED_SCALE;
        CHECK( kf_world_add_body( world, &desc, &shape, 1, &handle ) == KF_OK );
        CHECK( kf_world_step( world ) == KF_OK );
        CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
        CHECK( state.velocity.x < F(2) );
        CHECK( state.pose.position.y >= R(1, 4) - config.contact_slop );
    }
    kf_world_destroy( world );
}

static void test_sleeping_body_loses_support( void )
{
    kf_world_config_t config = kf_world_config_default();
    kf_world_t * world;
    kf_handle_t floor_handle;
    kf_handle_t body;
    kf_body_desc_t floor = kf_body_desc_default( 2 );
    kf_shape_desc_t shape = {0};
    kf_body_state_t state;
    unsigned index;
    config.sleep_ticks = 3;
    world = make_world( config );
    if( world == NULL ) return;
    shape.id = 1;
    shape.shape.type = KF_SHAPE_AABB;
    shape.shape.data.aabb = (kf_aabb_t){{-F(4), -F(1), -F(4)}, {F(4), 0, F(4)}};
    CHECK( kf_world_add_body( world, &floor, &shape, 1, &floor_handle ) == KF_OK );
    body = add_sphere( world, 1, KF_BODY_DYNAMIC, (kf_vec3_t){0, R(1, 4), 0}, (kf_vec3_t){0, 0, 0}, 0 );
    for( index = 0; index != 3; ++index ) CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_get_body( world, body, &state ) == KF_OK );
    CHECK( state.sleeping == KF_TRUE );
    CHECK( kf_world_remove_body( world, floor_handle ) == KF_OK );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_get_body( world, body, &state ) == KF_OK );
    CHECK( state.sleeping == KF_FALSE );
    CHECK( state.velocity.y < 0 );
    CHECK( state.pose.position.y < R(1, 4) );
    kf_world_destroy( world );
}

static void test_static_velocity_is_ignored( void )
{
    kf_world_config_t config = kf_world_config_default();
    kf_world_t * world;
    kf_body_desc_t desc = kf_body_desc_default( 2 );
    kf_shape_desc_t shape = {0};
    kf_handle_t wall;
    kf_handle_t body;
    kf_body_state_t state;
    config.gravity = (kf_vec3_t){0, 0, 0};
    world = make_world( config );
    if( world == NULL ) return;
    desc.velocity.x = F(100);
    desc.angular_velocity.y = F(100);
    shape.id = 1;
    shape.shape.type = KF_SHAPE_AABB;
    shape.shape.data.aabb = (kf_aabb_t){{0, -F(2), -F(2)}, {R(1, 100), F(2), F(2)}};
    CHECK( kf_world_add_body( world, &desc, &shape, 1, &wall ) == KF_OK );
    body = add_sphere( world, 1, KF_BODY_DYNAMIC, (kf_vec3_t){-F(3), 0, 0}, (kf_vec3_t){F(100), 0, 0}, KF_FIXED_SCALE );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_get_body( world, body, &state ) == KF_OK );
    CHECK( near( state.velocity.x, -F(100), R(1, 100) ) == KF_TRUE );
    CHECK( kf_world_get_body( world, wall, &state ) == KF_OK );
    CHECK( state.pose.position.x == 0 );
    CHECK( state.pose.rotation.x == 0 && state.pose.rotation.y == 0 && state.pose.rotation.z == 0 );
    kf_world_destroy( world );
}

static void test_near_equal_impact_order( void )
{
    kf_world_config_t config = kf_world_config_default();
    kf_world_t * world;
    kf_body_desc_t far_wall = kf_body_desc_default( 2 );
    kf_body_desc_t near_wall = kf_body_desc_default( 3 );
    kf_shape_desc_t wall_shape = {0};
    kf_shape_desc_t projectile_shape = sphere_shape( R(1, 4) );
    kf_handle_t handle;
    kf_handle_t projectile;
    kf_body_state_t state;
    kf_sweep_hit_t near_hit;
    kf_sweep_hit_t far_hit;
    kf_pose_t projectile_pose = {{-F(20), 0, 0}, {0, 0, 0}};
    kf_vec3_t displacement = {KF_SPEED_LIMIT / 25, 0, 0};
    config.gravity = (kf_vec3_t){0, 0, 0};
    world = make_world( config );
    if( world == NULL ) return;
    wall_shape.id = 1;
    wall_shape.shape.type = KF_SHAPE_AABB;
    wall_shape.shape.data.aabb = (kf_aabb_t){{0, -F(4), -F(4)}, {R(1, 100), F(4), F(4)}};
    far_wall.pose.position.x = 20; /* 0.000305 units: distinct TOIs within one Q16 bin. */
    far_wall.friction = 0;
    near_wall.friction = 0;
    near_wall.restitution = KF_FIXED_SCALE;
    CHECK( kf_shape_sweep( &projectile_shape.shape, projectile_pose, displacement, &wall_shape.shape, near_wall.pose, &near_hit ) == KF_OK );
    CHECK( kf_shape_sweep( &projectile_shape.shape, projectile_pose, displacement, &wall_shape.shape, far_wall.pose, &far_hit ) == KF_OK );
    CHECK( near_hit.fraction == far_hit.fraction );
    CHECK( near_hit.fraction_q32 < far_hit.fraction_q32 );
    CHECK( kf_world_add_body( world, &far_wall, &wall_shape, 1, &handle ) == KF_OK );
    CHECK( kf_world_add_body( world, &near_wall, &wall_shape, 1, &handle ) == KF_OK );
    projectile = add_sphere( world, 1, KF_BODY_DYNAMIC, projectile_pose.position, (kf_vec3_t){KF_SPEED_LIMIT, 0, 0}, 0 );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_get_body( world, projectile, &state ) == KF_OK );
    CHECK( near( state.velocity.x, -KF_SPEED_LIMIT, R(1, 100) ) == KF_TRUE );
    CHECK( state.pose.position.x < -F(20) );
    kf_world_destroy( world );
}

static void test_relative_motion_near_world_boundary( void )
{
    kf_world_config_t config = kf_world_config_default();
    kf_world_t * world;
    kf_handle_t dynamic;
    kf_body_state_t state;
    config.gravity = (kf_vec3_t){0, 0, 0};
    config.step = (kf_timestep_t){1, 1};
    world = make_world( config );
    if( world == NULL ) return;
    dynamic = add_sphere( world, 1, KF_BODY_DYNAMIC, (kf_vec3_t){F(4000), 0, 0}, (kf_vec3_t){-F(100), 0, 0}, 0 );
    (void)add_sphere( world, 2, KF_BODY_KINEMATIC, (kf_vec3_t){F(4010), 0, 0}, (kf_vec3_t){-F(1000), 0, 0}, 0 );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_get_body( world, dynamic, &state ) == KF_OK );
    CHECK( near( state.velocity.x, -F(1000), R(1, 1000) ) == KF_TRUE );
    CHECK( near( state.pose.position.x, R(6019, 2), R(1, 1000) ) == KF_TRUE );
    kf_world_destroy( world );
}

static void test_order_independence( void )
{
    kf_world_config_t config = kf_world_config_default();
    kf_world_t * first;
    kf_world_t * second;
    kf_handle_t first_handles[3];
    kf_handle_t second_handles[3];
    kf_vec3_t positions[3] = {{-F(3), 0, 0}, {0, 0, 0}, {F(3), 0, 0}};
    kf_vec3_t velocities[3] = {{F(50), 0, 0}, {0, 0, 0}, {-F(50), 0, 0}};
    unsigned index;
    config.gravity = (kf_vec3_t){0, 0, 0};
    first = make_world( config );
    second = make_world( config );
    if( first == NULL || second == NULL ) { kf_world_destroy( first ); kf_world_destroy( second ); return; }
    for( index = 0; index != 3; ++index ) first_handles[index] = add_sphere( first, index + 1, KF_BODY_DYNAMIC, positions[index], velocities[index], R(1, 2) );
    for( index = 3; index != 0; --index ) second_handles[index - 1] = add_sphere( second, index, KF_BODY_DYNAMIC, positions[index - 1], velocities[index - 1], R(1, 2) );
    for( index = 0; index != 12; ++index )
    {
        CHECK( kf_world_step( first ) == KF_OK );
        CHECK( kf_world_step( second ) == KF_OK );
    }
    for( index = 0; index != 3; ++index )
    {
        kf_body_state_t first_state;
        kf_body_state_t second_state;
        CHECK( kf_world_get_body( first, first_handles[index], &first_state ) == KF_OK );
        CHECK( kf_world_get_body( second, second_handles[index], &second_state ) == KF_OK );
        CHECK( same_body_state( &first_state, &second_state ) == KF_TRUE );
    }
    kf_world_destroy( first );
    kf_world_destroy( second );
}

static kf_world_t * make_large_world( kf_world_config_t config )
{
    kf_world_t * world = NULL;
    CHECK( kf_world_create( &config, NULL, &world ) == KF_OK );
    if( world == NULL ) return NULL;
    CHECK( kf_world_reserve( world, (kf_capacity_t){128, 128, 8, 0, 0} ) == KF_OK );
    return world;
}

static kf_handle_t add_floor( kf_world_t * world, uint32_t id )
{
    kf_body_desc_t desc = kf_body_desc_default( id );
    kf_shape_desc_t shape = {0};
    kf_handle_t handle = {0};
    shape.id = 1;
    shape.shape.type = KF_SHAPE_AABB;
    shape.shape.data.aabb = (kf_aabb_t){{-F(100), -F(1), -F(100)}, {F(100), 0, F(100)}};
    CHECK( kf_world_add_body( world, &desc, &shape, 1, &handle ) == KF_OK );
    return handle;
}

static kf_handle_t add_box( kf_world_t * world, uint32_t id, kf_body_type_t type, kf_pose_t pose, kf_vec3_t half, kf_fixed_t friction )
{
    kf_body_desc_t desc = kf_body_desc_default( id );
    kf_shape_desc_t shape = {0};
    kf_handle_t handle = {0};
    desc.type = type;
    desc.pose = pose;
    desc.friction = friction;
    shape.id = 1;
    shape.shape.type = KF_SHAPE_BOX;
    shape.shape.data.box.half_extents = half;
    CHECK( kf_world_add_body( world, &desc, &shape, 1, &handle ) == KF_OK );
    return handle;
}

static kf_pose_t pose_at( kf_fixed_t x, kf_fixed_t y, kf_fixed_t z )
{
    kf_pose_t pose = {{x, y, z}, {0, 0, 0}};
    return pose;
}

static kf_vec3_t unit_half( void )
{
    return (kf_vec3_t){R(1, 2), R(1, 2), R(1, 2)};
}

static kf_bool_t step_many( kf_world_t * world, unsigned count )
{
    unsigned index;
    for( index = 0; index != count; ++index )
    {
        if( kf_world_step( world ) != KF_OK ) return KF_FALSE;
    }
    return KF_TRUE;
}

static void test_resting_contacts_and_stacks( void )
{
    kf_world_t * world = make_large_world( kf_world_config_default() );
    kf_body_state_t state;
    unsigned index;
    if( world == NULL ) return;
    (void)add_floor( world, 1 );
    /* Resting contacts do not consume a global event budget. */
    for( index = 0; index != 40; ++index )
    {
        (void)add_box( world, 100 + index, KF_BODY_DYNAMIC, pose_at( F(index * 2) - F(40), R(1, 2), F(10) ), unit_half(), R(1, 2) );
    }
    for( index = 0; index != 10; ++index )
    {
        (void)add_box( world, 10 + index, KF_BODY_DYNAMIC, pose_at( 0, R(1, 2) + F(index), 0 ), unit_half(), R(1, 2) );
    }
    CHECK( step_many( world, 250 ) == KF_TRUE );
    for( index = 0; index != 10; ++index )
    {
        kf_handle_t handle = {10 + index, 1};
        CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
        CHECK( state.sleeping == KF_TRUE );
        CHECK( near( state.pose.position.x, 0, R(1, 50) ) == KF_TRUE );
        CHECK( near( state.pose.position.y, R(1, 2) + F(index), R(1, 10) ) == KF_TRUE );
    }
    kf_world_destroy( world );
}

static void test_restitution_threshold_and_islands( void )
{
    kf_world_t * world = make_world( kf_world_config_default() );
    kf_body_desc_t desc = kf_body_desc_default( 3 );
    kf_shape_desc_t shape = {0};
    kf_handle_t bouncy;
    kf_handle_t bottom;
    kf_handle_t top;
    kf_body_state_t state;
    if( world == NULL ) return;
    (void)add_floor( world, 1 );
    /* Slow approaches do not bounce, so a resting body with restitution sleeps. */
    desc.type = KF_BODY_DYNAMIC;
    desc.pose.position = (kf_vec3_t){F(5), R(1, 2), 0};
    desc.restitution = R(1, 2);
    shape.id = 1;
    shape.shape.type = KF_SHAPE_BOX;
    shape.shape.data.box.half_extents = unit_half();
    CHECK( kf_world_add_body( world, &desc, &shape, 1, &bouncy ) == KF_OK );
    bottom = add_box( world, 10, KF_BODY_DYNAMIC, pose_at( 0, R(1, 2), 0 ), unit_half(), R(1, 2) );
    top = add_box( world, 11, KF_BODY_DYNAMIC, pose_at( 0, R(3, 2), 0 ), unit_half(), R(1, 2) );
    CHECK( step_many( world, 100 ) == KF_TRUE );
    CHECK( kf_world_get_body( world, bouncy, &state ) == KF_OK );
    CHECK( state.sleeping == KF_TRUE );
    CHECK( kf_world_get_body( world, top, &state ) == KF_OK );
    CHECK( state.sleeping == KF_TRUE );
    /* Waking the support wakes its island, so the top box falls instead of hovering. */
    CHECK( kf_world_apply_impulse( world, bottom, (kf_vec3_t){F(30), 0, 0} ) == KF_OK );
    CHECK( step_many( world, 25 ) == KF_TRUE );
    CHECK( kf_world_get_body( world, top, &state ) == KF_OK );
    CHECK( near( state.pose.position.y, R(1, 2), R(1, 20) ) == KF_TRUE );
    kf_world_destroy( world );
}

static void test_limits_do_not_stall( void )
{
    kf_world_config_t config = kf_world_config_default();
    kf_world_t * world = make_world( config );
    kf_handle_t faller;
    kf_handle_t platform;
    kf_handle_t box;
    kf_handle_t pusher;
    kf_body_desc_t wall = kf_body_desc_default( 4 );
    kf_shape_desc_t wall_shape = {0};
    kf_handle_t handle;
    kf_body_state_t state;
    if( world == NULL ) return;
    /* A body leaving the coordinate range stops at the boundary; others continue. */
    faller = add_sphere( world, 1, KF_BODY_DYNAMIC, (kf_vec3_t){0, 0, 0}, (kf_vec3_t){0, 0, 0}, 0 );
    platform = add_sphere( world, 2, KF_BODY_KINEMATIC, (kf_vec3_t){F(10), 0, 0}, (kf_vec3_t){0, 0, F(1)}, 0 );
    CHECK( step_many( world, 600 ) == KF_TRUE );
    CHECK( kf_world_tick( world ) == 600 );
    CHECK( kf_world_get_body( world, faller, &state ) == KF_OK );
    CHECK( state.pose.position.y >= -KF_COORDINATE_LIMIT + R(1, 4) );
    CHECK( kf_world_get_body( world, platform, &state ) == KF_OK );
    CHECK( state.pose.position.z == F(24) );
    kf_world_destroy( world );
    /* A kinematic crusher passes through a wedged box, never the box through static geometry. */
    config.gravity = (kf_vec3_t){0, 0, 0};
    world = make_world( config );
    if( world == NULL ) return;
    wall_shape.id = 1;
    wall_shape.shape.type = KF_SHAPE_AABB;
    wall_shape.shape.data.aabb = (kf_aabb_t){{F(2), -F(2), -F(2)}, {F(3), F(2), F(2)}};
    CHECK( kf_world_add_body( world, &wall, &wall_shape, 1, &handle ) == KF_OK );
    box = add_box( world, 5, KF_BODY_DYNAMIC, pose_at( F(1), 0, 0 ), unit_half(), 0 );
    pusher = add_box( world, 6, KF_BODY_KINEMATIC, pose_at( -F(1), 0, 0 ), unit_half(), 0 );
    CHECK( kf_world_get_body( world, pusher, &state ) == KF_OK );
    state.velocity.x = F(2);
    CHECK( kf_world_set_body( world, &state ) == KF_OK );
    CHECK( step_many( world, 50 ) == KF_TRUE );
    CHECK( kf_world_get_body( world, box, &state ) == KF_OK );
    CHECK( state.pose.position.x < F(2) - R(1, 2) + R(1, 20) );
    kf_world_destroy( world );
}

static void test_rotation_dynamics( void )
{
    kf_world_config_t config = kf_world_config_default();
    kf_world_t * world;
    kf_body_desc_t desc;
    kf_handle_t handle;
    kf_body_state_t state;
    kf_pose_t pose = pose_at( 0, F(1) + R(1, 50), 0 );
    /* Slow rotation is kept at Q2.30 precision: 1/1000 rad/s for 100 s. */
    config.gravity = (kf_vec3_t){0, 0, 0};
    world = make_world( config );
    if( world == NULL ) return;
    desc = kf_body_desc_default( 1 );
    desc.type = KF_BODY_KINEMATIC;
    desc.angular_velocity.y = R(1, 1000);
    CHECK( kf_world_add_body( world, &desc, NULL, 0, &handle ) == KF_OK );
    CHECK( step_many( world, 2500 ) == KF_TRUE );
    CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
    /* 65/65536 rad/s for 100 s turns 0.09918 rad: y = sin(angle / 2) * 2^30 = 53.23e6. */
    CHECK( state.pose.rotation.y > 53000000 && state.pose.rotation.y < 53400000 );
    kf_world_destroy( world );
    /* A tall box tipped past its critical angle falls over (inertia and contact torque). */
    world = make_world( kf_world_config_default() );
    if( world == NULL ) return;
    (void)add_floor( world, 1 );
    pose.rotation = kf_rotation_from_axis_angle( (kf_vec3_t){0, 0, F(1)}, kf_fixed_from_double( 0.4 ) );
    handle = add_box( world, 2, KF_BODY_DYNAMIC, pose, (kf_vec3_t){R(1, 4), F(1), R(1, 4)}, R(1, 2) );
    CHECK( step_many( world, 100 ) == KF_TRUE );
    CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
    CHECK( near( state.pose.position.y, R(1, 4), R(1, 50) ) == KF_TRUE );
    kf_world_destroy( world );
    /* A sphere with an off-center impulse spins; one at its center does not. */
    world = make_world( config );
    if( world == NULL ) return;
    handle = add_sphere( world, 3, KF_BODY_DYNAMIC, (kf_vec3_t){0, 0, 0}, (kf_vec3_t){0, 0, 0}, 0 );
    CHECK( kf_world_apply_impulse_at( world, handle, (kf_vec3_t){F(1), 0, 0}, (kf_vec3_t){0, 0, 0} ) == KF_OK );
    CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
    CHECK( state.angular_velocity.x == 0 && state.angular_velocity.y == 0 && state.angular_velocity.z == 0 );
    CHECK( kf_world_apply_impulse_at( world, handle, (kf_vec3_t){F(1), 0, 0}, (kf_vec3_t){0, R(1, 4), 0} ) == KF_OK );
    CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
    /* r x J = (0, 1/4, 0) x (1, 0, 0) points along -Z. */
    CHECK( state.angular_velocity.z < 0 && state.angular_velocity.x == 0 );
    kf_world_destroy( world );
}

static void test_rotational_continuous_collision( void )
{
    kf_world_config_t config = kf_world_config_default();
    kf_world_t * world;
    kf_body_desc_t wall = kf_body_desc_default( 1 );
    kf_shape_desc_t wall_shape = {0};
    kf_body_desc_t rod = kf_body_desc_default( 2 );
    kf_shape_desc_t rod_shape = {0};
    kf_handle_t handle;
    kf_body_state_t state;
    unsigned tick;
    config.gravity = (kf_vec3_t){0, 0, 0};
    world = make_world( config );
    if( world == NULL ) return;
    wall_shape.id = 1;
    wall_shape.shape.type = KF_SHAPE_AABB;
    wall_shape.shape.data.aabb = (kf_aabb_t){{F(1), -F(5), -F(5)}, {F(1) + R(1, 100), F(5), F(5)}};
    CHECK( kf_world_add_body( world, &wall, &wall_shape, 1, &handle ) == KF_OK );
    /* Tip speed 90 units/s sweeps 3.6 units per tick past a 0.01 wall. */
    rod.type = KF_BODY_DYNAMIC;
    rod.angular_velocity.y = F(60);
    rod.velocity.x = R(1, 2);
    rod_shape.id = 1;
    rod_shape.shape.type = KF_SHAPE_BOX;
    rod_shape.shape.data.box.half_extents = (kf_vec3_t){R(3, 2), R(1, 20), R(1, 20)};
    CHECK( kf_world_add_body( world, &rod, &rod_shape, 1, &handle ) == KF_OK );
    for( tick = 0; tick != 50; ++tick )
    {
        kf_vec3_t tip;
        CHECK( kf_world_step( world ) == KF_OK );
        CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
        tip = kf_rotation_apply( state.pose.rotation, (kf_vec3_t){R(3, 2), 0, R(1, 20)} );
        CHECK( (int64_t)state.pose.position.x + (tip.x < 0 ? -(int64_t)tip.x : tip.x) < (int64_t)F(1) + R(1, 10) );
    }
    kf_world_destroy( world );
}

static void test_kinematic_target_and_contact_snapshot( void )
{
    kf_world_t * first = make_large_world( kf_world_config_default() );
    kf_world_t * second = make_large_world( kf_world_config_default() );
    kf_body_desc_t pusher = kf_body_desc_default( 2 );
    kf_shape_desc_t capsule = {0};
    kf_handle_t pusher_handle;
    kf_handle_t box;
    kf_body_state_t state;
    kf_pose_t target = pose_at( -F(1), 0, 0 );
    kf_snapshot_t snapshot = {0};
    kf_object_snapshot_t objects[16];
    kf_shape_snapshot_t shapes[16];
    kf_contact_snapshot_t contacts[16];
    unsigned index;
    if( first == NULL || second == NULL ) { kf_world_destroy( first ); kf_world_destroy( second ); return; }
    (void)add_floor( first, 1 );
    box = add_box( first, 3, KF_BODY_DYNAMIC, pose_at( F(1), R(1, 2), 0 ), unit_half(), R(1, 2) );
    (void)add_box( first, 4, KF_BODY_DYNAMIC, pose_at( F(1), R(3, 2), 0 ), unit_half(), R(1, 2) );
    pusher.type = KF_BODY_KINEMATIC;
    pusher.pose = target;
    capsule.id = 1;
    capsule.shape.type = KF_SHAPE_CAPSULE;
    capsule.shape.data.capsule.radius = R(1, 3);
    capsule.shape.data.capsule.height = F(2);
    CHECK( kf_world_add_body( first, &pusher, &capsule, 1, &pusher_handle ) == KF_OK );
    /* A kinematic target is reached exactly and pushes with its implied velocity. */
    for( index = 0; index != 30; ++index )
    {
        target.position.x += R(2, 25);
        CHECK( kf_world_set_kinematic_target( first, pusher_handle, target ) == KF_OK );
        CHECK( kf_world_step( first ) == KF_OK );
        CHECK( kf_world_get_body( first, pusher_handle, &state ) == KF_OK );
        CHECK( state.pose.position.x == target.position.x && state.has_target == KF_FALSE );
    }
    CHECK( kf_world_get_body( first, box, &state ) == KF_OK );
    CHECK( near( state.velocity.x, F(2), R(1, 10) ) == KF_TRUE );
    /* Carried contact impulses are part of the snapshot, so a restored world continues identically. */
    snapshot.objects = objects; snapshot.object_capacity = 16;
    snapshot.shapes = shapes; snapshot.shape_capacity = 16;
    CHECK( kf_world_save( first, &snapshot ) == KF_CAPACITY_EXCEEDED );
    CHECK( snapshot.contact_count != 0 );
    snapshot.contacts = contacts; snapshot.contact_capacity = 16;
    CHECK( kf_world_save( first, &snapshot ) == KF_OK );
    CHECK( kf_world_restore( second, &snapshot ) == KF_OK );
    for( index = 0; index != 20; ++index )
    {
        unsigned body;
        CHECK( kf_world_step( first ) == KF_OK );
        CHECK( kf_world_step( second ) == KF_OK );
        for( body = 1; body != 5; ++body )
        {
            kf_handle_t handle = {body, 1};
            kf_body_state_t left;
            kf_body_state_t right;
            CHECK( kf_world_get_body( first, handle, &left ) == KF_OK );
            CHECK( kf_world_get_body( second, handle, &right ) == KF_OK );
            CHECK( same_body_state( &left, &right ) == KF_TRUE );
        }
    }
    kf_world_destroy( first );
    kf_world_destroy( second );
}

static void test_gravity_scale_and_rotation_locks( void )
{
    kf_world_t * world = make_large_world( kf_world_config_default() );
    kf_body_desc_t desc;
    kf_body_state_t normal;
    kf_body_state_t floating;
    kf_body_state_t heavy;
    kf_body_state_t state;
    kf_handle_t handles[3];
    kf_handle_t pillar;
    kf_shape_desc_t shape = sphere_shape( R(1, 4) );
    unsigned index;
    if( world == NULL ) return;
    (void)add_floor( world, 1 );
    for( index = 0; index != 3; ++index )
    {
        desc = kf_body_desc_default( 10 + index );
        desc.type = KF_BODY_DYNAMIC;
        desc.pose.position = (kf_vec3_t){F(index * 2), F(40), 0};
        desc.gravity_scale = (kf_fixed_t)(index * KF_FIXED_SCALE);
        CHECK( kf_world_add_body( world, &desc, &shape, 1, handles + index ) == KF_OK );
    }
    CHECK( step_many( world, 25 ) == KF_TRUE );
    CHECK( kf_world_get_body( world, handles[0], &floating ) == KF_OK );
    CHECK( kf_world_get_body( world, handles[1], &normal ) == KF_OK );
    CHECK( kf_world_get_body( world, handles[2], &heavy ) == KF_OK );
    CHECK( floating.velocity.y == 0 && floating.pose.position.y == F(40) );
    CHECK( near( normal.velocity.y, -F(20), R(1, 100) ) == KF_TRUE );
    CHECK( near( heavy.velocity.y, normal.velocity.y * 2, 4 ) == KF_TRUE );
    desc.gravity_scale = F(17);
    CHECK( kf_world_modify_body( world, handles[2], &desc ) == KF_OUT_OF_RANGE );
    desc.gravity_scale = KF_FIXED_SCALE;
    desc.rotation_locks = 8u;
    CHECK( kf_world_modify_body( world, handles[2], &desc ) == KF_INVALID_ARGUMENT );
    /* A pillar locked to yaw slides instead of toppling when struck high. */
    pillar = add_box( world, 20, KF_BODY_DYNAMIC, pose_at( F(10), F(1), 0 ), (kf_vec3_t){R(1, 4), F(1), R(1, 4)}, R(1, 10) );
    CHECK( kf_world_get_body( world, pillar, &state ) == KF_OK );
    desc = kf_body_desc_default( 20 );
    desc.type = KF_BODY_DYNAMIC;
    desc.friction = R(1, 10);
    desc.rotation_locks = KF_LOCK_ROTATION_X | KF_LOCK_ROTATION_Z;
    CHECK( kf_world_modify_body( world, pillar, &desc ) == KF_OK );
    CHECK( step_many( world, 10 ) == KF_TRUE );
    CHECK( kf_world_apply_impulse_at( world, pillar, (kf_vec3_t){F(4), 0, F(1)}, (kf_vec3_t){F(10) + R(1, 4), R(19, 10), R(1, 4)} ) == KF_OK );
    CHECK( kf_world_get_body( world, pillar, &state ) == KF_OK );
    CHECK( state.angular_velocity.x == 0 && state.angular_velocity.z == 0 && state.angular_velocity.y != 0 );
    CHECK( step_many( world, 60 ) == KF_TRUE );
    CHECK( kf_world_get_body( world, pillar, &state ) == KF_OK );
    CHECK( state.pose.rotation.x == 0 && state.pose.rotation.z == 0 && state.pose.rotation.y != 0 );
    CHECK( state.pose.position.x > F(11) && near( state.pose.position.y, F(1), R(1, 50) ) == KF_TRUE );
    kf_world_destroy( world );
}

static void test_modify_body_and_set_velocity( void )
{
    kf_world_t * world = make_large_world( kf_world_config_default() );
    kf_body_desc_t desc = kf_body_desc_default( 10 );
    kf_shape_desc_t shape = sphere_shape( R(1, 4) );
    kf_body_state_t state;
    kf_handle_t floor;
    kf_handle_t ball;
    if( world == NULL ) return;
    floor = add_floor( world, 1 );
    desc.pose.position = (kf_vec3_t){0, F(2), 0};
    CHECK( kf_world_add_body( world, &desc, &shape, 1, &ball ) == KF_OK );
    CHECK( kf_world_set_velocity( world, ball, (kf_vec3_t){F(1), 0, 0}, (kf_vec3_t){0, 0, 0} ) == KF_INVALID_ARGUMENT );
    CHECK( kf_world_set_velocity( world, floor, (kf_vec3_t){F(1), 0, 0}, (kf_vec3_t){0, 0, 0} ) == KF_INVALID_ARGUMENT );
    /* A static body made dynamic keeps its pose and starts falling. */
    desc.type = KF_BODY_DYNAMIC;
    desc.mass = F(3);
    desc.pose.position = (kf_vec3_t){F(50), F(50), 0};
    CHECK( kf_world_modify_body( world, ball, &desc ) == KF_OK );
    CHECK( kf_world_get_body( world, ball, &state ) == KF_OK );
    CHECK( state.pose.position.x == 0 && state.pose.position.y == F(2) );
    CHECK( state.inverse_mass == kf_fixed_div( KF_FIXED_SCALE, F(3) ) );
    CHECK( step_many( world, 120 ) == KF_TRUE );
    CHECK( kf_world_get_body( world, ball, &state ) == KF_OK );
    CHECK( state.sleeping == KF_TRUE && near( state.pose.position.y, R(1, 4), R(1, 50) ) == KF_TRUE );
    /* Velocities are replaced and the sleeping body wakes. */
    CHECK( kf_world_set_velocity( world, ball, (kf_vec3_t){F(2), 0, 0}, (kf_vec3_t){0, F(1), 0} ) == KF_OK );
    CHECK( kf_world_get_body( world, ball, &state ) == KF_OK );
    CHECK( state.sleeping == KF_FALSE && state.velocity.x == F(2) && state.angular_velocity.y == F(1) );
    CHECK( kf_world_set_velocity( world, ball, (kf_vec3_t){KF_SPEED_LIMIT + 1, 0, 0}, (kf_vec3_t){0, 0, 0} ) == KF_OUT_OF_RANGE );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_get_body( world, ball, &state ) == KF_OK );
    CHECK( state.pose.position.x > 0 );
    /* A body made static stops. */
    desc.type = KF_BODY_STATIC;
    CHECK( kf_world_modify_body( world, ball, &desc ) == KF_OK );
    CHECK( kf_world_get_body( world, ball, &state ) == KF_OK );
    CHECK( state.velocity.x == 0 && state.angular_velocity.y == 0 && state.inverse_mass == 0 );
    desc.id = 11;
    CHECK( kf_world_modify_body( world, ball, &desc ) == KF_INVALID_ARGUMENT );
    kf_world_destroy( world );
}

static void test_spring_constraint( void )
{
    kf_world_t * world = make_large_world( kf_world_config_default() );
    kf_constraint_desc_t desc = {0};
    kf_constraint_state_t joint;
    kf_body_state_t state;
    kf_handle_t anchor;
    kf_handle_t weight;
    kf_handle_t handle;
    kf_fixed_t lowest = F(100);
    unsigned index;
    if( world == NULL ) return;
    anchor = add_sphere( world, 1, KF_BODY_STATIC, (kf_vec3_t){0, F(10), 0}, (kf_vec3_t){0, 0, 0}, 0 );
    weight = add_sphere( world, 2, KF_BODY_DYNAMIC, (kf_vec3_t){0, F(8), 0}, (kf_vec3_t){0, 0, 0}, 0 );
    desc.id = 1;
    desc.type = KF_CONSTRAINT_DISTANCE;
    desc.left = anchor;
    desc.right = weight;
    desc.length = F(2);
    desc.hertz = F(2);
    desc.damping_ratio = R(1, 10);
    CHECK( kf_world_add_constraint( world, &desc, &handle ) == KF_OK );
    /* A lightly damped spring overshoots, then settles at the stretch g / omega^2. */
    for( index = 0; index != 600; ++index )
    {
        CHECK( kf_world_step( world ) == KF_OK );
        CHECK( kf_world_get_body( world, weight, &state ) == KF_OK );
        if( state.pose.position.y < lowest ) lowest = state.pose.position.y;
    }
    CHECK( lowest < F(8) - R(2, 10) );
    CHECK( near( state.pose.position.y, F(8) - R(1267, 10000), R(1, 100) ) == KF_TRUE );
    CHECK( kf_world_get_constraint( world, handle, &joint ) == KF_OK );
    joint.desc.hertz = -1;
    CHECK( kf_world_set_constraint( world, &joint ) == KF_OUT_OF_RANGE );
    kf_world_destroy( world );
}

static void test_contact_reports( void )
{
    kf_world_t * world = make_large_world( kf_world_config_default() );
    kf_contact_report_t reports[4];
    kf_snapshot_t snapshot = {0};
    kf_object_snapshot_t objects[8];
    kf_shape_snapshot_t shapes[8];
    kf_contact_snapshot_t contacts[8];
    kf_handle_t floor;
    kf_handle_t box;
    size_t required = 1;
    unsigned index;
    unsigned began = 0;
    if( world == NULL ) return;
    floor = add_floor( world, 1 );
    box = add_box( world, 10, KF_BODY_DYNAMIC, pose_at( 0, F(1), 0 ), unit_half(), R(1, 2) );
    CHECK( kf_world_contact_reports( world, reports, 4, &required ) == KF_NO_HIT && required == 0 );
    CHECK( kf_world_contact_reports( world, NULL, 4, &required ) == KF_INVALID_ARGUMENT );
    for( index = 0; index != 60; ++index )
    {
        kf_status_t status;
        CHECK( kf_world_step( world ) == KF_OK );
        status = kf_world_contact_reports( world, reports, 4, &required );
        CHECK( status == KF_OK || status == KF_NO_HIT );
        if( status != KF_OK ) continue;
        CHECK( required == 1 );
        CHECK( reports[0].left.id == floor.id && reports[0].right.id == box.id );
        CHECK( reports[0].left_shape == 1 && reports[0].right_shape == 1 );
        CHECK( reports[0].normal.y == -KF_FIXED_SCALE );
        if( reports[0].began == KF_TRUE )
        {
            ++began;
            CHECK( reports[0].approach_speed > F(3) );
            CHECK( near( reports[0].point.y, 0, R(1, 10) ) == KF_TRUE && near( reports[0].point.x, 0, R(1, 100) ) == KF_TRUE );
            CHECK( kf_world_contact_reports( world, NULL, 0, &required ) == KF_CAPACITY_EXCEEDED && required == 1 );
        }
    }
    CHECK( began == 1 );
    /* The touching state is carried by snapshots, so a restore does not begin the pair again. */
    snapshot.objects = objects; snapshot.object_capacity = 8;
    snapshot.shapes = shapes; snapshot.shape_capacity = 8;
    snapshot.contacts = contacts; snapshot.contact_capacity = 8;
    CHECK( kf_world_save( world, &snapshot ) == KF_OK );
    CHECK( snapshot.contact_count == 1 && contacts[0].touching == KF_TRUE );
    contacts[0].touching = 2;
    CHECK( kf_world_restore( world, &snapshot ) == KF_INVALID_ARGUMENT );
    contacts[0].touching = KF_TRUE;
    contacts[0].count = KF_MANIFOLD_MAX_POINTS + 1u;
    CHECK( kf_world_restore( world, &snapshot ) == KF_INVALID_ARGUMENT );
    contacts[0].count = 4;
    CHECK( kf_world_restore( world, &snapshot ) == KF_OK );
    CHECK( kf_world_contact_reports( world, reports, 4, &required ) == KF_NO_HIT );
    CHECK( kf_world_wake_body( world, box ) == KF_OK );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_contact_reports( world, reports, 4, &required ) == KF_OK && required == 1 && reports[0].began == KF_FALSE );
    (void)add_box( world, 11, KF_BODY_DYNAMIC, pose_at( F(5), F(1), 0 ), unit_half(), R(1, 2) );
    CHECK( kf_world_contact_reports( world, reports, 4, &required ) == KF_NO_HIT );
    kf_world_destroy( world );
}

static kf_handle_t add_zone( kf_world_t * world, uint32_t id, kf_body_type_t type, kf_vec3_t position, uint32_t category, uint32_t mask )
{
    kf_body_desc_t desc = kf_body_desc_default( id );
    kf_shape_desc_t shape = {0};
    kf_handle_t handle = {0};
    desc.type = type;
    desc.pose.position = position;
    desc.category = category;
    desc.mask = mask;
    desc.trigger = KF_TRUE;
    shape.id = 1;
    shape.shape.type = KF_SHAPE_BOX;
    shape.shape.data.box.half_extents = (kf_vec3_t){F(1), F(1), F(1)};
    CHECK( kf_world_add_body( world, &desc, &shape, 1, &handle ) == KF_OK );
    return handle;
}

static kf_bool_t same_trigger_event( const kf_trigger_event_t * left, const kf_trigger_event_t * right )
{
    return (kf_bool_t)(left->overlap.trigger.id == right->overlap.trigger.id && left->overlap.trigger.generation == right->overlap.trigger.generation &&
        left->overlap.trigger_shape == right->overlap.trigger_shape && left->overlap.visitor.id == right->overlap.visitor.id &&
        left->overlap.visitor.generation == right->overlap.visitor.generation && left->overlap.visitor_shape == right->overlap.visitor_shape &&
        left->began == right->began);
}

static unsigned trigger_events( kf_world_t * world, kf_trigger_event_t * events )
{
    size_t required = 0;
    kf_status_t status = kf_world_trigger_events( world, events, 8, &required );
    CHECK( status == KF_OK || status == KF_NO_HIT );
    CHECK( (status == KF_NO_HIT) == (required == 0) );
    return (unsigned)required;
}

static void test_trigger_events( void )
{
    kf_world_config_t config = kf_world_config_default();
    kf_world_t * world;
    kf_world_t * copy;
    kf_trigger_event_t events[8];
    kf_trigger_event_t copied[8];
    kf_snapshot_t snapshot = {0};
    kf_object_snapshot_t objects[16];
    kf_shape_snapshot_t shapes[16];
    kf_trigger_overlap_t overlaps[8];
    kf_body_desc_t filtered = kf_body_desc_default( 4 );
    kf_shape_desc_t box = {0};
    kf_handle_t zone;
    kf_handle_t ball;
    kf_handle_t handle;
    size_t required = 1;
    unsigned began = 0;
    unsigned ended = 0;
    unsigned index;
    config.gravity = (kf_vec3_t){0, 0, 0};
    world = make_large_world( config );
    copy = make_large_world( config );
    if( world == NULL || copy == NULL ) { kf_world_destroy( world ); kf_world_destroy( copy ); return; }
    zone = add_zone( world, 1, KF_BODY_STATIC, (kf_vec3_t){F(4), 0, 0}, 2u, UINT32_MAX );
    /* Static geometry, a filtered-out body and another trigger inside the zone are never reported. */
    (void)add_box( world, 2, KF_BODY_STATIC, pose_at( F(4), 0, F(1) ), unit_half(), 0 );
    (void)add_zone( world, 3, KF_BODY_DYNAMIC, (kf_vec3_t){F(4), F(1), 0}, 1u, 0u );
    filtered.type = KF_BODY_DYNAMIC;
    filtered.pose.position = (kf_vec3_t){F(4), 0, -F(1)};
    filtered.mask = 1u;
    box.id = 1;
    box.shape.type = KF_SHAPE_BOX;
    box.shape.data.box.half_extents = unit_half();
    CHECK( kf_world_add_body( world, &filtered, &box, 1, &handle ) == KF_OK );
    ball = add_sphere( world, 10, KF_BODY_DYNAMIC, (kf_vec3_t){0, 0, 0}, (kf_vec3_t){F(4), 0, 0}, 0 );
    CHECK( kf_world_trigger_events( world, events, 8, &required ) == KF_NO_HIT && required == 0 );
    CHECK( kf_world_trigger_events( world, NULL, 8, &required ) == KF_INVALID_ARGUMENT );
    for( index = 0; index != 60; ++index )
    {
        kf_body_state_t state;
        unsigned count;
        unsigned event;
        CHECK( kf_world_step( world ) == KF_OK );
        if( index == 25 )
        {
            /* Overlaps are part of the snapshot, so a restored world reports the same events. */
            snapshot.objects = objects; snapshot.object_capacity = 16;
            snapshot.shapes = shapes; snapshot.shape_capacity = 16;
            CHECK( kf_world_save( world, &snapshot ) == KF_CAPACITY_EXCEEDED && snapshot.trigger_count == 1 );
            snapshot.triggers = overlaps; snapshot.trigger_capacity = 8;
            CHECK( kf_world_save( world, &snapshot ) == KF_OK );
            CHECK( kf_world_restore( copy, &snapshot ) == KF_OK );
            CHECK( kf_world_trigger_events( copy, events, 8, &required ) == KF_NO_HIT );
        }
        count = trigger_events( world, events );
        if( index > 25 )
        {
            CHECK( kf_world_step( copy ) == KF_OK );
            CHECK( trigger_events( copy, copied ) == count );
            for( event = 0; event != count; ++event )
            {
                CHECK( same_trigger_event( events + event, copied + event ) == KF_TRUE );
            }
        }
        CHECK( kf_world_get_body( world, ball, &state ) == KF_OK );
        for( event = 0; event != count; ++event )
        {
            CHECK( events[event].overlap.trigger.id == zone.id && events[event].overlap.trigger_shape == 1 );
            CHECK( events[event].overlap.visitor.id == ball.id && events[event].overlap.visitor_shape == 1 );
            if( events[event].began == KF_TRUE )
            {
                ++began;
                CHECK( state.pose.position.x >= F(3) - R(1, 4) - R(1, 5) && state.pose.position.x <= F(3) - R(1, 4) + R(1, 5) );
            }
            else
            {
                ++ended;
                CHECK( state.pose.position.x >= F(5) + R(1, 4) && state.pose.position.x <= F(5) + R(1, 4) + R(1, 5) );
            }
        }
    }
    CHECK( began == 1 && ended == 1 );
    /* Removing a visitor ends its overlap with its last handle. */
    handle = add_sphere( world, 11, KF_BODY_DYNAMIC, (kf_vec3_t){F(4), 0, 0}, (kf_vec3_t){0, 0, 0}, 0 );
    CHECK( kf_world_trigger_events( world, events, 8, &required ) == KF_NO_HIT );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( trigger_events( world, events ) == 1 && events[0].began == KF_TRUE && events[0].overlap.visitor.id == 11 );
    CHECK( kf_world_remove_body( world, handle ) == KF_OK );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( trigger_events( world, events ) == 1 && events[0].began == KF_FALSE );
    CHECK( events[0].overlap.visitor.id == handle.id && events[0].overlap.visitor.generation == handle.generation );
    /* Snapshots must list overlaps in strict canonical order. */
    overlaps[1] = overlaps[0];
    snapshot.trigger_count = 2;
    CHECK( kf_world_restore( copy, &snapshot ) == KF_INVALID_ARGUMENT );
    kf_world_destroy( world );
    kf_world_destroy( copy );
    /* A step finding more overlaps than reserved keeps the previous set. */
    world = NULL;
    CHECK( kf_world_create( &config, NULL, &world ) == KF_OK );
    if( world == NULL ) return;
    CHECK( kf_world_reserve( world, (kf_capacity_t){8, 8, 0, 0, 1} ) == KF_OK );
    (void)add_zone( world, 1, KF_BODY_STATIC, (kf_vec3_t){0, 0, 0}, 1u, UINT32_MAX );
    (void)add_sphere( world, 2, KF_BODY_DYNAMIC, (kf_vec3_t){-R(1, 2), 0, 0}, (kf_vec3_t){0, 0, 0}, 0 );
    (void)add_sphere( world, 3, KF_BODY_DYNAMIC, (kf_vec3_t){R(1, 2), 0, 0}, (kf_vec3_t){0, 0, 0}, 0 );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_trigger_events( world, events, 8, &required ) == KF_CAPACITY_EXCEEDED && required == 2 );
    CHECK( kf_world_reserve( world, (kf_capacity_t){8, 8, 0, 0, 2} ) == KF_OK );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( trigger_events( world, events ) == 2 && events[0].began == KF_TRUE && events[1].began == KF_TRUE );
    CHECK( events[0].overlap.visitor.id == 2 && events[1].overlap.visitor.id == 3 );
    kf_world_destroy( world );
}

static void test_full_turn_continuous_collision( void )
{
    const uint32_t substeps[] = {1u, 8u, 16u};
    size_t index;
    for( index = 0; index != sizeof(substeps) / sizeof(substeps[0]); ++index )
    {
        kf_world_config_t config = kf_world_config_default();
        kf_world_t * world;
        kf_body_desc_t desc = kf_body_desc_default( 1 );
        kf_shape_desc_t shape = {0};
        kf_handle_t handle;
        kf_body_state_t state;
        config.gravity = (kf_vec3_t){0, 0, 0};
        config.sleep_ticks = 0;
        config.substeps = substeps[index];
        world = make_world( config );
        if( world == NULL ) return;
        shape.id = 1;
        shape.shape.type = KF_SHAPE_AABB;
        shape.shape.data.aabb = (kf_aabb_t){{-F(4), -F(4), F(1)}, {F(4), F(4), F(1) + R(1, 100)}};
        CHECK( kf_world_add_body( world, &desc, &shape, 1, &handle ) == KF_OK );
        desc = kf_body_desc_default( 2 );
        desc.type = KF_BODY_DYNAMIC;
        desc.angular_velocity.y = kf_fixed_from_double( 157.079632679 );
        desc.friction = 0;
        shape = (kf_shape_desc_t){0};
        shape.id = 1;
        shape.shape.type = KF_SHAPE_BOX;
        shape.shape.data.box.half_extents = (kf_vec3_t){R(3, 2), R(1, 20), R(1, 20)};
        CHECK( kf_world_add_body( world, &desc, &shape, 1, &handle ) == KF_OK );
        CHECK( kf_world_step( world ) == KF_OK );
        CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
        /* The free end orientation is almost identity; its intervening turn hits the wall. */
        CHECK( state.angular_velocity.y < F(100) );
        CHECK( state.pose.rotation.y != 0 || state.pose.position.z != 0 );
        kf_world_destroy( world );
    }
}

static void test_continuous_path_invalidation( void )
{
    kf_world_config_t config = kf_world_config_default();
    kf_world_t * world;
    kf_handle_t first;
    kf_handle_t second;
    kf_body_state_t a;
    kf_body_state_t b;
    config.gravity = (kf_vec3_t){0, 0, 0};
    config.sleep_ticks = 0;
    world = make_world( config );
    if( world == NULL ) return;
    first = add_sphere( world, 1, KF_BODY_DYNAMIC, (kf_vec3_t){-F(3), 0, 0}, (kf_vec3_t){F(100), 0, 0}, KF_FIXED_SCALE );
    second = add_sphere( world, 2, KF_BODY_DYNAMIC, (kf_vec3_t){R(-4, 5), R(19, 5), 0}, (kf_vec3_t){0, -F(100), 0}, KF_FIXED_SCALE );
    (void)add_sphere( world, 3, KF_BODY_DYNAMIC, (kf_vec3_t){0, 0, 0}, (kf_vec3_t){0, 0, 0}, KF_FIXED_SCALE );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_get_body( world, first, &a ) == KF_OK );
    CHECK( kf_world_get_body( world, second, &b ) == KF_OK );
    CHECK( kf_vec3_length( kf_vec3_sub( a.pose.position, b.pose.position ) ) >= R(1, 2) );
    kf_world_destroy( world );
}

static void test_unreachable_kinematic_target( void )
{
    kf_world_config_t config = kf_world_config_default();
    kf_world_t * world;
    kf_handle_t handle;
    kf_body_state_t before;
    kf_body_state_t after;
    kf_pose_t target = {{F(40), 0, 0}, {0, 0, 0}};
    config.gravity = (kf_vec3_t){0, 0, 0};
    world = make_world( config );
    if( world == NULL ) return;
    handle = add_sphere( world, 1, KF_BODY_KINEMATIC, (kf_vec3_t){-F(40), 0, 0}, (kf_vec3_t){0, 0, 0}, 0 );
    CHECK( kf_world_get_body( world, handle, &before ) == KF_OK );
    CHECK( kf_world_set_kinematic_target( world, handle, target ) == KF_OUT_OF_RANGE );
    CHECK( kf_world_get_body( world, handle, &after ) == KF_OK );
    CHECK( same_body_state( &before, &after ) == KF_TRUE );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_get_body( world, handle, &after ) == KF_OK );
    CHECK( after.pose.position.x == before.pose.position.x );
    /* The exact speed boundary is accepted and its complete path reaches the target. */
    target.position.x = -F(40) + (kf_fixed_t)((int64_t)KF_SPEED_LIMIT * config.step.numerator / config.step.denominator);
    CHECK( kf_world_set_kinematic_target( world, handle, target ) == KF_OK );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_get_body( world, handle, &after ) == KF_OK );
    CHECK( after.pose.position.x == target.position.x );
    kf_world_destroy( world );

    /* A rapid target rotation also obeys the angular speed limit. */
    config.step.denominator = 1000000u;
    world = make_world( config );
    if( world == NULL ) return;
    handle = add_sphere( world, 1, KF_BODY_KINEMATIC, (kf_vec3_t){0, 0, 0}, (kf_vec3_t){0, 0, 0}, 0 );
    target.position = (kf_vec3_t){0, 0, 0};
    target.rotation = kf_rotation_from_yaw( KF_ANGLE16_QUARTER );
    CHECK( kf_world_set_kinematic_target( world, handle, target ) == KF_OUT_OF_RANGE );
    kf_world_destroy( world );
}

static void test_dormant_contact_capacity_rollback( void )
{
    kf_world_config_t config = kf_world_config_default();
    kf_world_t * world = NULL;
    kf_world_t * reference = NULL;
    kf_body_desc_t floor = kf_body_desc_default( 1 );
    kf_shape_desc_t shape = {0};
    kf_handle_t handle;
    kf_handle_t bodies[2];
    kf_snapshot_t snapshot = {0};
    kf_object_snapshot_t objects[8];
    kf_shape_snapshot_t shapes[8];
    kf_contact_snapshot_t contacts[4];
    unsigned tick;
    kf_bool_t retried = KF_FALSE;
    config.sleep_ticks = 3;
    CHECK( kf_world_create( &config, NULL, &world ) == KF_OK );
    CHECK( kf_world_create( &config, NULL, &reference ) == KF_OK );
    if( world == NULL || reference == NULL ) { kf_world_destroy( world ); kf_world_destroy( reference ); return; }
    CHECK( kf_world_reserve( world, (kf_capacity_t){8, 8, 0, 1, 0} ) == KF_OK );
    CHECK( kf_world_reserve( reference, (kf_capacity_t){8, 8, 0, 4, 0} ) == KF_OK );
    shape.id = 1;
    shape.shape.type = KF_SHAPE_AABB;
    shape.shape.data.aabb = (kf_aabb_t){{-F(10), -F(1), -F(10)}, {F(10), 0, F(10)}};
    CHECK( kf_world_add_body( world, &floor, &shape, 1, &handle ) == KF_OK );
    bodies[0] = add_sphere( world, 2, KF_BODY_DYNAMIC, (kf_vec3_t){-F(1), R(1, 4), 0}, (kf_vec3_t){0, 0, 0}, 0 );
    bodies[1] = add_sphere( world, 3, KF_BODY_DYNAMIC, (kf_vec3_t){F(1), F(2), 0}, (kf_vec3_t){0, 0, 0}, 0 );
    snapshot.objects = objects; snapshot.object_capacity = 8;
    snapshot.shapes = shapes; snapshot.shape_capacity = 8;
    snapshot.contacts = contacts; snapshot.contact_capacity = 4;
    CHECK( kf_world_save( world, &snapshot ) == KF_OK );
    CHECK( kf_world_restore( reference, &snapshot ) == KF_OK );
    for( tick = 0; tick != 30u; ++tick )
    {
        kf_body_state_t before[2];
        kf_status_t status;
        uint64_t previous_tick = kf_world_tick( world );
        size_t index;
        for( index = 0; index != 2u; ++index ) CHECK( kf_world_get_body( world, bodies[index], before + index ) == KF_OK );
        CHECK( kf_world_save( world, &snapshot ) == KF_OK );
        status = kf_world_step( world );
        if( status == KF_CAPACITY_EXCEEDED )
        {
            kf_snapshot_t after_snapshot = {0};
            kf_contact_snapshot_t after_contacts[4];
            kf_object_snapshot_t after_objects[8];
            kf_shape_snapshot_t after_shapes[8];
            CHECK( retried == KF_FALSE );
            CHECK( before[0].sleeping == KF_TRUE );
            CHECK( kf_world_tick( world ) == previous_tick );
            for( index = 0; index != 2u; ++index )
            {
                kf_body_state_t after;
                CHECK( kf_world_get_body( world, bodies[index], &after ) == KF_OK );
                CHECK( same_body_state( before + index, &after ) == KF_TRUE );
            }
            after_snapshot.objects = after_objects; after_snapshot.object_capacity = 8;
            after_snapshot.shapes = after_shapes; after_snapshot.shape_capacity = 8;
            after_snapshot.contacts = after_contacts; after_snapshot.contact_capacity = 4;
            CHECK( kf_world_save( world, &after_snapshot ) == KF_OK );
            CHECK( snapshot.contact_count == after_snapshot.contact_count );
            CHECK( memcmp( contacts, after_contacts, snapshot.contact_count * sizeof(*contacts) ) == 0 );
            CHECK( kf_world_step( world ) == KF_CAPACITY_EXCEEDED );
            CHECK( kf_world_tick( world ) == previous_tick );
            CHECK( kf_world_reserve( world, (kf_capacity_t){8, 8, 0, 4, 0} ) == KF_OK );
            CHECK( kf_world_step( world ) == KF_OK );
            retried = KF_TRUE;
        }
        else
        {
            CHECK( status == KF_OK );
        }
        CHECK( kf_world_step( reference ) == KF_OK );
        for( index = 0; index != 2u; ++index )
        {
            kf_body_state_t actual;
            kf_body_state_t expected;
            CHECK( kf_world_get_body( world, bodies[index], &actual ) == KF_OK );
            CHECK( kf_world_get_body( reference, bodies[index], &expected ) == KF_OK );
            CHECK( same_body_state( &actual, &expected ) == KF_TRUE );
        }
        {
            kf_contact_report_t actual[4];
            kf_contact_report_t expected[4];
            size_t actual_count;
            size_t expected_count;
            kf_status_t actual_status = kf_world_contact_reports( world, actual, 4, &actual_count );
            kf_status_t expected_status = kf_world_contact_reports( reference, expected, 4, &expected_count );
            CHECK( actual_status == KF_OK || actual_status == KF_NO_HIT );
            CHECK( actual_status == expected_status );
            CHECK( actual_count == expected_count );
            for( index = 0; index != actual_count && index != expected_count; ++index )
            {
                CHECK( actual[index].left.id == expected[index].left.id );
                CHECK( actual[index].right.id == expected[index].right.id );
                CHECK( actual[index].began == expected[index].began );
            }
        }
    }
    CHECK( retried == KF_TRUE );
    kf_world_destroy( world );
    kf_world_destroy( reference );
}

static void test_large_cached_impulses_with_small_inertia( void )
{
    kf_world_config_t config = kf_world_config_default();
    kf_world_t * world;
    kf_body_desc_t desc = kf_body_desc_default( 1 );
    kf_shape_desc_t shape = {0};
    kf_handle_t handle;
    kf_snapshot_t snapshot = {0};
    kf_object_snapshot_t objects[4];
    kf_shape_snapshot_t shapes[4];
    kf_contact_snapshot_t contacts[4];
    kf_body_state_t state;
    size_t index;
    config.sleep_ticks = 0;
    world = make_world( config );
    if( world == NULL ) return;
    shape.id = 1;
    shape.shape.type = KF_SHAPE_AABB;
    shape.shape.data.aabb = (kf_aabb_t){{-F(1), -F(1), -F(1)}, {F(1), 0, F(1)}};
    CHECK( kf_world_add_body( world, &desc, &shape, 1, &handle ) == KF_OK );
    desc = kf_body_desc_default( 2 );
    desc.type = KF_BODY_DYNAMIC;
    desc.mass = 128;
    desc.pose.position.y = R(1, 1000);
    shape = sphere_shape( R(1, 1000) );
    CHECK( kf_world_add_body( world, &desc, &shape, 1, &handle ) == KF_OK );
    CHECK( kf_world_step( world ) == KF_OK );
    snapshot.objects = objects; snapshot.object_capacity = 4;
    snapshot.shapes = shapes; snapshot.shape_capacity = 4;
    snapshot.contacts = contacts; snapshot.contact_capacity = 4;
    CHECK( kf_world_save( world, &snapshot ) == KF_OK );
    CHECK( snapshot.contact_count == 1 );
    for( index = 0; index != snapshot.contact_count; ++index )
    {
        uint32_t point;
        for( point = 0; point != contacts[index].count; ++point )
        {
            contacts[index].points[point].normal_impulse = INT64_MAX;
            contacts[index].points[point].tangent_impulse[0] = INT64_MAX;
            contacts[index].points[point].tangent_impulse[1] = -INT64_MAX;
            contacts[index].points[point].tangent_impulse[2] = INT64_MAX;
        }
    }
    CHECK( kf_world_restore( world, &snapshot ) == KF_OK );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
    CHECK( state.velocity.x >= -KF_SPEED_LIMIT && state.velocity.x <= KF_SPEED_LIMIT );
    CHECK( state.velocity.y >= -KF_SPEED_LIMIT && state.velocity.y <= KF_SPEED_LIMIT );
    CHECK( state.velocity.z >= -KF_SPEED_LIMIT && state.velocity.z <= KF_SPEED_LIMIT );
    CHECK( state.angular_velocity.x >= -KF_SPEED_LIMIT && state.angular_velocity.x <= KF_SPEED_LIMIT );
    CHECK( state.angular_velocity.y >= -KF_SPEED_LIMIT && state.angular_velocity.y <= KF_SPEED_LIMIT );
    CHECK( state.angular_velocity.z >= -KF_SPEED_LIMIT && state.angular_velocity.z <= KF_SPEED_LIMIT );
    CHECK( kf_world_save( world, &snapshot ) == KF_OK );
    CHECK( kf_world_restore( world, &snapshot ) == KF_OK );
    CHECK( kf_world_step( world ) == KF_OK );
    kf_world_destroy( world );
}

static kf_handle_t add_log( kf_world_t * world, uint32_t id, kf_vec3_t position, kf_fixed_t yaw )
{
    kf_body_desc_t desc = kf_body_desc_default( id );
    kf_shape_desc_t shape = {0};
    kf_handle_t handle = {0};
    desc.type = KF_BODY_DYNAMIC;
    desc.pose.position = position;
    /* A capsule lying along X: a quarter roll about Z, then a yaw about Y. */
    desc.pose.rotation = kf_rotation_multiply( kf_rotation_from_axis_angle( (kf_vec3_t){0, KF_FIXED_SCALE, 0}, yaw ), kf_rotation_from_axis_angle( (kf_vec3_t){0, 0, KF_FIXED_SCALE}, kf_fixed_from_double( 1.5707963 ) ) );
    desc.friction = R(3, 5);
    shape.id = 1;
    shape.shape.type = KF_SHAPE_CAPSULE;
    shape.shape.data.capsule.position.y = -F(1);
    shape.shape.data.capsule.radius = R(1, 4);
    shape.shape.data.capsule.height = F(2);
    CHECK( kf_world_add_body( world, &desc, &shape, 1, &handle ) == KF_OK );
    return handle;
}

static void test_capsule_stacks( void )
{
    kf_fixed_t yaws[2] = {0, R(1, 200)};
    unsigned variant;
    /* Nearly parallel lying capsules rest on both ends of their overlap. */
    for( variant = 0; variant != 3u; ++variant )
    {
        kf_world_t * world = make_large_world( kf_world_config_default() );
        kf_body_state_t state;
        kf_handle_t top;
        kf_fixed_t expected;
        if( world == NULL ) return;
        (void)add_floor( world, 1 );
        if( variant == 0 )
        {
            (void)add_log( world, 2, (kf_vec3_t){0, R(1, 4), 0}, 0 );
            top = add_log( world, 3, (kf_vec3_t){R(1, 10), R(76, 100), 0}, 0 );
            expected = R(3, 4);
        }
        else
        {
            /* A cradle of two capsules holds a slightly yawed third one. */
            (void)add_log( world, 2, (kf_vec3_t){0, R(1, 4), -R(26, 100)}, 0 );
            (void)add_log( world, 4, (kf_vec3_t){0, R(1, 4), R(26, 100)}, 0 );
            top = add_log( world, 3, (kf_vec3_t){R(1, 10), R(69, 100), 0}, yaws[variant - 1u] );
            expected = kf_fixed_from_double( 0.25 + 0.42708313 );
        }
        CHECK( step_many( world, 60 ) == KF_TRUE );
        CHECK( kf_world_get_body( world, top, &state ) == KF_OK );
        CHECK( state.sleeping == KF_TRUE );
        CHECK( near( state.pose.position.y, expected, R(1, 50) ) == KF_TRUE );
        kf_world_destroy( world );
    }
}

static kf_world_t * restore_mass_fixture( kf_shape_snapshot_t * shapes, size_t count )
{
    kf_world_config_t config = kf_world_config_default();
    kf_world_t * world = NULL;
    kf_object_snapshot_t object = {0};
    kf_snapshot_t snapshot = {0};
    object.desc = kf_body_desc_default( 1 );
    object.desc.type = KF_BODY_DYNAMIC;
    object.state.handle = (kf_handle_t){1, 1};
    object.state.inverse_mass = KF_FIXED_SCALE;
    object.active = KF_TRUE;
    config.gravity = (kf_vec3_t){0, 0, 0};
    config.sleep_ticks = 0;
    snapshot.version = KF_STATE_VERSION;
    snapshot.config = config;
    snapshot.objects = &object;
    snapshot.object_capacity = snapshot.object_count = 1;
    snapshot.shapes = shapes;
    snapshot.shape_capacity = snapshot.shape_count = count;
    CHECK( kf_world_create( &config, NULL, &world ) == KF_OK );
    if( world == NULL ) return NULL;
    CHECK( kf_world_reserve( world, (kf_capacity_t){1, count, 0, 1, 1} ) == KF_OK );
    CHECK( kf_world_restore( world, &snapshot ) == KF_OK );
    return world;
}

static kf_bool_t same_tensor( kf_tensor_t left, kf_tensor_t right, int64_t tolerance )
{
    int64_t differences[6] = {left.xx - right.xx, left.yy - right.yy, left.zz - right.zz, left.xy - right.xy, left.xz - right.xz, left.yz - right.yz};
    unsigned index;
    for( index = 0; index != 6u; ++index )
    {
        if( differences[index] < -tolerance || differences[index] > tolerance ) return KF_FALSE;
    }
    return KF_TRUE;
}

static kf_mass_t fixture_mass( kf_world_t * world )
{
    kf_mass_t mass = {0};
    CHECK( kf_internal_body_mass( world, world->objects, &mass ) == KF_OK );
    return mass;
}

static void test_large_compound_mass( void )
{
    const size_t count = 131072u;
    kf_shape_snapshot_t * shapes = calloc( count, sizeof(*shapes) );
    kf_world_t * world;
    kf_mass_t mass;
    kf_body_state_t state;
    kf_vec3_t center = {F(3) + 1, F(2) + 1, -F(1) - 1};
    kf_handle_t handle = {1, 1};
    /* Uniform box inertia per unit mass is (512^2 + 512^2) / 3.
       The inverse tensor is Q32.32, including the documented inertia floor. */
    const int64_t diagonal = (INT64_C(1) << 43) / 3 + 4096;
    const int64_t inverse = (INT64_C(1) << 56) / diagonal;
    size_t index;
    CHECK( shapes != NULL );
    if( shapes == NULL ) return;
    for( index = 0; index != count; ++index )
    {
        shapes[index].object_id = 1;
        shapes[index].desc.id = (uint32_t)index + 1u;
        shapes[index].desc.shape.type = KF_SHAPE_BOX;
        shapes[index].desc.shape.data.box.center = center;
        shapes[index].desc.shape.data.box.half_extents = (kf_vec3_t){KF_SIZE_LIMIT, KF_SIZE_LIMIT, KF_SIZE_LIMIT};
    }
    world = restore_mass_fixture( shapes, count );
    free( shapes );
    if( world == NULL ) return;
    mass = fixture_mass( world );
    CHECK( mass.inverse_mass == KF_FIXED_SCALE );
    CHECK( kf_vec3_equal( mass.center, center ) == KF_TRUE );
    CHECK( same_tensor( mass.inverse_inertia, (kf_tensor_t){inverse, inverse, inverse, 0, 0, 0}, 1 ) == KF_TRUE );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_apply_impulse_at( world, handle, (kf_vec3_t){F(1), 0, 0}, center ) == KF_OK );
    CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
    CHECK( state.velocity.x == F(1) );
    CHECK( kf_vec3_equal( state.angular_velocity, (kf_vec3_t){0, 0, 0} ) == KF_TRUE );
    center.y += F(512);
    CHECK( kf_world_apply_impulse_at( world, handle, (kf_vec3_t){F(1), 0, 0}, center ) == KF_OK );
    CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
    CHECK( state.velocity.x == F(2) );
    CHECK( near( state.angular_velocity.z, (kf_fixed_t)(-inverse * 512 / 65536), 1 ) == KF_TRUE );
    CHECK( state.angular_velocity.x == 0 && state.angular_velocity.y == 0 );
    kf_world_destroy( world );
}

static void test_compound_mass_invariance( void )
{
    kf_shape_snapshot_t shapes[8] = {{0}};
    kf_mass_t whole = {0};
    kf_vec3_t center = {F(3) + 1, F(2) + 2, -F(1) - 1};
    unsigned variant;
    unsigned index;
    for( index = 0; index != 8u; ++index )
    {
        shapes[index].object_id = 1;
        shapes[index].desc.id = index + 1u;
        shapes[index].desc.shape.type = KF_SHAPE_BOX;
        shapes[index].desc.shape.data.box.center = center;
        shapes[index].desc.shape.data.box.half_extents = (kf_vec3_t){F(2), F(1), F(1)};
    }
    for( variant = 0; variant != 3u; ++variant )
    {
        size_t count = variant == 0u ? 1u : (variant == 1u ? 3u : 8u);
        kf_world_t * world;
        kf_mass_t mass;
        kf_body_state_t state;
        if( variant == 2u )
        {
            /* Eight non-overlapping slabs describe the original box exactly. */
            for( index = 0; index != 8u; ++index )
            {
                shapes[index].desc.shape.data.box.center.x = center.x - F(2) + R(1, 4) + (kf_fixed_t)index * R(1, 2);
                shapes[index].desc.shape.data.box.half_extents.x = R(1, 4);
            }
        }
        world = restore_mass_fixture( shapes, count );
        if( world == NULL ) return;
        mass = fixture_mass( world );
        if( variant == 0u ) whole = mass;
        CHECK( kf_vec3_equal( mass.center, center ) == KF_TRUE );
        CHECK( mass.inverse_mass == whole.inverse_mass );
        CHECK( same_tensor( mass.inverse_inertia, whole.inverse_inertia, variant == 1u ? 0 : 128 ) == KF_TRUE );
        CHECK( kf_world_apply_impulse_at( world, (kf_handle_t){1, 1}, (kf_vec3_t){F(1), 0, 0}, center ) == KF_OK );
        CHECK( kf_world_get_body( world, (kf_handle_t){1, 1}, &state ) == KF_OK );
        CHECK( kf_vec3_equal( state.angular_velocity, (kf_vec3_t){0, 0, 0} ) == KF_TRUE );
        kf_world_destroy( world );
    }
    /* Reassign leaf ids to every permutation of mixed volumes and signed
       first/product moments. Canonical sorting must not change mass. */
    {
        static const unsigned permutations[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
        kf_shape_snapshot_t source[3] = {shapes[0], shapes[1], shapes[2]};
        for( index = 0; index != 3u; ++index )
        {
            source[index].desc.shape.data.box.center = (kf_vec3_t){(kf_fixed_t)(index + 1u) * F(1), index == 0u ? -F(2) : F(1), index == 2u ? -F(1) : F(1)};
            source[index].desc.shape.data.box.half_extents.x = (kf_fixed_t)(index + 1u) * R(1, 4);
        }
        for( variant = 0; variant != 6u; ++variant )
        {
            kf_world_t * world;
            kf_mass_t mass;
            for( index = 0; index != 3u; ++index )
            {
                shapes[index] = source[permutations[variant][index]];
                shapes[index].desc.id = index + 1u;
            }
            world = restore_mass_fixture( shapes, 3 );
            if( world == NULL ) return;
            mass = fixture_mass( world );
            if( variant == 0u ) whole = mass;
            CHECK( kf_vec3_equal( mass.center, (kf_vec3_t){F(14) / 6, R(1, 2), 0} ) == KF_TRUE );
            CHECK( kf_vec3_equal( mass.center, whole.center ) == KF_TRUE );
            CHECK( same_tensor( mass.inverse_inertia, whole.inverse_inertia, 0 ) == KF_TRUE );
            kf_world_destroy( world );
        }
    }
}

int main( void )
{
    test_fast_bounce();
    test_relative_dynamic_motion();
    test_rational_time_sleep_and_impulse();
    test_chain_and_lifecycle();
    test_max_distance_and_generation();
    test_no_allocations_and_rollback();
    test_order_independence();
    test_mass_damping_yaw_and_friction();
    test_sleeping_body_loses_support();
    test_static_velocity_is_ignored();
    test_near_equal_impact_order();
    test_relative_motion_near_world_boundary();
    test_resting_contacts_and_stacks();
    test_restitution_threshold_and_islands();
    test_limits_do_not_stall();
    test_rotation_dynamics();
    test_rotational_continuous_collision();
    test_kinematic_target_and_contact_snapshot();
    test_gravity_scale_and_rotation_locks();
    test_modify_body_and_set_velocity();
    test_spring_constraint();
    test_contact_reports();
    test_trigger_events();
    test_capsule_stacks();
    test_full_turn_continuous_collision();
    test_continuous_path_invalidation();
    test_unreachable_kinematic_target();
    test_dormant_contact_capacity_rollback();
    test_large_cached_impulses_with_small_inertia();
    test_large_compound_mass();
    test_compound_mass_invariance();
    if( failures != 0 ) return 1;
    puts( "kinefix dynamics tests passed" );
    return 0;
}
