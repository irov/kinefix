#include "kinefix/kinefix.h"

#include <stdio.h>
#include <string.h>

#define F(Value) kf_fixed_from_ratio( (Value), 1 )
#define R(Numerator, Denominator) kf_fixed_from_ratio( (Numerator), (Denominator) )
static int failures;
#define CHECK(Expression) do { if( (Expression) == 0 ) { fprintf( stderr, "ccd:%d: %s\n", __LINE__, #Expression ); ++failures; } } while( 0 )

static kf_world_t * make_world( kf_world_config_t config )
{
    kf_world_t * world = NULL;
    CHECK( kf_world_create( &config, NULL, &world ) == KF_OK );
    if( world == NULL ) return NULL;
    CHECK( kf_world_reserve( world, (kf_capacity_t){8, 8, 0, 8, 0} ) == KF_OK );
    return world;
}

static kf_handle_t add_sphere( kf_world_t * world, kf_body_desc_t desc, kf_fixed_t radius )
{
    kf_shape_desc_t shape = {0};
    kf_handle_t handle = {0};
    shape.id = 1;
    shape.shape.type = KF_SHAPE_SPHERE;
    shape.shape.data.sphere.radius = radius;
    CHECK( kf_world_add_body( world, &desc, &shape, 1, &handle ) == KF_OK );
    return handle;
}

static void add_wall( kf_world_t * world, kf_fixed_t minimum, kf_fixed_t maximum )
{
    kf_body_desc_t desc = kf_body_desc_default( 2 );
    kf_shape_desc_t shape = {0};
    kf_handle_t handle;
    shape.id = 1;
    shape.shape.type = KF_SHAPE_AABB;
    shape.shape.data.aabb = (kf_aabb_t){{-F(4), minimum, -F(4)}, {F(4), maximum, F(4)}};
    CHECK( kf_world_add_body( world, &desc, &shape, 1, &handle ) == KF_OK );
}

static kf_bool_t same_state( const kf_body_state_t * left, const kf_body_state_t * right )
{
    unsigned axis;
    if( kf_vec3_equal( left->pose.position, right->pose.position ) == KF_FALSE ) return KF_FALSE;
    if( kf_vec3_equal( left->velocity, right->velocity ) == KF_FALSE ) return KF_FALSE;
    if( kf_vec3_equal( left->angular_velocity, right->angular_velocity ) == KF_FALSE ) return KF_FALSE;
    if( left->pose.rotation.x != right->pose.rotation.x || left->pose.rotation.y != right->pose.rotation.y || left->pose.rotation.z != right->pose.rotation.z ) return KF_FALSE;
    if( left->has_target != right->has_target || left->sleeping != right->sleeping || left->sleep_counter != right->sleep_counter ) return KF_FALSE;
    for( axis = 0; axis != 3u; ++axis )
    {
        if( left->position_remainder[axis] != right->position_remainder[axis] || left->velocity_remainder[axis] != right->velocity_remainder[axis] ) return KF_FALSE;
    }
    return KF_TRUE;
}

static void test_damped_path( void )
{
    unsigned variant;
    for( variant = 0; variant != 3u; ++variant )
    {
        kf_world_config_t config = kf_world_config_default();
        kf_body_desc_t desc = kf_body_desc_default( 1 );
        kf_body_state_t free_state;
        kf_body_state_t ceiling_state;
        kf_world_t * free_world;
        kf_world_t * ceiling_world;
        kf_handle_t free_handle;
        kf_handle_t ceiling_handle;
        config.sleep_ticks = 0;
        config.contact_slop = R(1, 1000);
        if( variant == 1u ) config.step = (kf_timestep_t){1, 1};
        if( variant == 2u ) config.gravity.y = -KF_SPEED_LIMIT;
        desc.type = KF_BODY_DYNAMIC;
        desc.gravity_scale = F(16);
        desc.linear_damping = KF_SPEED_LIMIT;
        desc.friction = 0;
        free_world = make_world( config );
        ceiling_world = make_world( config );
        if( free_world == NULL || ceiling_world == NULL ) return;
        free_handle = add_sphere( free_world, desc, R(1, 100) );
        /* Reverse the insertion order as well as adding the unreachable ceiling. */
        add_wall( ceiling_world, R(3, 100), F(1) );
        ceiling_handle = add_sphere( ceiling_world, desc, R(1, 100) );
        CHECK( kf_world_step( free_world ) == KF_OK );
        CHECK( kf_world_step( ceiling_world ) == KF_OK );
        CHECK( kf_world_get_body( free_world, free_handle, &free_state ) == KF_OK );
        CHECK( kf_world_get_body( ceiling_world, ceiling_handle, &ceiling_state ) == KF_OK );
        CHECK( free_state.pose.position.y < 0 );
        CHECK( same_state( &free_state, &ceiling_state ) == KF_TRUE );
        kf_world_destroy( free_world );
        kf_world_destroy( ceiling_world );
    }
}

static void test_damped_reachable_obstacles( void )
{
    unsigned ceiling;
    for( ceiling = 0; ceiling != 2u; ++ceiling )
    {
        kf_world_config_t config = kf_world_config_default();
        kf_world_t * world;
        kf_body_desc_t desc = kf_body_desc_default( 1 );
        kf_body_state_t state;
        kf_handle_t handle;
        config.sleep_ticks = 0;
        config.contact_slop = R(1, 1000);
        world = make_world( config );
        if( world == NULL ) return;
        desc.type = KF_BODY_DYNAMIC;
        desc.gravity_scale = F(16);
        desc.linear_damping = KF_SPEED_LIMIT;
        desc.friction = 0;
        if( ceiling != 0u ) desc.velocity.y = F(100);
        handle = add_sphere( world, desc, R(1, 100) );
        if( ceiling == 0u ) add_wall( world, -F(1), R(-2, 100) );
        else add_wall( world, R(6, 100), F(1) );
        CHECK( kf_world_step( world ) == KF_OK );
        CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
        if( ceiling == 0u ) CHECK( state.pose.position.y >= R(-1, 100) - config.contact_slop - KF_GEOMETRY_TOLERANCE );
        else CHECK( state.pose.position.y <= R(5, 100) + config.contact_slop + KF_GEOMETRY_TOLERANCE );
        {
            kf_world_t * restored = make_world( config );
            kf_snapshot_t snapshot = {0};
            kf_object_snapshot_t objects[8];
            kf_shape_snapshot_t shapes[8];
            kf_contact_snapshot_t contacts[8];
            unsigned step;
            snapshot.objects = objects; snapshot.object_capacity = 8;
            snapshot.shapes = shapes; snapshot.shape_capacity = 8;
            snapshot.contacts = contacts; snapshot.contact_capacity = 8;
            CHECK( kf_world_save( world, &snapshot ) == KF_OK );
            CHECK( kf_world_restore( restored, &snapshot ) == KF_OK );
            for( step = 0; step != 8u; ++step )
            {
                kf_body_state_t restored_state;
                CHECK( kf_world_step( world ) == KF_OK );
                CHECK( kf_world_step( restored ) == KF_OK );
                CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
                CHECK( kf_world_get_body( restored, handle, &restored_state ) == KF_OK );
                CHECK( same_state( &state, &restored_state ) == KF_TRUE );
            }
            kf_world_destroy( restored );
        }
        kf_world_destroy( world );
    }
}

static void test_parallel_damped_paths( void )
{
    kf_world_config_t config = kf_world_config_default();
    kf_body_desc_t desc = kf_body_desc_default( 1 );
    kf_world_t * isolated;
    kf_world_t * pair;
    kf_handle_t isolated_handle;
    kf_handle_t lower;
    kf_handle_t upper;
    unsigned step;
    config.contact_slop = R(1, 1000);
    config.sleep_ticks = 0;
    isolated = make_world( config );
    pair = make_world( config );
    if( isolated == NULL || pair == NULL ) return;
    desc.type = KF_BODY_DYNAMIC;
    desc.gravity_scale = F(16);
    desc.linear_damping = KF_SPEED_LIMIT;
    desc.friction = 0;
    isolated_handle = add_sphere( isolated, desc, R(1, 100) );
    lower = add_sphere( pair, desc, R(1, 100) );
    desc.id = 3;
    desc.pose.position.y = R(5, 100);
    upper = add_sphere( pair, desc, R(1, 100) );
    for( step = 0; step != 20u; ++step )
    {
        kf_body_state_t isolated_state;
        kf_body_state_t lower_state;
        kf_body_state_t upper_state;
        CHECK( kf_world_step( isolated ) == KF_OK );
        CHECK( kf_world_step( pair ) == KF_OK );
        CHECK( kf_world_get_body( isolated, isolated_handle, &isolated_state ) == KF_OK );
        CHECK( kf_world_get_body( pair, lower, &lower_state ) == KF_OK );
        CHECK( kf_world_get_body( pair, upper, &upper_state ) == KF_OK );
        CHECK( same_state( &isolated_state, &lower_state ) == KF_TRUE );
        CHECK( upper_state.pose.position.y - lower_state.pose.position.y == R(5, 100) );
        CHECK( kf_vec3_equal( upper_state.velocity, lower_state.velocity ) == KF_TRUE );
    }
    kf_world_destroy( isolated );
    kf_world_destroy( pair );
}

static void test_returning_path( void )
{
    kf_world_config_t config = kf_world_config_default();
    kf_world_t * world;
    kf_body_desc_t desc = kf_body_desc_default( 1 );
    kf_handle_t handle;
    kf_body_state_t state;
    config.step = (kf_timestep_t){1, 1};
    config.gravity.y = -F(8);
    config.sleep_ticks = 0;
    config.contact_slop = R(1, 1000);
    world = make_world( config );
    if( world == NULL ) return;
    desc.type = KF_BODY_DYNAMIC;
    desc.velocity.y = R(9, 2);
    desc.friction = 0;
    handle = add_sphere( world, desc, R(1, 10) );
    add_wall( world, R(1, 2), F(1) );
    /* Free integration rises to one unit then returns exactly to its start. */
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
    CHECK( state.pose.position.y < 0 );
    CHECK( state.velocity.y < R(-7, 2) );
    kf_world_destroy( world );
}

static void test_fast_kinematic( void )
{
    unsigned variant;
    for( variant = 0; variant != 8u; ++variant )
    {
        kf_world_config_t config = kf_world_config_default();
        kf_world_t * first;
        kf_world_t * restored;
        kf_body_desc_t pusher = kf_body_desc_default( 1 );
        kf_body_desc_t ball = kf_body_desc_default( 2 );
        kf_handle_t pusher_handle;
        kf_handle_t ball_handle;
        kf_body_state_t pusher_state;
        kf_body_state_t ball_state;
        kf_body_state_t restored_state;
        kf_snapshot_t snapshot = {0};
        kf_object_snapshot_t objects[8];
        kf_shape_snapshot_t shapes[8];
        kf_contact_snapshot_t contacts[8];
        config.gravity = (kf_vec3_t){0, 0, 0};
        config.step = (kf_timestep_t){1, 1};
        config.sleep_ticks = 0;
        first = make_world( config );
        restored = make_world( config );
        if( first == NULL || restored == NULL ) return;
        pusher.type = KF_BODY_KINEMATIC;
        if( (variant & 4u) != 0u ) pusher.id = 9;
        pusher.velocity.x = KF_SPEED_LIMIT;
        pusher.friction = 0;
        ball.type = KF_BODY_DYNAMIC;
        ball.pose.position.x = F(600);
        ball.friction = 0;
        if( (variant & 1u) == 0u )
        {
            pusher_handle = add_sphere( first, pusher, R(1, 4) );
            ball_handle = add_sphere( first, ball, R(1, 4) );
        }
        else
        {
            ball_handle = add_sphere( first, ball, R(1, 4) );
            pusher_handle = add_sphere( first, pusher, R(1, 4) );
        }
        if( (variant & 2u) != 0u )
        {
            kf_pose_t target = {{KF_SPEED_LIMIT, 0, 0}, {0, 0, 0}};
            CHECK( kf_world_set_kinematic_target( first, pusher_handle, target ) == KF_OK );
        }
        if( (variant & 4u) != 0u )
        {
            CHECK( kf_world_get_body( first, ball_handle, &ball_state ) == KF_OK );
            ball_state.sleeping = KF_TRUE;
            CHECK( kf_world_set_body( first, &ball_state ) == KF_OK );
        }
        snapshot.objects = objects; snapshot.object_capacity = 8;
        snapshot.shapes = shapes; snapshot.shape_capacity = 8;
        snapshot.contacts = contacts; snapshot.contact_capacity = 8;
        CHECK( kf_world_save( first, &snapshot ) == KF_OK );
        CHECK( kf_world_restore( restored, &snapshot ) == KF_OK );
        CHECK( kf_world_step( first ) == KF_OK );
        CHECK( kf_world_step( restored ) == KF_OK );
        CHECK( kf_world_get_body( first, pusher_handle, &pusher_state ) == KF_OK );
        CHECK( kf_world_get_body( first, ball_handle, &ball_state ) == KF_OK );
        CHECK( kf_world_get_body( restored, ball_handle, &restored_state ) == KF_OK );
        CHECK( same_state( &ball_state, &restored_state ) == KF_TRUE );
        CHECK( pusher_state.pose.position.x == KF_SPEED_LIMIT );
        CHECK( ball_state.pose.position.x >= KF_SPEED_LIMIT + R(1, 2) - config.contact_slop - KF_GEOMETRY_TOLERANCE );
        CHECK( ball_state.velocity.x == KF_SPEED_LIMIT );
        kf_world_destroy( first );
        kf_world_destroy( restored );
    }
}

static void test_kinematic_stops_at_world_boundary( void )
{
    kf_world_config_t config = kf_world_config_default();
    kf_world_t * world;
    kf_body_desc_t pusher = kf_body_desc_default( 1 );
    kf_body_desc_t ball = kf_body_desc_default( 2 );
    kf_handle_t pusher_handle;
    kf_handle_t ball_handle;
    kf_body_state_t pusher_state;
    kf_body_state_t ball_state;
    config.gravity = (kf_vec3_t){0, 0, 0};
    config.step = (kf_timestep_t){1, 1};
    config.sleep_ticks = 0;
    world = make_world( config );
    if( world == NULL ) return;
    pusher.type = KF_BODY_KINEMATIC;
    pusher.pose.position.x = F(3200);
    pusher.velocity.x = KF_SPEED_LIMIT;
    pusher.friction = 0;
    ball.type = KF_BODY_DYNAMIC;
    ball.pose.position.x = F(3800);
    ball.friction = 0;
    pusher_handle = add_sphere( world, pusher, R(1, 4) );
    ball_handle = add_sphere( world, ball, R(1, 4) );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_get_body( world, pusher_handle, &pusher_state ) == KF_OK );
    CHECK( kf_world_get_body( world, ball_handle, &ball_state ) == KF_OK );
    CHECK( pusher_state.pose.position.x == KF_COORDINATE_LIMIT - R(1, 4) );
    CHECK( pusher_state.velocity.x == 0 );
    CHECK( ball_state.pose.position.x >= F(4095) );
    CHECK( ball_state.pose.position.x <= KF_COORDINATE_LIMIT - R(1, 4) );
    CHECK( ball_state.velocity.x == 0 );
    kf_world_destroy( world );
}

static kf_world_t * make_support_world( kf_world_config_t config, unsigned variant, kf_bool_t reversed, kf_handle_t * ball )
{
    kf_body_desc_t bodies[4];
    kf_shape_desc_t shapes[4] = {0};
    kf_world_t * world = make_world( config );
    unsigned count = variant == 4u ? 4u : (variant == 6u ? 2u : 3u);
    unsigned index;
    if( world == NULL ) return NULL;
    for( index = 0; index != count; ++index )
    {
        bodies[index] = kf_body_desc_default( index + 1u );
        bodies[index].friction = 0;
        shapes[index].id = 1;
        shapes[index].shape.type = KF_SHAPE_AABB;
    }
    bodies[0].type = KF_BODY_DYNAMIC;
    bodies[0].pose.position = (kf_vec3_t){R(-2, 5), R(1, 5), 0};
    bodies[0].velocity = (kf_vec3_t){F(20), -F(10), 0};
    bodies[0].gravity_scale = F(16);
    bodies[0].rotation_locks = KF_LOCK_ROTATION_X | KF_LOCK_ROTATION_Y | KF_LOCK_ROTATION_Z;
    shapes[0].shape.type = KF_SHAPE_SPHERE;
    shapes[0].shape.data.sphere.radius = R(1, 100);
    if( variant == 1u )
    {
        bodies[0].pose.position.y = R(1, 100);
        bodies[0].velocity.y = 0;
    }
    if( variant == 4u )
    {
        bodies[0].pose.position.z = R(-2, 5);
        bodies[0].velocity.z = F(20);
    }
    if( variant >= 5u ) bodies[0].restitution = F(1);
    if( variant == 2u || variant == 3u ) bodies[1].type = KF_BODY_KINEMATIC;
    if( variant == 3u ) bodies[1].velocity.y = R(1, 4);
    shapes[1].shape.data.aabb = (kf_aabb_t){{-F(10), R(-1, 50), -F(10)}, {F(10), 0, F(10)}};
    shapes[2].shape.data.aabb = (kf_aabb_t){{0, 0, -F(10)}, {F(1), F(10), F(10)}};
    shapes[3].shape.data.aabb = (kf_aabb_t){{-F(10), 0, 0}, {F(10), F(10), F(1)}};
    for( index = 0; index != count; ++index )
    {
        unsigned selected = reversed == KF_TRUE ? count - index - 1u : index;
        kf_handle_t handle;
        CHECK( kf_world_add_body( world, bodies + selected, shapes + selected, 1, &handle ) == KF_OK );
        if( selected == 0u ) *ball = handle;
    }
    return world;
}

static void test_replayed_supports( void )
{
    kf_body_state_t bounced = {0};
    unsigned variant;
    for( variant = 0; variant != 7u; ++variant )
    {
        kf_world_config_t config = kf_world_config_default();
        kf_body_state_t first = {0};
        kf_fixed_t floor_end = variant == 3u ? R(1, 100) : 0;
        unsigned order;
        config.sleep_ticks = 0;
        config.contact_slop = R(1, 1000);
        for( order = 0; order != 2u; ++order )
        {
            kf_handle_t ball;
            kf_body_state_t state;
            kf_world_t * world = make_support_world( config, variant, (kf_bool_t)order, &ball );
            if( world == NULL ) return;
            CHECK( kf_world_step( world ) == KF_OK );
            CHECK( kf_world_get_body( world, ball, &state ) == KF_OK );
            CHECK( state.pose.position.y >= floor_end + R(1, 100) - config.contact_slop - KF_GEOMETRY_TOLERANCE );
            CHECK( state.velocity.y >= 0 );
            if( variant != 6u ) CHECK( state.pose.position.x <= -R(1, 100) + config.contact_slop + KF_GEOMETRY_TOLERANCE );
            if( variant == 4u ) CHECK( state.pose.position.z <= -R(1, 100) + config.contact_slop + KF_GEOMETRY_TOLERANCE );
            if( variant == 5u ) bounced = state;
            if( variant == 6u )
            {
                /* A side impact reverses X without changing the floor rebound. */
                CHECK( state.pose.position.y == bounced.pose.position.y );
                CHECK( state.velocity.y == bounced.velocity.y );
                CHECK( state.velocity.x == -bounced.velocity.x );
            }
            if( order == 0u ) first = state;
            else CHECK( same_state( &first, &state ) == KF_TRUE );
            kf_world_destroy( world );
        }
    }
}

static void test_partner_impact_budget( void )
{
    unsigned variant;
    for( variant = 0; variant != 4u; ++variant )
    {
        kf_world_config_t config = kf_world_config_default();
        kf_world_t * world;
        kf_handle_t handles[3];
        kf_body_state_t state;
        kf_body_state_t incoming;
        kf_bool_t mirrored = (kf_bool_t)(variant & 1u);
        unsigned index;
        config.step = (kf_timestep_t){1, 60};
        config.gravity = (kf_vec3_t){0, 0, 0};
        config.ccd_iterations = 1;
        config.sleep_ticks = 0;
        world = make_world( config );
        if( world == NULL ) return;
        for( index = 0; index != 3u; ++index )
        {
            kf_body_desc_t desc = kf_body_desc_default( index + 1u );
            if( (variant & 2u) != 0u ) desc.id = index == 2u ? 1u : index + 2u;
            desc.type = KF_BODY_DYNAMIC;
            desc.restitution = F(1);
            desc.friction = 0;
            desc.rotation_locks = KF_LOCK_ROTATION_X | KF_LOCK_ROTATION_Y | KF_LOCK_ROTATION_Z;
            desc.pose.position.x = index == 0u ? -F(1) : (index == 1u ? F(1) : 0);
            desc.velocity.x = index == 0u ? F(200) : (index == 1u ? -F(200) : F(100));
            if( mirrored == KF_TRUE )
            {
                desc.pose.position.x = -desc.pose.position.x;
                desc.velocity.x = -desc.velocity.x;
            }
            handles[index] = add_sphere( world, desc, R(1, 10) );
        }
        CHECK( kf_world_step( world ) == KF_OK );
        CHECK( kf_world_get_body( world, handles[2], &state ) == KF_OK );
        CHECK( kf_world_get_body( world, handles[0], &incoming ) == KF_OK );
        CHECK( kf_fixed_abs( state.pose.position.x ) < R(1, 2) );
        CHECK( state.velocity.x == (mirrored == KF_FALSE ? -F(200) : F(200)) );
        CHECK( (mirrored == KF_FALSE ? state.pose.position.x - incoming.pose.position.x : incoming.pose.position.x - state.pose.position.x) >= R(1, 5) - config.contact_slop - KF_GEOMETRY_TOLERANCE );
        kf_world_destroy( world );
    }
}

static void test_compound_leaf_contacts( void )
{
    unsigned variant;
    for( variant = 0; variant != 16u; ++variant )
    {
        kf_world_config_t config = kf_world_config_default();
        kf_body_desc_t wall = kf_body_desc_default( (variant & 4u) == 0u ? 2u : 1u );
        kf_body_desc_t body = kf_body_desc_default( (variant & 4u) == 0u ? 1u : 2u );
        kf_shape_desc_t wall_shape = {0};
        kf_shape_desc_t spheres[2] = {{0}, {0}};
        kf_world_t * world;
        kf_handle_t wall_handle;
        kf_handle_t handle;
        kf_body_state_t state;
        kf_bool_t mirrored = (kf_bool_t)(variant & 1u);
        unsigned index;
        config.gravity = (kf_vec3_t){0, 0, 0};
        config.sleep_ticks = 0;
        config.contact_slop = R(1, 1000);
        world = make_world( config );
        if( world == NULL ) return;
        wall_shape.id = 1;
        wall_shape.shape.type = KF_SHAPE_AABB;
        wall_shape.shape.data.aabb = (kf_aabb_t){{0, 0, -F(1)}, {R(1, 10), F(1), F(1)}};
        if( mirrored == KF_TRUE )
        {
            wall_shape.shape.data.aabb.minimum.x = -R(1, 10);
            wall_shape.shape.data.aabb.maximum.x = 0;
        }
        if( (variant & 8u) != 0u ) wall.type = KF_BODY_KINEMATIC;
        body.type = KF_BODY_DYNAMIC;
        body.friction = 0;
        body.pose.position.x = mirrored == KF_FALSE ? -F(1) : F(1);
        body.velocity.x = mirrored == KF_FALSE ? F(100) : -F(100);
        body.rotation_locks = KF_LOCK_ROTATION_X | KF_LOCK_ROTATION_Y | KF_LOCK_ROTATION_Z;
        for( index = 0; index != 2u; ++index )
        {
            spheres[index].id = (variant & 2u) == 0u ? index + 1u : 2u - index;
            spheres[index].shape.type = KF_SHAPE_SPHERE;
            spheres[index].shape.data.sphere.radius = R(1, 10);
        }
        /* The first leaf already slides on the wall's top. That contact must
           never exempt the second leaf from hitting the wall's side. */
        spheres[0].shape.data.sphere.position = (kf_vec3_t){mirrored == KF_FALSE ? R(105, 100) : -R(105, 100), R(11, 10), 0};
        spheres[1].shape.data.sphere.position = (kf_vec3_t){0, R(1, 2), 0};
        if( (variant & 4u) == 0u )
        {
            CHECK( kf_world_add_body( world, &wall, &wall_shape, 1, &wall_handle ) == KF_OK );
        }
        CHECK( kf_world_add_body( world, &body, spheres, 2, &handle ) == KF_OK );
        if( (variant & 4u) != 0u )
        {
            CHECK( kf_world_add_body( world, &wall, &wall_shape, 1, &wall_handle ) == KF_OK );
        }
        CHECK( kf_world_step( world ) == KF_OK );
        CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
        CHECK( (mirrored == KF_FALSE ? -state.pose.position.x : state.pose.position.x) >= R(9, 100) );
        CHECK( kf_fixed_abs( state.velocity.x ) < R(1, 100) );
        kf_world_destroy( world );
    }
}

int main( void )
{
    test_damped_path();
    test_damped_reachable_obstacles();
    test_parallel_damped_paths();
    test_returning_path();
    test_fast_kinematic();
    test_kinematic_stops_at_world_boundary();
    test_replayed_supports();
    test_partner_impact_budget();
    test_compound_leaf_contacts();
    if( failures != 0 ) return 1;
    puts( "ccd tests passed" );
    return 0;
}
