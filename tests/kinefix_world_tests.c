#include "kinefix/kinefix.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define I(x) ((x) * KF_FIXED_SCALE)
#define CHECK(c) do { if( !(c) ) { fprintf( stderr, "%s:%d: %s\n", __FILE__, __LINE__, #c ); exit(1); } } while(0)

static kf_shape_t sphere( kf_fixed_t radius )
{
    kf_shape_t shape = {0};
    shape.type = KF_SHAPE_SPHERE;
    shape.data.sphere.radius = radius;
    return shape;
}

static kf_shape_t box( kf_vec3_t minimum, kf_vec3_t maximum )
{
    kf_shape_t shape = {0};
    shape.type = KF_SHAPE_AABB;
    shape.data.aabb = (kf_aabb_t){minimum, maximum};
    return shape;
}

static kf_world_t * create_world( void )
{
    kf_world_t * world = NULL;
    kf_world_config_t config = kf_world_config_default();
    config.gravity = (kf_vec3_t){0, 0, 0};
    CHECK( kf_world_create( &config, NULL, &world ) == KF_OK );
    CHECK( kf_world_reserve( world, (kf_capacity_t){32, 64, 8, 0, 0} ) == KF_OK );
    return world;
}

static kf_handle_t add( kf_world_t * world, uint32_t id, kf_fixed_t x, kf_bool_t trigger )
{
    kf_body_desc_t desc = kf_body_desc_default( id );
    kf_shape_desc_t shapes[2] = {{2, {0}}, {1, {0}}};
    kf_handle_t handle;
    desc.pose.position.x = x;
    desc.trigger = trigger;
    shapes[0].shape = box( (kf_vec3_t){0, -I(2), -I(2)}, (kf_vec3_t){I(1), I(2), I(2)} );
    shapes[1].shape = shapes[0].shape;
    CHECK( kf_world_add_body( world, &desc, shapes, 2, &handle ) == KF_OK );
    return handle;
}

static void test_queries( void )
{
    kf_world_t * world = create_world();
    kf_handle_t second = add( world, 20, I(2), KF_FALSE );
    kf_handle_t first = add( world, 10, 0, KF_FALSE );
    kf_handle_t trigger = add( world, 5, -I(2), KF_TRUE );
    kf_shape_t moving = sphere( KF_FIXED_SCALE / 4 );
    kf_pose_t pose = {{-I(5), 0, 0}, {0, 0, 0}};
    kf_vec3_t delta = {I(10), 0, 0};
    kf_sweep_hit_t hits[8];
    kf_query_filter_t filter = kf_query_filter_default();
    size_t count;
    kf_ray_t ray;
    kf_handle_t region_objects[4];
    kf_aabb_t region = {{-I(1), -I(1), -I(1)}, {I(4), I(1), I(1)}};
    kf_aabb_t full_region = {{-KF_COORDINATE_LIMIT, -KF_COORDINATE_LIMIT, -KF_COORDINATE_LIMIT}, {KF_COORDINATE_LIMIT, KF_COORDINATE_LIMIT, KF_COORDINATE_LIMIT}};
    kf_ray_t long_ray = {{-I(4000), 0, 0}, {I(1), 0, 0}, I(8000)};
    kf_query_t batch[2] = {0};
    kf_handle_t replacement;
    kf_body_state_t state;
    CHECK( kf_world_sweep_all( world, &moving, pose, delta, &filter, hits, 8, &count ) == KF_OK );
    CHECK( count == 4 );
    CHECK( hits[0].object.id == 10 && hits[0].shape_id == 1 );
    CHECK( hits[1].object.id == 10 && hits[1].shape_id == 2 );
    CHECK( hits[2].object.id == 20 );
    CHECK( hits[0].position.x < 0 );
    CHECK( kf_fixed_abs( hits[0].contact_point.x ) <= KF_GEOMETRY_TOLERANCE * 2 );
    CHECK( kf_fixed_abs( hits[0].position.x + moving.data.sphere.radius ) <= 32 );
    CHECK( kf_world_sweep_all( world, &moving, pose, delta, &filter, hits, 1, &count ) == KF_CAPACITY_EXCEEDED );
    CHECK( count == 4 && hits[0].object.id == 10 );
    CHECK( kf_world_sweep_all( world, &moving, pose, delta, &filter, NULL, 0, &count ) == KF_CAPACITY_EXCEEDED );
    CHECK( count == 4 );
    filter.include_triggers = KF_TRUE;
    CHECK( kf_world_sweep_first( world, &moving, pose, delta, &filter, hits ) == KF_OK );
    CHECK( hits[0].object.id == trigger.id && hits[0].trigger == KF_TRUE );
    filter.exclude = &trigger;
    filter.exclude_count = 1;
    CHECK( kf_world_sweep_first( world, &moving, pose, delta, &filter, hits ) == KF_OK );
    CHECK( hits[0].object.id == first.id );
    filter.exclude = &first;
    CHECK( kf_world_sweep_first( world, &moving, pose, delta, &filter, hits ) == KF_OK );
    CHECK( hits[0].object.id == trigger.id );
    ray = (kf_ray_t){pose.position, {I(1), 0, 0}, I(10)};
    filter = kf_query_filter_default();
    CHECK( kf_world_raycast_all( world, &ray, &filter, hits, 8, &count ) == KF_OK );
    CHECK( count == 4 );
    CHECK( kf_fixed_abs( hits[0].contact_point.x ) <= 32 );
    CHECK( kf_world_query_region( world, region, &filter, region_objects, 1, &count ) == KF_CAPACITY_EXCEEDED );
    CHECK( count == 2 && region_objects[0].id == first.id );
    CHECK( kf_world_query_region( world, region, &filter, region_objects, 4, &count ) == KF_OK );
    CHECK( count == 2 && region_objects[1].id == second.id );
    CHECK( kf_world_query_region( world, full_region, &filter, region_objects, 4, &count ) == KF_OK );
    CHECK( count == 2 && region_objects[0].id == first.id && region_objects[1].id == second.id );
    CHECK( kf_world_raycast_first( world, &long_ray, &filter, hits ) == KF_OK );
    CHECK( hits[0].object.id == first.id );

    batch[0].type = KF_QUERY_RAY;
    batch[0].ray = ray;
    batch[0].filter = filter;
    batch[0].output = hits;
    batch[0].capacity = 8;
    batch[1].type = KF_QUERY_REGION;
    batch[1].region = region;
    batch[1].filter = filter;
    batch[1].output = region_objects;
    batch[1].capacity = 4;
    CHECK( kf_world_query_batch( world, batch, 2 ) == KF_OK );
    CHECK( batch[0].status == KF_OK && batch[0].required == 4 );
    CHECK( batch[1].status == KF_OK && batch[1].required == 2 );
    CHECK( kf_world_remove_body( world, first ) == KF_OK );
    CHECK( kf_world_remove_body( world, first ) == KF_STALE_HANDLE );
    replacement = add( world, 10, 0, KF_FALSE );
    CHECK( replacement.generation != first.generation );
    CHECK( kf_world_get_body( world, first, &state ) == KF_STALE_HANDLE );
    CHECK( kf_world_get_body( world, replacement, &state ) == KF_OK );
    state.pose.position.x = I(50);
    CHECK( kf_world_set_body( world, &state ) == KF_OK );
    CHECK( kf_world_sweep_first( world, &moving, pose, delta, &filter, hits ) == KF_OK );
    CHECK( hits[0].object.id == second.id );
    kf_world_destroy( world );
}

static void test_subfraction_hit_order( void )
{
    kf_world_t * world = create_world();
    kf_body_desc_t desc = kf_body_desc_default( 1 );
    kf_shape_desc_t wall = {1, {0}};
    kf_handle_t handle;
    kf_shape_t moving = sphere( 0 );
    kf_pose_t pose = {{-I(4000), 0, 0}, {0, 0, 0}};
    kf_sweep_hit_t hits[2];
    size_t count;
    wall.shape = box( (kf_vec3_t){0, -I(1), -I(1)}, (kf_vec3_t){64, I(1), I(1)} );
    desc.pose.position.x = 65;
    CHECK( kf_world_add_body( world, &desc, &wall, 1, &handle ) == KF_OK );
    desc.id = 20;
    desc.pose.position.x = 1;
    CHECK( kf_world_add_body( world, &desc, &wall, 1, &handle ) == KF_OK );
    CHECK( kf_world_sweep_all( world, &moving, pose, (kf_vec3_t){I(8000), 0, 0}, NULL, hits, 2, &count ) == KF_OK );
    CHECK( count == 2 );
    CHECK( hits[0].fraction == hits[1].fraction );
    CHECK( hits[0].fraction_q32 < hits[1].fraction_q32 );
    CHECK( hits[0].object.id == 20 );
    CHECK( kf_world_sweep_first( world, &moving, pose, (kf_vec3_t){I(8000), 0, 0}, NULL, hits ) == KF_OK );
    CHECK( hits[0].object.id == 20 );
    kf_world_destroy( world );
}

static void test_filters_contacts_relative( void )
{
    kf_world_t * world = create_world();
    kf_body_desc_t desc = kf_body_desc_default( 1 );
    kf_shape_desc_t shape = {1, {0}};
    kf_handle_t handle;
    kf_shape_t moving = sphere( I(1) );
    kf_query_filter_t filter = {4, 2, NULL, 0, KF_FALSE};
    kf_contact_t contacts[2];
    kf_pose_t pose = {{KF_FIXED_SCALE / 2, 0, 0}, {0, 0, 0}};
    size_t count;
    kf_vec3_t correction;
    kf_pose_t target_pose = {{I(10), 0, 0}, {0, 0, 0}};
    kf_sweep_hit_t hit;
    kf_vec3_t relative_centers;
    kf_vec3_t target_center;
    desc.category = 2;
    desc.mask = 4;
    shape.shape = sphere( I(1) );
    CHECK( kf_world_add_body( world, &desc, &shape, 1, &handle ) == KF_OK );
    CHECK( kf_world_contacts( world, &moving, pose, &filter, contacts, 2, &count ) == KF_OK );
    CHECK( count == 1 && contacts[0].normal.x > KF_FIXED_SCALE - 16 );
    CHECK( kf_fixed_abs( contacts[0].depth - (I(1) + KF_FIXED_SCALE / 2) ) < 16 );
    filter.category = 1;
    CHECK( kf_world_contacts( world, &moving, pose, &filter, contacts, 2, &count ) == KF_NO_HIT );
    filter.category = 4;
    CHECK( kf_world_depenetrate( world, &moving, &pose, &filter, 8, I(3), &correction ) == KF_OK );
    CHECK( pose.position.x >= I(2) - KF_GEOMETRY_TOLERANCE );
    CHECK( correction.x > I(1) );
    pose.position = (kf_vec3_t){-I(10), 0, 0};
    CHECK( kf_shape_sweep_relative( &moving, pose, (kf_vec3_t){I(15), 0, 0}, &moving, target_pose, (kf_vec3_t){-I(15), 0, 0}, &hit ) == KF_OK );
    CHECK( kf_fixed_abs( hit.fraction - KF_FIXED_SCALE * 3 / 5 ) < 16 );
    CHECK( kf_fixed_abs( hit.position.x + I(1) ) < 64 );
    CHECK( kf_fixed_abs( hit.contact_point.x ) < 64 );
    moving = sphere( KF_FIXED_SCALE / 4 );
    pose.position = (kf_vec3_t){-I(1000), 0, 0};
    target_pose.position = (kf_vec3_t){0, -I(1000), 0};
    CHECK( kf_shape_sweep_relative( &moving, pose, (kf_vec3_t){I(2000), 0, 0}, &moving, target_pose, (kf_vec3_t){0, I(2000), 0}, &hit ) == KF_OK );
    relative_centers = (kf_vec3_t){hit.position.x, -hit.position.x, 0};
    CHECK( kf_fixed_abs( kf_vec3_length( relative_centers ) - KF_FIXED_SCALE / 2 ) <= 32 );
    target_center = (kf_vec3_t){0, hit.position.x, 0};
    CHECK( kf_fixed_abs( kf_vec3_length( kf_vec3_sub( hit.contact_point, target_center ) ) - KF_FIXED_SCALE / 4 ) <= 32 );
    pose.position = (kf_vec3_t){I(3000), 0, 0};
    target_pose.position = (kf_vec3_t){I(3500), 0, 0};
    CHECK( kf_shape_sweep_relative( &moving, pose, (kf_vec3_t){I(1000), 0, 0}, &moving, target_pose, (kf_vec3_t){-I(7500), 0, 0}, &hit ) == KF_OK );
    CHECK( hit.position.x > I(3000) && hit.position.x < I(3100) );
    CHECK( kf_fixed_abs( hit.contact_point.x - hit.position.x - KF_FIXED_SCALE / 4 ) <= 32 );
    CHECK( kf_shape_sweep_relative( &moving, pose, (kf_vec3_t){0, 0, 0}, &moving, target_pose, (kf_vec3_t){INT32_MIN, 0, 0}, &hit ) == KF_OUT_OF_RANGE );

    kf_world_destroy( world );
}

static void test_curve( void )
{
    kf_world_t * world = create_world();
    kf_shape_t moving = sphere( KF_FIXED_SCALE / 10 );
    kf_pose_t pose = {{-I(5), I(5), 0}, {0, 0, 0}};
    kf_sweep_hit_t hits[64];
    size_t count;
    (void)add( world, 9, 0, KF_FALSE );
    CHECK( kf_world_sweep_curve( world, &moving, pose, (kf_vec3_t){0, -I(5), 0}, (kf_vec3_t){I(5), I(5), 0}, 16, 1, NULL, hits, 64, &count ) == KF_ITERATION_LIMIT );
    CHECK( kf_world_sweep_curve( world, &moving, pose, (kf_vec3_t){0, -I(5), 0}, (kf_vec3_t){I(5), I(5), 0}, KF_FIXED_SCALE / 100, 128, NULL, hits, 64, &count ) == KF_OK );
    CHECK( count >= 2 );
    CHECK( hits[0].object.id == 9 && hits[0].fraction > 0 );
    kf_world_destroy( world );
    world = create_world();
    pose.position = (kf_vec3_t){0, 0, 0};
    CHECK( kf_world_sweep_curve( world, &moving, pose, (kf_vec3_t){I(2048), 0, 0}, (kf_vec3_t){KF_COORDINATE_LIMIT, 0, 0}, KF_FIXED_SCALE / 100, 128, NULL, hits, 64, &count ) == KF_OUT_OF_RANGE );
    CHECK( count == 0 );
    kf_world_destroy( world );
}

static uint64_t hash_word( uint64_t hash, uint64_t word )
{
    return (hash ^ word) * UINT64_C(1099511628211);
}

static uint64_t hash_world( kf_world_t * world )
{
    kf_snapshot_t snapshot = {0};
    kf_object_snapshot_t objects[32];
    kf_shape_snapshot_t shapes[64];
    kf_constraint_snapshot_t constraints[8];
    kf_contact_snapshot_t contacts[64];
    uint64_t hash = UINT64_C(1469598103934665603);
    size_t index;
    snapshot.objects = objects; snapshot.object_capacity = 32;
    snapshot.shapes = shapes; snapshot.shape_capacity = 64;
    snapshot.constraints = constraints; snapshot.constraint_capacity = 8;
    snapshot.contacts = contacts; snapshot.contact_capacity = 64;
    CHECK( kf_world_save( world, &snapshot ) == KF_OK );
    hash = hash_word( hash, snapshot.tick );
    for( index = 0; index != snapshot.object_count; ++index )
    {
        const kf_body_state_t * state = &objects[index].state;
        size_t axis;
        hash = hash_word( hash, objects[index].active );
        hash = hash_word( hash, state->handle.id );
        hash = hash_word( hash, state->handle.generation );
        hash = hash_word( hash, (uint32_t)state->pose.position.x );
        hash = hash_word( hash, (uint32_t)state->pose.position.y );
        hash = hash_word( hash, (uint32_t)state->pose.position.z );
        hash = hash_word( hash, (uint32_t)state->pose.rotation.x );
        hash = hash_word( hash, (uint32_t)state->pose.rotation.y );
        hash = hash_word( hash, (uint32_t)state->pose.rotation.z );
        hash = hash_word( hash, (uint32_t)state->velocity.x );
        hash = hash_word( hash, (uint32_t)state->velocity.y );
        hash = hash_word( hash, (uint32_t)state->velocity.z );
        hash = hash_word( hash, (uint32_t)state->angular_velocity.x );
        hash = hash_word( hash, (uint32_t)state->angular_velocity.y );
        hash = hash_word( hash, (uint32_t)state->angular_velocity.z );
        for( axis = 0; axis != 3; ++axis )
        {
            hash = hash_word( hash, (uint64_t)state->position_remainder[axis] );
            hash = hash_word( hash, (uint64_t)state->velocity_remainder[axis] );
        }
        hash = hash_word( hash, state->has_target );
        hash = hash_word( hash, state->sleep_counter );
        hash = hash_word( hash, state->sleeping );
    }
    for( index = 0; index != snapshot.constraint_count; ++index )
    {
        const kf_constraint_state_t * state = &constraints[index].state;
        hash = hash_word( hash, constraints[index].active );
        hash = hash_word( hash, state->handle.id );
        hash = hash_word( hash, state->handle.generation );
        hash = hash_word( hash, (uint32_t)state->desc.type );
        hash = hash_word( hash, state->desc.left.id );
        hash = hash_word( hash, state->desc.left.generation );
        hash = hash_word( hash, state->desc.right.id );
        hash = hash_word( hash, state->desc.right.generation );
        hash = hash_word( hash, (uint32_t)state->desc.length );
        hash = hash_word( hash, (uint32_t)state->impulse );
    }
    return hash;
}

static void test_target_bounds( void )
{
    kf_world_t * world = create_world();
    kf_world_t * restored = create_world();
    kf_body_desc_t desc = kf_body_desc_default( 1 );
    kf_shape_desc_t shape = {1, {0}};
    kf_handle_t handle;
    kf_body_state_t state;
    kf_snapshot_t snapshot = {0};
    kf_object_snapshot_t objects[1];
    kf_shape_snapshot_t shapes[1];
    uint64_t before;
    desc.type = KF_BODY_KINEMATIC;
    desc.pose.position.x = KF_COORDINATE_LIMIT - I(2);
    shape.shape = sphere( KF_FIXED_SCALE / 2 );
    CHECK( kf_world_add_body( world, &desc, &shape, 1, &handle ) == KF_OK );
    CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
    before = hash_world( world );
    state.has_target = KF_TRUE;
    state.target = state.pose;
    state.target.position.x = KF_COORDINATE_LIMIT;
    CHECK( kf_world_set_kinematic_target( world, handle, state.target ) == KF_OUT_OF_RANGE );
    CHECK( kf_world_set_body( world, &state ) == KF_OUT_OF_RANGE );
    CHECK( hash_world( world ) == before );
    state.target.position.x = -KF_COORDINATE_LIMIT + I(2);
    CHECK( kf_world_set_body( world, &state ) == KF_OUT_OF_RANGE );
    CHECK( hash_world( world ) == before );
    state.target.position.x = KF_COORDINATE_LIMIT - KF_FIXED_SCALE / 2;
    CHECK( kf_world_set_body( world, &state ) == KF_OK );
    shape.shape = sphere( I(1) );
    CHECK( kf_world_update_shape( world, handle, &shape ) == KF_OUT_OF_RANGE );
    snapshot.objects = objects; snapshot.object_capacity = 1;
    snapshot.shapes = shapes; snapshot.shape_capacity = 1;
    CHECK( kf_world_save( world, &snapshot ) == KF_OK );
    CHECK( kf_world_restore( restored, &snapshot ) == KF_OK );
    before = hash_world( restored );
    objects[0].state.target.position.x = KF_COORDINATE_LIMIT;
    CHECK( kf_world_restore( restored, &snapshot ) == KF_OUT_OF_RANGE );
    CHECK( hash_world( restored ) == before );
    objects[0].state.target.position.x = -KF_COORDINATE_LIMIT + I(2);
    CHECK( kf_world_restore( restored, &snapshot ) == KF_OUT_OF_RANGE );
    CHECK( hash_world( restored ) == before );
    objects[0].state.target = state.target;
    CHECK( kf_world_restore( restored, &snapshot ) == KF_OK );
    CHECK( kf_world_step( restored ) == KF_OK );
    CHECK( kf_world_save( restored, &snapshot ) == KF_OK );
    CHECK( kf_world_restore( world, &snapshot ) == KF_OK );
    CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
    state.pose.position = (kf_vec3_t){0, 0, 0};
    state.has_target = KF_TRUE;
    state.target = state.pose;
    state.target.rotation.y = INT32_C(1073741824);
    CHECK( kf_world_set_body( world, &state ) == KF_OK );
    shape.shape = sphere( KF_FIXED_SCALE / 2 );
    shape.shape.data.sphere.position.x = I(30);
    CHECK( kf_world_update_shape( world, handle, &shape ) == KF_OUT_OF_RANGE );
    CHECK( kf_world_save( world, &snapshot ) == KF_OK );
    CHECK( shapes[0].desc.shape.data.sphere.position.x == 0 );
    CHECK( kf_world_restore( restored, &snapshot ) == KF_OK );
    CHECK( kf_world_remove_body( world, handle ) == KF_OK );
    CHECK( kf_world_save( world, &snapshot ) == KF_OK );
    CHECK( kf_world_restore( restored, &snapshot ) == KF_OK );
    kf_world_destroy( world );
    kf_world_destroy( restored );
}

static void test_contact_lifecycle( void )
{
    kf_world_t * world = create_world();
    kf_body_desc_t floor = kf_body_desc_default( 1 );
    kf_body_desc_t desc = kf_body_desc_default( 2 );
    kf_shape_desc_t floor_shape = {1, {0}};
    kf_shape_desc_t shape = {1, {0}};
    kf_handle_t floor_handle;
    kf_handle_t handle;
    kf_contact_report_t reports[2];
    kf_trigger_event_t events[2];
    size_t count;
    uint32_t generation;
    floor_shape.shape = box( (kf_vec3_t){-I(5), -I(1), -I(5)}, (kf_vec3_t){I(5), 0, I(5)} );
    CHECK( kf_world_add_body( world, &floor, &floor_shape, 1, &floor_handle ) == KF_OK );
    desc.type = KF_BODY_DYNAMIC;
    desc.pose.position.y = KF_FIXED_SCALE / 2;
    shape.shape = sphere( KF_FIXED_SCALE / 2 );
    CHECK( kf_world_add_body( world, &desc, &shape, 1, &handle ) == KF_OK );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_contact_reports( world, reports, 2, &count ) == KF_OK );
    CHECK( count == 1 && reports[0].began == KF_TRUE );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_contact_reports( world, reports, 2, &count ) == KF_OK );
    CHECK( count == 1 && reports[0].began == KF_FALSE );
    generation = handle.generation;
    CHECK( kf_world_remove_body( world, handle ) == KF_OK );
    CHECK( kf_world_add_body( world, &desc, &shape, 1, &handle ) == KF_OK );
    CHECK( handle.generation == generation + 1u );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_contact_reports( world, reports, 2, &count ) == KF_OK );
    CHECK( count == 1 && reports[0].began == KF_TRUE && reports[0].right.generation == handle.generation );
    /* Body and shape edits keep the last step's reports for the game to read. */
    CHECK( kf_world_modify_body( world, handle, &desc ) == KF_OK );
    CHECK( kf_world_contact_reports( world, reports, 2, &count ) == KF_OK && count == 1 && reports[0].began == KF_TRUE );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_contact_reports( world, reports, 2, &count ) == KF_OK );
    CHECK( count == 1 && reports[0].began == KF_FALSE );
    CHECK( kf_world_update_body( world, handle, &desc ) == KF_OK );
    CHECK( kf_world_contact_reports( world, reports, 2, &count ) == KF_OK && count == 1 );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_update_shape( world, handle, &shape ) == KF_OK );
    CHECK( kf_world_contact_reports( world, reports, 2, &count ) == KF_OK && count == 1 );
    CHECK( kf_world_step( world ) == KF_OK );
    desc.trigger = KF_TRUE;
    CHECK( kf_world_modify_body( world, handle, &desc ) == KF_OK );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_trigger_events( world, events, 2, &count ) == KF_OK && count == 1 );
    CHECK( kf_world_update_shape( world, handle, &shape ) == KF_OK );
    CHECK( kf_world_trigger_events( world, events, 2, &count ) == KF_OK && count == 1 );
    /* Adding a body shifts solver indices: contact reports clear, trigger events stay. */
    desc = kf_body_desc_default( 3 );
    desc.pose.position.x = I(4);
    CHECK( kf_world_add_body( world, &desc, &shape, 1, &handle ) == KF_OK );
    CHECK( kf_world_contact_reports( world, reports, 2, &count ) == KF_NO_HIT && count == 0 );
    CHECK( kf_world_trigger_events( world, events, 2, &count ) == KF_OK && count == 1 );
    kf_world_destroy( world );
}

static void check_snapshot_unchanged( kf_world_t * world, const kf_snapshot_t * expected )
{
    kf_snapshot_t actual = {0};
    kf_object_snapshot_t objects[2];
    kf_shape_snapshot_t shapes[2];
    kf_contact_snapshot_t contacts[1];
    actual.objects = objects; actual.object_capacity = 2;
    actual.shapes = shapes; actual.shape_capacity = 2;
    actual.contacts = contacts; actual.contact_capacity = 1;
    CHECK( kf_world_save( world, &actual ) == KF_OK );
    CHECK( actual.tick == expected->tick );
    CHECK( memcmp( &actual.config, &expected->config, sizeof(actual.config) ) == 0 );
    CHECK( actual.object_count == expected->object_count && actual.shape_count == expected->shape_count && actual.contact_count == expected->contact_count );
    CHECK( memcmp( objects, expected->objects, actual.object_count * sizeof(*objects) ) == 0 );
    CHECK( memcmp( shapes, expected->shapes, actual.shape_count * sizeof(*shapes) ) == 0 );
    CHECK( memcmp( contacts, expected->contacts, actual.contact_count * sizeof(*contacts) ) == 0 );
}

static void test_snapshot_contact_validation_case( kf_fixed_t radius, kf_fixed_t mass )
{
    kf_world_t * world = create_world();
    kf_world_t * restored = create_world();
    kf_body_desc_t desc = kf_body_desc_default( 1 );
    kf_shape_desc_t shape = {1, {0}};
    kf_handle_t handle;
    kf_snapshot_t snapshot = {0};
    kf_object_snapshot_t objects[2];
    kf_shape_snapshot_t shapes[2];
    kf_contact_snapshot_t contacts[1];
    kf_contact_snapshot_t saved;
    int64_t invalid_normal[] = {-1, INT64_MIN};
    size_t index;
    shape.shape = box( (kf_vec3_t){-I(5), -I(1), -I(5)}, (kf_vec3_t){I(5), 0, I(5)} );
    CHECK( kf_world_add_body( world, &desc, &shape, 1, &handle ) == KF_OK );
    desc = kf_body_desc_default( 2 );
    desc.type = KF_BODY_DYNAMIC;
    desc.mass = mass;
    desc.pose.position.y = radius;
    desc.velocity.y = -I(1);
    shape.shape = sphere( radius );
    CHECK( kf_world_add_body( world, &desc, &shape, 1, &handle ) == KF_OK );
    CHECK( kf_world_step( world ) == KF_OK );
    snapshot.objects = objects; snapshot.object_capacity = 2;
    snapshot.shapes = shapes; snapshot.shape_capacity = 2;
    snapshot.contacts = contacts; snapshot.contact_capacity = 1;
    CHECK( kf_world_save( world, &snapshot ) == KF_OK );
    CHECK( snapshot.contact_count == 1 && contacts[0].count != 0 );
    CHECK( kf_world_restore( restored, &snapshot ) == KF_OK );
    saved = contacts[0];
    for( index = 0; index != sizeof(invalid_normal) / sizeof(*invalid_normal); ++index )
    {
        contacts[0].points[0].normal_impulse = invalid_normal[index];
        CHECK( kf_world_restore( restored, &snapshot ) == KF_OUT_OF_RANGE );
        contacts[0] = saved;
        check_snapshot_unchanged( restored, &snapshot );
    }
    for( index = 0; index != 3u; ++index )
    {
        contacts[0].points[0].tangent_impulse[index] = INT64_MIN;
        CHECK( kf_world_restore( restored, &snapshot ) == KF_OUT_OF_RANGE );
        contacts[0] = saved;
        check_snapshot_unchanged( restored, &snapshot );
    }
    contacts[0].points[0].anchor.x = INT32_MAX;
    CHECK( kf_world_restore( restored, &snapshot ) == KF_OUT_OF_RANGE );
    contacts[0] = saved;
    check_snapshot_unchanged( restored, &snapshot );
    contacts[0].points[0].anchor.z = INT32_MIN;
    CHECK( kf_world_restore( restored, &snapshot ) == KF_OUT_OF_RANGE );
    contacts[0] = saved;
    check_snapshot_unchanged( restored, &snapshot );
    contacts[0].right_shape = UINT32_MAX;
    CHECK( kf_world_restore( restored, &snapshot ) == KF_STALE_HANDLE );
    contacts[0] = saved;
    check_snapshot_unchanged( restored, &snapshot );
    contacts[0].count = 0;
    CHECK( kf_world_restore( restored, &snapshot ) == KF_INVALID_ARGUMENT );
    contacts[0] = saved;
    check_snapshot_unchanged( restored, &snapshot );
    CHECK( kf_world_step( world ) == KF_OK );
    CHECK( kf_world_step( restored ) == KF_OK );
    CHECK( hash_world( world ) == hash_world( restored ) );
    /* Accepted boundary values must remain safe to consume, even when a
       caller supplies impulses much larger than this scene can generate. */
    contacts[0].points[0].normal_impulse = INT64_MAX;
    contacts[0].points[0].tangent_impulse[0] = INT64_MAX;
    contacts[0].points[0].tangent_impulse[1] = -INT64_MAX;
    contacts[0].points[0].tangent_impulse[2] = INT64_MAX;
    CHECK( kf_world_restore( restored, &snapshot ) == KF_OK );
    CHECK( kf_world_step( restored ) == KF_OK );
    CHECK( kf_world_save( restored, &snapshot ) == KF_OK );
    CHECK( kf_world_restore( world, &snapshot ) == KF_OK );
    kf_world_destroy( world );
    kf_world_destroy( restored );
}

static void test_snapshot_contact_validation( void )
{
    test_snapshot_contact_validation_case( KF_FIXED_SCALE / 2, KF_FIXED_SCALE );
    test_snapshot_contact_validation_case( 64, 128 );
    test_snapshot_contact_validation_case( 16, 128 );
}

static void test_snapshot_permutation( void )
{
    kf_world_t * first = create_world();
    kf_world_t * second = create_world();
    kf_shape_desc_t shape = {1, {0}};
    kf_snapshot_t snapshot = {0};
    kf_object_snapshot_t objects[32];
    kf_shape_snapshot_t shapes[64];
    kf_constraint_snapshot_t constraints[8];
    kf_contact_snapshot_t contacts[64];
    uint64_t expected;
    uint32_t old_id;
    size_t index;
    shape.shape = sphere( KF_FIXED_SCALE / 4 );
    for( index = 0; index != 6; ++index )
    {
        kf_body_desc_t desc = kf_body_desc_default( (uint32_t)index + 1 );
        kf_handle_t handle;
        desc.type = KF_BODY_DYNAMIC;
        desc.pose.position.x = I(-5) + (kf_fixed_t)index * I(2);
        desc.velocity.x = index < 3 ? I(3) + 1 : -I(3) - 1;
        desc.restitution = KF_FIXED_SCALE / 2;
        CHECK( kf_world_add_body( first, &desc, &shape, 1, &handle ) == KF_OK );
    }
    for( index = 6; index != 0; --index )
    {
        kf_body_desc_t desc = kf_body_desc_default( (uint32_t)index );
        kf_handle_t handle;
        desc.type = KF_BODY_DYNAMIC;
        desc.pose.position.x = I(-5) + ((kf_fixed_t)index - 1) * I(2);
        desc.velocity.x = index <= 3 ? I(3) + 1 : -I(3) - 1;
        desc.restitution = KF_FIXED_SCALE / 2;
        CHECK( kf_world_add_body( second, &desc, &shape, 1, &handle ) == KF_OK );
    }
    for( index = 0; index != 10; ++index )
    {
        CHECK( kf_world_step( first ) == KF_OK );
        CHECK( kf_world_step( second ) == KF_OK );
        CHECK( hash_world( first ) == hash_world( second ) );
    }
    CHECK( kf_world_save( first, &snapshot ) == KF_CAPACITY_EXCEEDED );
    CHECK( snapshot.object_count == 6 && snapshot.shape_count == 6 );
    snapshot.objects = objects; snapshot.object_capacity = 32;
    snapshot.shapes = shapes; snapshot.shape_capacity = 64;
    snapshot.constraints = constraints; snapshot.constraint_capacity = 8;
    snapshot.contacts = contacts; snapshot.contact_capacity = 64;
    CHECK( kf_world_save( first, &snapshot ) == KF_OK );
    for( index = 0; index != 8; ++index )
    {
        CHECK( kf_world_step( first ) == KF_OK );
    }
    expected = hash_world( first );
    CHECK( kf_world_restore( second, &snapshot ) == KF_OK );
    for( index = 0; index != 8; ++index )
    {
        CHECK( kf_world_step( second ) == KF_OK );
    }
    CHECK( hash_world( second ) == expected );
    old_id = objects[1].desc.id;
    objects[1].desc.id = objects[0].desc.id;
    CHECK( kf_world_restore( second, &snapshot ) == KF_DUPLICATE_ID );
    CHECK( hash_world( second ) == expected );
    objects[1].desc.id = old_id;
    printf( "deterministic-state=%016llx\n", (unsigned long long)expected );
    kf_world_destroy( first );
    kf_world_destroy( second );
}

static void test_snapshot_links_and_tombstones( void )
{
    kf_world_t * first = create_world();
    kf_world_t * second = create_world();
    kf_shape_desc_t shape = {1, {0}};
    kf_body_desc_t anchor = kf_body_desc_default( 1 );
    kf_handle_t anchor_handle;
    kf_body_desc_t body = kf_body_desc_default( 2 );
    kf_handle_t handle;
    kf_constraint_desc_t link;
    kf_handle_t link_handle;
    kf_handle_t stale_link;
    kf_body_desc_t ephemeral = kf_body_desc_default( 77 );
    kf_handle_t stale_body;
    kf_snapshot_t snapshot = {0};
    kf_object_snapshot_t objects[32];
    kf_shape_snapshot_t shapes[64];
    kf_constraint_snapshot_t links[8];
    kf_contact_snapshot_t contacts[64];
    kf_constraint_state_t link_state;
    kf_handle_t new_body;
    kf_body_state_t state;
    uint64_t expected;
    size_t index;
    shape.shape = sphere( KF_FIXED_SCALE / 4 );
    anchor.pose.position.y = I(4);
    CHECK( kf_world_add_body( first, &anchor, &shape, 1, &anchor_handle ) == KF_OK );
    body.type = KF_BODY_DYNAMIC;
    body.pose.position = (kf_vec3_t){I(2), I(4), 0};
    body.velocity.z = KF_FIXED_SCALE / 10 + 1;
    body.angular_velocity.y = 1;
    CHECK( kf_world_add_body( first, &body, &shape, 1, &handle ) == KF_OK );
    link = (kf_constraint_desc_t){7, KF_CONSTRAINT_DISTANCE, anchor_handle, handle, {0, 0, 0}, {0, 0, 0}, I(2), 0, 0};
    CHECK( kf_world_add_constraint( first, &link, &link_handle ) == KF_OK );
    stale_link = link_handle;
    CHECK( kf_world_remove_constraint( first, link_handle ) == KF_OK );
    CHECK( kf_world_add_constraint( first, &link, &link_handle ) == KF_OK );
    CHECK( link_handle.generation != stale_link.generation );
    CHECK( kf_world_add_body( first, &ephemeral, NULL, 0, &stale_body ) == KF_OK );
    CHECK( kf_world_remove_body( first, stale_body ) == KF_OK );
    for( index = 0; index != 17; ++index )
    {
        CHECK( kf_world_step( first ) == KF_OK );
    }
    snapshot.objects = objects; snapshot.object_capacity = 32;
    snapshot.shapes = shapes; snapshot.shape_capacity = 64;
    snapshot.constraints = links; snapshot.constraint_capacity = 8;
    snapshot.contacts = contacts; snapshot.contact_capacity = 64;
    CHECK( kf_world_save( first, &snapshot ) == KF_OK );
    CHECK( snapshot.object_count == 3 && snapshot.constraint_count == 1 );
    CHECK( kf_world_restore( second, &snapshot ) == KF_OK );
    for( index = 0; index != 37; ++index )
    {
        CHECK( kf_world_step( first ) == KF_OK );
        CHECK( kf_world_step( second ) == KF_OK );
        CHECK( hash_world( first ) == hash_world( second ) );
    }
    expected = hash_world( second );
    ++links[0].state.desc.right.generation;
    CHECK( kf_world_restore( second, &snapshot ) == KF_STALE_HANDLE );
    CHECK( hash_world( second ) == expected );
    --links[0].state.desc.right.generation;
    CHECK( kf_world_get_constraint( second, stale_link, &link_state ) == KF_STALE_HANDLE );
    CHECK( kf_world_get_constraint( second, link_handle, &link_state ) == KF_OK );
    CHECK( kf_world_set_constraint( second, &link_state ) == KF_OK );
    CHECK( kf_world_add_body( second, &ephemeral, NULL, 0, &new_body ) == KF_OK );
    CHECK( new_body.generation == stale_body.generation + 1u );
    CHECK( kf_world_get_body( second, stale_body, &state ) == KF_STALE_HANDLE );
    CHECK( kf_world_remove_body( second, handle ) == KF_OK );
    CHECK( kf_world_get_constraint( second, link_handle, &link_state ) == KF_STALE_HANDLE );
    printf( "restored-links-state=%016llx\n", (unsigned long long)expected );
    kf_world_destroy( first );
    kf_world_destroy( second );
}

typedef struct allocation_tracker_t { size_t allocations; size_t frees; size_t fail_at; } allocation_tracker_t;
static void * allocate( void * context, size_t size )
{
    allocation_tracker_t * tracker = context;
    ++tracker->allocations;
    if( tracker->allocations == tracker->fail_at )
    {
        return NULL;
    }
    return malloc( size );
}
static void deallocate( void * context, void * allocation )
{
    allocation_tracker_t * tracker = context;
    ++tracker->frees;
    free( allocation );
}

static void test_capacity_allocator_numeric( void )
{
    allocation_tracker_t tracker = {0};
    kf_allocator_t allocator = {allocate, deallocate, &tracker};
    kf_world_t * world;
    kf_body_desc_t desc = kf_body_desc_default( 1 );
    kf_shape_desc_t shape = {1, {0}};
    kf_handle_t handle;
    size_t allocations;
    size_t index;
    kf_fixed_t result = 123;
    int64_t remainder = 0;
    kf_fixed_t accumulated = 0;
    CHECK( kf_world_create( NULL, &allocator, &world ) == KF_OK );
    shape.shape = sphere( KF_FIXED_SCALE / 4 );
    CHECK( kf_world_add_body( world, &desc, &shape, 1, &handle ) == KF_CAPACITY_EXCEEDED );
    tracker.fail_at = tracker.allocations + 3;
    CHECK( kf_world_reserve( world, (kf_capacity_t){3, 4, 2, 0, 0} ) == KF_OUT_OF_MEMORY );
    tracker.fail_at = 0;
    CHECK( kf_world_reserve( world, (kf_capacity_t){3, 4, 2, 0, 0} ) == KF_OK );
    CHECK( kf_world_add_body( world, &desc, &shape, 1, &handle ) == KF_OK );
    CHECK( kf_world_reserve( world, (kf_capacity_t){SIZE_MAX, 4, 2, 0, 0} ) == KF_OVERFLOW );
    allocations = tracker.allocations;
    for( index = 0; index != 5; ++index )
    {
        CHECK( kf_world_step( world ) == KF_OK );
    }
    CHECK( tracker.allocations == allocations );
    kf_world_destroy( world );
    CHECK( tracker.allocations == tracker.frees + 1 ); /* one deliberate failed allocation */
    CHECK( kf_checked_add( INT32_MAX, 1, &result ) == KF_OVERFLOW );
    CHECK( result == 123 );
    CHECK( kf_checked_mul( INT32_MAX, I(2), &result ) == KF_OVERFLOW );
    for( index = 0; index != 25; ++index )
    {
        CHECK( kf_integrate_scalar( -1, (kf_timestep_t){1, 25}, &remainder, &result ) == KF_OK );
        accumulated += result;
    }
    CHECK( accumulated == -1 && remainder == 0 );
    CHECK( kf_integrate_scalar( 1, (kf_timestep_t){1, 0}, &remainder, &result ) == KF_INVALID_ARGUMENT );
}

static void test_ineligible_contact_cache( void )
{
    uint32_t mode;
    for( mode = 0; mode != 10; ++mode )
    {
        kf_world_config_t config = kf_world_config_default();
        kf_world_t * world = NULL;
        kf_body_desc_t floor = kf_body_desc_default( 1 );
        kf_body_desc_t body = kf_body_desc_default( 2 );
        kf_shape_desc_t shape = {1, {0}};
        kf_handle_t floor_handle;
        kf_handle_t handle;
        kf_handle_t visitor;
        kf_body_state_t state;
        kf_snapshot_t snapshot = {0};
        uint32_t tick;
        config.sleep_ticks = 3;
        CHECK( kf_world_create( &config, NULL, &world ) == KF_OK );
        CHECK( kf_world_reserve( world, (kf_capacity_t){3, 3, 0, 1, 3} ) == KF_OK );
        shape.shape = box( (kf_vec3_t){-I(5), -I(1), -I(5)}, (kf_vec3_t){I(5), 0, I(5)} );
        CHECK( kf_world_add_body( world, &floor, &shape, 1, &floor_handle ) == KF_OK );
        body.type = KF_BODY_DYNAMIC;
        body.pose.position = (kf_vec3_t){-I(1), KF_FIXED_SCALE / 2, 0};
        shape.shape = sphere( KF_FIXED_SCALE / 2 );
        CHECK( kf_world_add_body( world, &body, &shape, 1, &handle ) == KF_OK );
        for( tick = 0; tick != 100; ++tick )
        {
            CHECK( kf_world_step( world ) == KF_OK );
        }
        CHECK( kf_world_get_body( world, handle, &state ) == KF_OK );
        CHECK( state.sleeping == KF_TRUE );
        CHECK( kf_world_save( world, &snapshot ) == KF_CAPACITY_EXCEEDED );
        CHECK( snapshot.contact_count == 1 );
        body.pose = state.pose;
        switch( mode % 5 )
        {
        case 0: body.type = KF_BODY_STATIC; break;
        case 1: body.type = KF_BODY_KINEMATIC; break;
        case 2: body.trigger = KF_TRUE; break;
        case 3: body.mask = 0; break;
        default: body.category = 0; break;
        }
        if( mode < 5 )
        {
            CHECK( kf_world_modify_body( world, handle, &body ) == KF_OK );
        }
        else
        {
            CHECK( kf_world_update_body( world, handle, &body ) == KF_OK );
        }
        for( tick = 0; tick != 3; ++tick )
        {
            CHECK( kf_world_step( world ) == KF_OK );
            CHECK( kf_world_save( world, &snapshot ) == KF_CAPACITY_EXCEEDED );
            CHECK( snapshot.contact_count == 0 );
        }
        body = kf_body_desc_default( 3 );
        body.type = KF_BODY_DYNAMIC;
        body.pose.position = (kf_vec3_t){I(1), KF_FIXED_SCALE / 2, 0};
        CHECK( kf_world_add_body( world, &body, &shape, 1, &visitor ) == KF_OK );
        for( tick = 0; tick != 3; ++tick )
        {
            CHECK( kf_world_step( world ) == KF_OK );
        }
        CHECK( kf_world_save( world, &snapshot ) == KF_CAPACITY_EXCEEDED );
        CHECK( snapshot.contact_count == 1 );
        kf_world_destroy( world );
    }
}

static void test_removed_constraint_anchor_wakes_island( void )
{
    uint32_t mode;
    for( mode = 0; mode != 4; ++mode )
    {
        kf_world_t * world = NULL;
        kf_body_desc_t desc = kf_body_desc_default( 1 );
        kf_shape_desc_t shape = {1, {0}};
        kf_constraint_desc_t constraint = {0};
        kf_handle_t anchor;
        kf_handle_t weight;
        kf_handle_t tail;
        kf_handle_t link;
        kf_handle_t tail_link;
        kf_body_state_t state;
        kf_constraint_state_t link_state;
        uint32_t tick;
        CHECK( kf_world_create( NULL, NULL, &world ) == KF_OK );
        CHECK( kf_world_reserve( world, (kf_capacity_t){3, 3, 2, 0, 0} ) == KF_OK );
        shape.shape = sphere( KF_FIXED_SCALE / 4 );
        desc.pose.position.y = I(10);
        CHECK( kf_world_add_body( world, &desc, &shape, 1, &anchor ) == KF_OK );
        desc = kf_body_desc_default( 2 );
        desc.type = KF_BODY_DYNAMIC;
        CHECK( kf_world_add_body( world, &desc, &shape, 1, &weight ) == KF_OK );
        desc.id = 3;
        desc.pose.position.y = -I(2);
        CHECK( kf_world_add_body( world, &desc, &shape, 1, &tail ) == KF_OK );
        constraint.id = 1;
        constraint.type = mode < 2 ? KF_CONSTRAINT_DISTANCE : KF_CONSTRAINT_MAX_DISTANCE;
        constraint.left = ( mode & 1u ) == 0 ? anchor : weight;
        constraint.right = ( mode & 1u ) == 0 ? weight : anchor;
        constraint.length = I(10);
        CHECK( kf_world_add_constraint( world, &constraint, &link ) == KF_OK );
        constraint.id = 2;
        constraint.left = weight;
        constraint.right = tail;
        constraint.length = I(2);
        CHECK( kf_world_add_constraint( world, &constraint, &tail_link ) == KF_OK );
        for( tick = 0; tick != 200; ++tick )
        {
            CHECK( kf_world_step( world ) == KF_OK );
        }
        CHECK( kf_world_get_body( world, weight, &state ) == KF_OK );
        CHECK( state.sleeping == KF_TRUE );
        CHECK( kf_world_get_body( world, tail, &state ) == KF_OK );
        CHECK( state.sleeping == KF_TRUE );
        CHECK( kf_world_remove_body( world, anchor ) == KF_OK );
        CHECK( kf_world_get_constraint( world, link, &link_state ) == KF_STALE_HANDLE );
        CHECK( kf_world_get_constraint( world, tail_link, &link_state ) == KF_OK );
        CHECK( kf_world_get_body( world, weight, &state ) == KF_OK );
        CHECK( state.sleeping == KF_FALSE );
        CHECK( kf_world_step( world ) == KF_OK );
        CHECK( kf_world_get_body( world, tail, &state ) == KF_OK );
        CHECK( state.sleeping == KF_FALSE );
        for( tick = 0; tick != 49; ++tick )
        {
            CHECK( kf_world_step( world ) == KF_OK );
        }
        CHECK( kf_world_get_body( world, weight, &state ) == KF_OK );
        CHECK( state.pose.position.y < -I(1) );
        CHECK( state.velocity.y < -I(1) );
        CHECK( kf_world_get_body( world, tail, &state ) == KF_OK );
        CHECK( state.pose.position.y < -I(3) );
        CHECK( state.velocity.y < -I(1) );
        kf_world_destroy( world );
    }
}

int main( void )
{
    test_removed_constraint_anchor_wakes_island();
    test_ineligible_contact_cache();
    test_queries();
    test_subfraction_hit_order();
    test_filters_contacts_relative();
    test_curve();
    test_target_bounds();
    test_contact_lifecycle();
    test_snapshot_contact_validation();
    test_snapshot_permutation();
    test_snapshot_links_and_tombstones();
    test_capacity_allocator_numeric();
    puts( "kinefix world tests passed" );
    return 0;
}
