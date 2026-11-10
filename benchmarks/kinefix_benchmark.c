#include "kinefix/kinefix.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define CHARACTER_COUNT 256u
#define PROJECTILE_COUNT 512u
#define WALL_COUNT 64u
#define BODY_COUNT (CHARACTER_COUNT + PROJECTILE_COUNT + WALL_COUNT + 1u)
#define I(x) ((x) * KF_FIXED_SCALE)

static size_t allocations;
static size_t frees;

static void * allocate( void * context, size_t bytes )
{
    (void)context;
    ++allocations;
    return malloc( bytes );
}

static void deallocate( void * context, void * allocation )
{
    (void)context;
    ++frees;
    free( allocation );
}

static void require( kf_status_t status )
{
    if( status == KF_OK )
    {
        return;
    }
    fprintf( stderr, "scenario failed: status=%d\n", (int)status );
    exit( 1 );
}

static uint64_t word( uint64_t hash, uint64_t value )
{
    return (hash ^ value) * UINT64_C(1099511628211);
}

int main( int argc, char ** argv )
{
    uint32_t ticks = 25;
    kf_bool_t reverse = KF_FALSE;
    kf_allocator_t allocator = {allocate, deallocate, NULL};
    kf_world_config_t config = kf_world_config_default();
    kf_world_t * world;
    kf_handle_t handles[BODY_COUNT];
    kf_character_state_t characters[CHARACTER_COUNT] = {0};
    kf_character_settings_t settings = kf_character_settings_default();
    kf_shape_t query_shape = {0};
    kf_query_filter_t shot_filter = kf_query_filter_default();
    size_t reserved_allocations;
    clock_t query_time = 0;
    clock_t step_time = 0;
    clock_t character_time = 0;
    uint32_t insertion;
    uint32_t tick;
    uint64_t query_hash = UINT64_C(1469598103934665603);
    uint64_t hash;
    if( argc > 1 )
    {
        ticks = (uint32_t)strtoul( argv[1], NULL, 10 );
    }
    if( argc > 2 )
    {
        reverse = (kf_bool_t)(strtoul( argv[2], NULL, 10 ) != 0);
    }
    if( ticks == 0 || ticks > 10000 )
    {
        return 2;
    }
    config.gravity = (kf_vec3_t){0, 0, 0};
    config.solver_iterations = 4;
    config.ccd_iterations = 64;
    require( kf_world_create( &config, &allocator, &world ) );
    require( kf_world_reserve( world, (kf_capacity_t){BODY_COUNT, BODY_COUNT, 0, 0, 0} ) );
    for( insertion = 0; insertion != BODY_COUNT; ++insertion )
    {
        uint32_t index = reverse == KF_TRUE ? BODY_COUNT - insertion - 1u : insertion;
        kf_body_desc_t body = kf_body_desc_default( index + 1u );
        kf_shape_desc_t shape = {1, {0}};
        if( index < CHARACTER_COUNT )
        {
            body.type = KF_BODY_KINEMATIC;
            body.pose.position = (kf_vec3_t){((int32_t)(index % 16u) - 8) * I(3), 0, ((int32_t)(index / 16u) - 8) * I(3)};
            body.category = 1;
            body.mask = 1;
            shape.shape.type = KF_SHAPE_CAPSULE;
            shape.shape.data.capsule.radius = KF_FIXED_SCALE / 4;
            shape.shape.data.capsule.height = I(2);
            characters[index].position = body.pose.position;
            characters[index].velocity = (kf_vec3_t){I(1), 0, 0};
        }
        else if( index < CHARACTER_COUNT + PROJECTILE_COUNT )
        {
            uint32_t projectile = index - CHARACTER_COUNT;
            body.type = KF_BODY_DYNAMIC;
            body.pose.position = (kf_vec3_t){-I(50) + (int32_t)(projectile % 32u) * I(2), I(4), ((int32_t)(projectile / 32u) - 8) * I(3)};
            body.velocity = (kf_vec3_t){I(20), 0, 0};
            body.restitution = KF_FIXED_SCALE / 2;
            body.category = 2;
            body.mask = 4;
            shape.shape.type = KF_SHAPE_SPHERE;
            shape.shape.data.sphere.radius = KF_FIXED_SCALE / 10;
        }
        else if( index == BODY_COUNT - 1u )
        {
            body.category = 1;
            body.mask = 1;
            shape.shape.type = KF_SHAPE_AABB;
            shape.shape.data.aabb = (kf_aabb_t){{-I(64), -I(1), -I(64)}, {I(64), 0, I(64)}};
        }
        else
        {
            uint32_t wall = index - CHARACTER_COUNT - PROJECTILE_COUNT;
            body.pose.position = (kf_vec3_t){wall < 32 ? I(30) : -I(55), 0, ((int32_t)(wall % 32u) - 16) * I(3)};
            body.category = 4;
            body.mask = UINT32_MAX;
            shape.shape.type = KF_SHAPE_AABB;
            shape.shape.data.aabb = (kf_aabb_t){{0, -I(2), -I(1)}, {KF_FIXED_SCALE / 10, I(8), I(1)}};
        }
        require( kf_world_add_body( world, &body, &shape, 1, handles + index ) );
    }
    settings.gravity = 0;
    settings.step_height = 0;
    settings.snap_distance = 0;
    query_shape.type = KF_SHAPE_SPHERE;
    query_shape.data.sphere.radius = KF_FIXED_SCALE / 10;
    shot_filter.category = 2;
    shot_filter.mask = 4;
    reserved_allocations = allocations;
    for( tick = 0; tick != ticks; ++tick )
    {
        uint32_t index;
        clock_t begin = clock();
        for( index = 0; index != PROJECTILE_COUNT; ++index )
        {
            kf_body_state_t body;
            kf_sweep_hit_t hit;
            kf_status_t status;
            require( kf_world_get_body( world, handles[CHARACTER_COUNT + index], &body ) );
            status = kf_world_sweep_first( world, &query_shape, body.pose, (kf_vec3_t){I(60), 0, 0}, &shot_filter, &hit );
            if( status != KF_OK && status != KF_NO_HIT )
            {
                require( status );
            }
            query_hash = word( query_hash, status == KF_OK ? hit.object.id : 0 );
        }
        query_time += clock() - begin;
        begin = clock();
        for( index = 0; index != CHARACTER_COUNT; ++index )
        {
            kf_query_filter_t filter = kf_query_filter_default();
            kf_character_move_result_t result;
            kf_contact_t contacts[8];
            filter.category = 1;
            filter.mask = 1;
            filter.exclude = handles + index;
            filter.exclude_count = 1;
            require( kf_character_move( world, characters + index, &settings, &filter, contacts, 8, &result ) );
            require( kf_world_set_kinematic_target( world, handles[index], (kf_pose_t){characters[index].position, {0, 0, 0}} ) );
        }
        character_time += clock() - begin;
        begin = clock();
        require( kf_world_step( world ) );
        step_time += clock() - begin;
    }
    hash = word( query_hash, kf_world_tick( world ) );
    for( insertion = 0; insertion != BODY_COUNT; ++insertion )
    {
        kf_body_state_t state;
        require( kf_world_get_body( world, handles[insertion], &state ) );
        hash = word( hash, state.handle.id );
        hash = word( hash, (uint32_t)state.pose.position.x );
        hash = word( hash, (uint32_t)state.pose.position.y );
        hash = word( hash, (uint32_t)state.pose.position.z );
        hash = word( hash, (uint32_t)state.pose.rotation.x );
        hash = word( hash, (uint32_t)state.pose.rotation.y );
        hash = word( hash, (uint32_t)state.pose.rotation.z );
        hash = word( hash, (uint32_t)state.velocity.x );
        hash = word( hash, (uint32_t)state.angular_velocity.y );
        hash = word( hash, (uint64_t)state.position_remainder[0] );
    }
    if( allocations != reserved_allocations )
    {
        return 3;
    }
    printf( "characters=%u projectiles=%u walls=%u ticks=%u reverse=%u\n", CHARACTER_COUNT, PROJECTILE_COUNT, WALL_COUNT, ticks, reverse );
    printf( "queries_ms_per_tick=%.3f character_ms_per_tick=%.3f step_ms_per_tick=%.3f\n", 1000.0 * query_time / CLOCKS_PER_SEC / ticks,
        1000.0 * character_time / CLOCKS_PER_SEC / ticks, 1000.0 * step_time / CLOCKS_PER_SEC / ticks );
    printf( "checksum=%016llx step_allocations=%zu\n", (unsigned long long)hash, allocations - reserved_allocations );
    kf_world_destroy( world );
    if( allocations != frees )
    {
        return 4;
    }
    return 0;
}
