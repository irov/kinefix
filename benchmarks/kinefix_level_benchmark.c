#include "kinefix/kinefix.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/* A level-like load: a floor with 2000 static wall cells, 30 piles of dynamic
   props, actors steered by velocity every tick, and line-of-sight rays. */
#define WALL_COUNT 2000u
#define PROP_COUNT 300u
#define ACTOR_COUNT 80u
#define RAY_COUNT 400u
#define BODY_COUNT (WALL_COUNT + PROP_COUNT + ACTOR_COUNT + 1u)
#define I(x) ((x) * KF_FIXED_SCALE)
#define R(numerator, denominator) kf_fixed_from_ratio( (numerator), (denominator) )

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
    if( status == KF_OK || status == KF_NO_HIT )
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

static kf_handle_t add_body( kf_world_t * world, kf_body_desc_t * body, kf_shape_desc_t * shape )
{
    kf_handle_t handle;
    require( kf_world_add_body( world, body, shape, 1, &handle ) );
    return handle;
}

/* Arguments: ticks, then optionally the step rate in Hz and the substep count. */
int main( int argc, char ** argv )
{
    uint32_t ticks = 300;
    uint32_t index;
    uint32_t tick;
    clock_t step_time = 0;
    clock_t ray_time = 0;
    size_t loop_allocations;
    uint64_t hash = UINT64_C(1469598103934665603);
    kf_allocator_t allocator = {allocate, deallocate, NULL};
    kf_world_config_t config = kf_world_config_default();
    kf_world_t * world;
    kf_handle_t actors[ACTOR_COUNT];
    kf_pcg32_t random;
    if( argc > 1 )
    {
        ticks = (uint32_t)strtoul( argv[1], NULL, 10 );
    }
    if( argc > 3 )
    {
        config.step = (kf_timestep_t){1u, (uint32_t)strtoul( argv[2], NULL, 10 )};
        config.substeps = (uint32_t)strtoul( argv[3], NULL, 10 );
    }
    if( ticks == 0 || ticks > 100000 )
    {
        return 2;
    }
    require( kf_world_create( &config, &allocator, &world ) );
    require( kf_world_reserve( world, (kf_capacity_t){BODY_COUNT, BODY_COUNT, 0, 0, 0} ) );
    {
        kf_body_desc_t body = kf_body_desc_default( 1 );
        kf_shape_desc_t shape = {1, {0}};
        shape.shape.type = KF_SHAPE_AABB;
        shape.shape.data.aabb = (kf_aabb_t){{-I(150), -I(1), -I(90)}, {I(150), 0, I(90)}};
        (void)add_body( world, &body, &shape );
    }
    for( index = 0; index != WALL_COUNT; ++index )
    {
        kf_body_desc_t body = kf_body_desc_default( 10u + index );
        kf_shape_desc_t shape = {1, {0}};
        body.pose.position = (kf_vec3_t){I((int32_t)(index % 50u) * 6 - 147), 0, I((int32_t)(index / 50u) * 4 - 80)};
        shape.shape.type = KF_SHAPE_BOX;
        shape.shape.data.box.half_extents = (kf_vec3_t){R(1, 2), I(1), R(1, 2)};
        shape.shape.data.box.center.y = I(1);
        (void)add_body( world, &body, &shape );
    }
    for( index = 0; index != PROP_COUNT; ++index )
    {
        kf_body_desc_t body = kf_body_desc_default( 10000u + index );
        kf_shape_desc_t shape = {1, {0}};
        int32_t pile = (int32_t)(index / 10u);
        body.type = KF_BODY_DYNAMIC;
        body.pose.position = (kf_vec3_t){I(pile % 10 * 12 - 60) + R(3, 2), R(1, 2) + I((int32_t)(index % 10u) / 3), I(pile / 10 * 12 - 20) + I((int32_t)(index % 3u))};
        shape.shape.type = KF_SHAPE_BOX;
        shape.shape.data.box.half_extents = (kf_vec3_t){R(2, 5), R(2, 5), R(2, 5)};
        (void)add_body( world, &body, &shape );
    }
    for( index = 0; index != ACTOR_COUNT; ++index )
    {
        kf_body_desc_t body = kf_body_desc_default( 20000u + index );
        kf_shape_desc_t shape = {1, {0}};
        body.type = KF_BODY_DYNAMIC;
        body.pose.position = (kf_vec3_t){I((int32_t)(index % 10u) * 8 - 40) + I(3), 0, I((int32_t)(index / 10u) * 8 - 30) + I(1)};
        body.mass = I(4);
        shape.shape.type = KF_SHAPE_CAPSULE;
        shape.shape.data.capsule.radius = R(2, 5);
        shape.shape.data.capsule.height = I(2);
        actors[index] = add_body( world, &body, &shape );
    }
    kf_pcg32_seed( &random, 7, 3 );
    loop_allocations = allocations;
    for( tick = 0; tick != ticks; ++tick )
    {
        clock_t begin;
        for( index = 0; index != ACTOR_COUNT; ++index )
        {
            kf_body_state_t state;
            int32_t angle = (int32_t)((tick + index * 7u) % 360u);
            require( kf_world_get_body( world, actors[index], &state ) );
            require( kf_world_set_velocity( world, actors[index], (kf_vec3_t){angle < 180 ? I(4) : -I(4), state.velocity.y, (angle / 90) % 2 == 0 ? I(3) : -I(3)}, (kf_vec3_t){0, 0, 0} ) );
        }
        begin = clock();
        require( kf_world_step( world ) );
        step_time += clock() - begin;
        begin = clock();
        for( index = 0; index != RAY_COUNT; ++index )
        {
            kf_ray_t ray;
            kf_sweep_hit_t hit;
            kf_status_t status;
            ray.origin = (kf_vec3_t){I((int32_t)kf_pcg32_bounded( &random, 280 ) - 140), I(1), I((int32_t)kf_pcg32_bounded( &random, 160 ) - 80)};
            ray.direction = (kf_vec3_t){I((int32_t)kf_pcg32_bounded( &random, 21 ) - 10), 0, I((int32_t)kf_pcg32_bounded( &random, 21 ) - 10)};
            if( ray.direction.x == 0 && ray.direction.z == 0 )
            {
                ray.direction.x = I(1);
            }
            ray.maximum_distance = I(30);
            status = kf_world_raycast_first( world, &ray, NULL, &hit );
            require( status );
            hash = word( hash, status == KF_OK ? hit.object.id : 0u );
        }
        ray_time += clock() - begin;
    }
    loop_allocations = allocations - loop_allocations;
    for( index = 0; index != ACTOR_COUNT; ++index )
    {
        kf_body_state_t state;
        require( kf_world_get_body( world, actors[index], &state ) );
        hash = word( hash, (uint32_t)state.pose.position.x );
        hash = word( hash, (uint32_t)state.pose.position.z );
    }
    kf_world_destroy( world );
    printf( "bodies=%u rays=%u ticks=%u\n", BODY_COUNT, RAY_COUNT, ticks );
    printf( "step_ms_per_tick=%.3f rays_ms_per_tick=%.3f\n", 1000.0 * (double)step_time / CLOCKS_PER_SEC / ticks, 1000.0 * (double)ray_time / CLOCKS_PER_SEC / ticks );
    printf( "checksum=%016llx step_allocations=%zu\n", (unsigned long long)hash, loop_allocations );
    return allocations == frees && loop_allocations == 0 ? 0 : 1;
}
