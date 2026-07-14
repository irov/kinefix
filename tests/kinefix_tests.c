#include "kinefix/kinefix.h"

#include <stdio.h>

static int failures = 0;

#define CHECK(Expression) do { if( !(Expression) ) { fprintf( stderr, "failed line %d: %s\n", __LINE__, #Expression ); ++failures; } } while( 0 )

static int fixed_near( kf_fixed_t left, kf_fixed_t right )
{
    return kf_fixed_abs( kf_fixed_sub( left, right ) ) <= kf_fixed_from_decimal( "0.000001" );
}

int main( void )
{
    kf_fault_t fault;
    kf_fault_t * previous;
    kf_fixed_t sine;
    kf_fixed_t cosine;
    kf_pcg32_t first;
    kf_pcg32_t second;
    kf_aabb_t floor_box = {1u, {-KF_FIXED_SCALE, -KF_FIXED_SCALE, -KF_FIXED_SCALE}, {KF_FIXED_SCALE, 0, KF_FIXED_SCALE}};
    kf_character_body_t body = {{0, 0, 0}, {0, 0, 0}, 1};
    kf_character_config_t config = {
        KF_FIXED_SCALE / 4, KF_FIXED_SCALE, KF_FIXED_SCALE / 2,
        kf_fixed_from_int( 20 ), kf_fixed_from_decimal( "0.01" ), kf_fixed_from_decimal( "0.02" ), kf_fixed_from_int( 32 )};
    kf_character_result_t result;
    uint32_t index;

    kf_fault_clear( &fault );
    previous = kf_fault_bind( &fault );
    CHECK( kf_fixed_mul( kf_fixed_from_decimal( "1.5" ), kf_fixed_from_decimal( "-2.25" ) ) == kf_fixed_from_decimal( "-3.375" ) );
    CHECK( kf_fixed_div( kf_fixed_from_decimal( "-2.25" ), kf_fixed_from_decimal( "1.5" ) ) == kf_fixed_from_decimal( "-1.5" ) );
    CHECK( kf_fixed_sqrt( kf_fixed_from_int( 9 ) ) == kf_fixed_from_int( 3 ) );
    kf_fixed_sin_cos( KF_FIXED_PI / 4, &sine, &cosine );
    CHECK( kf_fixed_abs( kf_fixed_sub( sine, cosine ) ) < 50000 );
    CHECK( kf_fault_failed( &fault ) == 0 );

    kf_pcg32_seed( &first, 42, 54 );
    kf_pcg32_seed( &second, 42, 54 );
    for( index = 0; index != 1000; ++index ) CHECK( kf_pcg32_next( &first ) == kf_pcg32_next( &second ) );

    CHECK( kf_world_character_collides( &floor_box, 1, body.position, config.radius, config.height ) == 0 );
    body.grounded = 0;
    kf_character_step( &body, &config, &floor_box, 1, &result );
    CHECK( body.grounded != 0 );
    CHECK( body.position.y == 0 );

    {
        const kf_aabb_t box = {7u, {0, -KF_FIXED_SCALE, -KF_FIXED_SCALE}, {KF_FIXED_SCALE, KF_FIXED_SCALE, KF_FIXED_SCALE}};
        const kf_ray_t ray = {{-kf_fixed_from_int( 10 ), 0, 0}, {KF_FIXED_SCALE, 0, 0}, kf_fixed_from_int( 20 )};
        kf_hit_t hit;
        CHECK( kf_raycast_aabb( &ray, &box, &hit ) != 0 );
        CHECK( hit.object_id == 7u );
        CHECK( fixed_near( hit.fraction, kf_fixed_from_decimal( "0.5" ) ) );
        CHECK( fixed_near( hit.distance, kf_fixed_from_int( 10 ) ) );
        CHECK( fixed_near( hit.position.x, 0 ) );
        CHECK( hit.normal.x == -KF_FIXED_SCALE && hit.normal.y == 0 && hit.normal.z == 0 );
        CHECK( hit.started_inside == 0 );
    }

    {
        const kf_aabb_t box = {8u, {0, -KF_FIXED_SCALE, -KF_FIXED_SCALE}, {KF_FIXED_SCALE, KF_FIXED_SCALE, KF_FIXED_SCALE}};
        const kf_sphere_t sphere = {{-kf_fixed_from_int( 10 ), 0, 0}, KF_FIXED_SCALE};
        kf_hit_t hit;
        CHECK( kf_sweep_sphere_aabb_hit( &sphere, (kf_vec3_t){kf_fixed_from_int( 20 ), 0, 0}, &box, &hit ) != 0 );
        CHECK( fixed_near( hit.fraction, kf_fixed_from_decimal( "0.45" ) ) );
        CHECK( fixed_near( hit.position.x, -KF_FIXED_SCALE ) );
        CHECK( hit.normal.x == -KF_FIXED_SCALE );
    }

    {
        const kf_aabb_t corner = {10u, {0, 0, 0}, {KF_FIXED_SCALE, kf_fixed_from_int( 3 ), KF_FIXED_SCALE}};
        kf_capsule_t capsule = {{kf_fixed_from_decimal( "1.4" ), 0, kf_fixed_from_decimal( "1.4" )},
            kf_fixed_from_decimal( "0.5" ), kf_fixed_from_int( 2 )};
        CHECK( kf_overlap_capsule_aabb( &capsule, &corner ) == 0 );
        capsule.position.x = kf_fixed_from_decimal( "1.3" );
        capsule.position.z = kf_fixed_from_decimal( "1.3" );
        CHECK( kf_overlap_capsule_aabb( &capsule, &corner ) != 0 );
    }

    {
        const kf_aabb_t walls[2] = {
            {9u, {0, 0, -KF_FIXED_SCALE}, {kf_fixed_from_decimal( "0.1" ), kf_fixed_from_int( 3 ), KF_FIXED_SCALE}},
            {3u, {0, 0, -KF_FIXED_SCALE}, {kf_fixed_from_decimal( "0.1" ), kf_fixed_from_int( 3 ), KF_FIXED_SCALE}}};
        const kf_capsule_t capsule = {{-kf_fixed_from_int( 10 ), 0, 0}, kf_fixed_from_decimal( "0.5" ), kf_fixed_from_int( 2 )};
        kf_hit_t hit;
        CHECK( kf_world_sweep_capsule( walls, 2, &capsule, (kf_vec3_t){kf_fixed_from_int( 20 ), 0, 0}, &hit ) != 0 );
        CHECK( hit.object_id == 3u );
        CHECK( fixed_near( hit.fraction, kf_fixed_from_decimal( "0.475" ) ) );
        CHECK( fixed_near( hit.position.x, kf_fixed_from_decimal( "-0.5" ) ) );
        CHECK( hit.normal.x == -KF_FIXED_SCALE );
    }

    {
        const kf_capsule_t target = {{0, 0, 0}, kf_fixed_from_decimal( "0.5" ), kf_fixed_from_int( 2 )};
        const kf_sphere_t projectile = {{-kf_fixed_from_int( 10 ), KF_FIXED_SCALE, 0}, kf_fixed_from_decimal( "0.1" )};
        kf_hit_t hit;
        CHECK( kf_sweep_sphere_capsule( &projectile, (kf_vec3_t){kf_fixed_from_int( 20 ), 0, 0}, &target, 77u, &hit ) != 0 );
        CHECK( hit.object_id == 77u );
        CHECK( fixed_near( hit.fraction, kf_fixed_from_decimal( "0.47" ) ) );
        CHECK( fixed_near( hit.position.x, kf_fixed_from_decimal( "-0.6" ) ) );
        CHECK( hit.normal.x < kf_fixed_from_decimal( "-0.999" ) );
        CHECK( hit.started_inside == 0 );

        {
            const kf_ray_t ray = {projectile.position, {KF_FIXED_SCALE, 0, 0}, kf_fixed_from_int( 20 )};
            CHECK( kf_raycast_capsule( &ray, &target, 79u, &hit ) != 0 );
            CHECK( hit.object_id == 79u );
            CHECK( fixed_near( hit.fraction, kf_fixed_from_decimal( "0.475" ) ) );
            CHECK( fixed_near( hit.position.x, kf_fixed_from_decimal( "-0.5" ) ) );
        }
    }

    {
        const kf_capsule_t target = {{0, 0, 0}, kf_fixed_from_decimal( "0.5" ), kf_fixed_from_int( 2 )};
        const kf_sphere_t projectile = {{0, kf_fixed_from_int( 5 ), 0}, kf_fixed_from_decimal( "0.1" )};
        kf_hit_t hit;
        CHECK( kf_sweep_sphere_capsule( &projectile, (kf_vec3_t){0, -kf_fixed_from_int( 10 ), 0}, &target, 78u, &hit ) != 0 );
        CHECK( fixed_near( hit.fraction, kf_fixed_from_decimal( "0.29" ) ) );
        CHECK( fixed_near( hit.position.y, kf_fixed_from_decimal( "2.1" ) ) );
        CHECK( hit.normal.y > kf_fixed_from_decimal( "0.999" ) );
    }

    {
        const kf_aabb_t world[2] = {
            {1u, {-kf_fixed_from_int( 100 ), -KF_FIXED_SCALE, -kf_fixed_from_int( 100 )}, {kf_fixed_from_int( 100 ), 0, kf_fixed_from_int( 100 )}},
            {2u, {0, 0, -KF_FIXED_SCALE}, {kf_fixed_from_decimal( "0.1" ), kf_fixed_from_int( 3 ), KF_FIXED_SCALE}}};
        kf_character_body_t fast_body = {{-kf_fixed_from_int( 10 ), 0, 0}, {kf_fixed_from_int( 2000 ), 0, 0}, 1};
        const kf_character_config_t fast_config = {
            kf_fixed_from_decimal( "0.5" ), kf_fixed_from_int( 2 ), kf_fixed_from_decimal( "0.4" ),
            kf_fixed_from_int( 20 ), kf_fixed_from_decimal( "0.01" ), kf_fixed_from_decimal( "0.001" ), kf_fixed_from_int( 100 )};
        kf_character_result_t fast_result;
        kf_character_step( &fast_body, &fast_config, world, 2, &fast_result );
        CHECK( fast_result.blocked != 0 );
        CHECK( fast_result.hit.object_id == 2u );
        CHECK( fast_body.position.x < kf_fixed_from_decimal( "-0.5" ) );
        CHECK( fast_body.velocity.x == 0 );
        CHECK( fast_body.grounded != 0 );
    }

    {
        const kf_aabb_t world[2] = {
            {1u, {-kf_fixed_from_int( 10 ), -KF_FIXED_SCALE, -kf_fixed_from_int( 10 )}, {kf_fixed_from_int( 10 ), 0, kf_fixed_from_int( 10 )}},
            {2u, {0, 0, -KF_FIXED_SCALE}, {KF_FIXED_SCALE, kf_fixed_from_decimal( "0.25" ), KF_FIXED_SCALE}}};
        kf_character_body_t step_body = {{-KF_FIXED_SCALE, 0, 0}, {kf_fixed_from_int( 100 ), 0, 0}, 1};
        const kf_character_config_t step_config = {
            kf_fixed_from_decimal( "0.25" ), KF_FIXED_SCALE, kf_fixed_from_decimal( "0.5" ),
            kf_fixed_from_int( 20 ), kf_fixed_from_decimal( "0.01" ), kf_fixed_from_decimal( "0.001" ), kf_fixed_from_int( 10 )};
        kf_character_result_t step_result;
        kf_character_step( &step_body, &step_config, world, 2, &step_result );
        CHECK( step_result.stepped != 0 );
        CHECK( fixed_near( step_body.position.x, 0 ) );
        CHECK( fixed_near( step_body.position.y, kf_fixed_from_decimal( "0.25" ) ) );
        CHECK( step_body.grounded != 0 );
    }

    kf_fault_restore( previous );
    if( failures != 0 ) return 1;
    puts( "kinefix C tests passed" );
    return 0;
}
