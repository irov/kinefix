#include "kinefix/kinefix.h"

#include <stdio.h>

static int failures = 0;

#define CHECK(Expression) do { if( !(Expression) ) { fprintf( stderr, "failed line %d: %s\n", __LINE__, #Expression ); ++failures; } } while( 0 )

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

    CHECK( kf_world_character_collides( &floor_box, 1, body.position, config.half_width, config.height ) == 0 );
    body.grounded = 0;
    kf_character_step( &body, &config, &floor_box, 1, &result );
    CHECK( body.grounded != 0 );
    CHECK( body.position.y == 0 );

    kf_fault_restore( previous );
    if( failures != 0 ) return 1;
    puts( "kinefix C tests passed" );
    return 0;
}
