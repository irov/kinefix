#include "kinefix/kinefix.h"

#include <stdio.h>
#include <stdlib.h>

#define I(x) ((x) * KF_FIXED_SCALE)
#define CHECK(c) do { if( !(c) ) { fprintf( stderr, "%s:%d: %s\n", __FILE__, __LINE__, #c ); exit(1); } } while(0)
#define OBJECTS 120

static kf_fixed_t random_fixed( kf_pcg32_t * rng, int32_t radius )
{
    return (int32_t)kf_pcg32_bounded( rng, (uint32_t)(2 * radius + 1) ) - radius;
}

static int hit_compare( const void * a, const void * b )
{
    const kf_sweep_hit_t * left = a;
    const kf_sweep_hit_t * right = b;
    if( left->fraction_q32 != right->fraction_q32 )
    {
        return left->fraction_q32 < right->fraction_q32 ? -1 : 1;
    }
    if( left->object.id != right->object.id )
    {
        return left->object.id < right->object.id ? -1 : 1;
    }
    return 0;
}

static void check_hit( const kf_sweep_hit_t * actual, const kf_sweep_hit_t * expected )
{
    CHECK( actual->object.id == expected->object.id );
    CHECK( actual->object.generation == expected->object.generation );
    CHECK( actual->shape_id == expected->shape_id );
    CHECK( actual->fraction == expected->fraction );
    CHECK( actual->fraction_q32 == expected->fraction_q32 );
    CHECK( actual->distance == expected->distance );
    CHECK( kf_vec3_equal( actual->position, expected->position ) );
    CHECK( kf_vec3_equal( actual->contact_point, expected->contact_point ) );
    CHECK( kf_vec3_equal( actual->normal, expected->normal ) );
    CHECK( actual->started_inside == expected->started_inside );
    CHECK( actual->trigger == expected->trigger );
}

/* A direct shape scan independently checks that index pruning loses no hits. */
int main( void )
{
    kf_world_t * world = NULL;
    kf_body_desc_t bodies[OBJECTS];
    kf_shape_desc_t shapes[OBJECTS];
    kf_handle_t handles[OBJECTS];
    kf_pcg32_t rng;
    uint32_t i;
    CHECK( kf_world_create( NULL, NULL, &world ) == KF_OK );
    CHECK( kf_world_reserve( world, (kf_capacity_t){OBJECTS, OBJECTS, 0, 0, 0} ) == KF_OK );
    kf_pcg32_seed( &rng, 12345, 6789 );
    for( i = 0; i != OBJECTS; ++i )
    {
        kf_body_desc_t * body = &bodies[i];
        kf_shape_t * shape = &shapes[i].shape;
        *body = kf_body_desc_default( i + 1 );
        shapes[i] = (kf_shape_desc_t){0};
        shapes[i].id = 1;
        body->pose.position = (kf_vec3_t){random_fixed( &rng, I(20) ), random_fixed( &rng, I(20) ), random_fixed( &rng, I(20) )};
        body->pose.rotation = kf_rotation_from_axis_angle( (kf_vec3_t){I(1), I(2), I(3)}, random_fixed( &rng, I(3) ) );
        body->trigger = (kf_bool_t)(i % 11 == 0);
        shape->type = (kf_shape_type_t)(i % 6);
        switch( shape->type )
        {
        case KF_SHAPE_AABB:
            shape->data.aabb = (kf_aabb_t){{-I(1), -I(1), -I(1)}, {I(1), I(1), I(1)}};
            break;
        case KF_SHAPE_SPHERE:
            shape->data.sphere.radius = I(1);
            break;
        case KF_SHAPE_CAPSULE:
            shape->data.capsule.radius = I(1) / 2;
            shape->data.capsule.height = I(3);
            break;
        case KF_SHAPE_BOX:
            shape->data.box.half_extents = (kf_vec3_t){I(1), I(2), I(1) / 2};
            shape->data.box.yaw = 8192;
            break;
        case KF_SHAPE_PRISM:
            shape->data.prism.count = 4;
            shape->data.prism.minimum_y = -I(1);
            shape->data.prism.maximum_y = I(1);
            shape->data.prism.vertices[0] = (kf_vec3_t){-I(1), 0, -I(1)};
            shape->data.prism.vertices[1] = (kf_vec3_t){I(1), 0, -I(1)};
            shape->data.prism.vertices[2] = (kf_vec3_t){I(1), 0, I(1)};
            shape->data.prism.vertices[3] = (kf_vec3_t){-I(1), 0, I(1)};
            break;
        case KF_SHAPE_TRIANGLE:
            shape->data.triangle.vertices[0] = (kf_vec3_t){-I(1), 0, -I(1)};
            shape->data.triangle.vertices[1] = (kf_vec3_t){I(1), 0, -I(1)};
            shape->data.triangle.vertices[2] = (kf_vec3_t){0, 0, I(1)};
            break;
        }
        CHECK( kf_world_add_body( world, body, &shapes[i], 1, &handles[i] ) == KF_OK );
    }
    for( i = 0; i != 2048; ++i )
    {
        kf_ray_t ray = {{random_fixed( &rng, I(25) ), random_fixed( &rng, I(25) ), random_fixed( &rng, I(25) )},
            {random_fixed( &rng, I(2) ), random_fixed( &rng, I(2) ), random_fixed( &rng, I(2) )}, I(50)};
        kf_query_filter_t filter = kf_query_filter_default();
        kf_sweep_hit_t expected[OBJECTS];
        kf_sweep_hit_t actual[OBJECTS];
        size_t count = 0;
        size_t required = 0;
        size_t j;
        if( i % 10 == 0 )
        {
            ray.direction = (kf_vec3_t){1, 1, 0};
        }
        if( i % 10 == 1 )
        {
            ray.direction = (kf_vec3_t){I(1), 0, 0};
        }
        if( ray.direction.x == 0 && ray.direction.y == 0 && ray.direction.z == 0 )
        {
            ray.direction.x = 1;
        }
        filter.include_triggers = (kf_bool_t)(i % 2);
        filter.exclude = &handles[i % OBJECTS];
        filter.exclude_count = 1;
        for( j = 0; j != OBJECTS; ++j )
        {
            kf_status_t status;
            if( j == i % OBJECTS )
            {
                continue;
            }
            if( bodies[j].trigger != KF_FALSE && filter.include_triggers == KF_FALSE )
            {
                continue;
            }
            status = kf_shape_raycast( &ray, &shapes[j].shape, bodies[j].pose, &expected[count] );
            CHECK( status == KF_OK || status == KF_NO_HIT );
            if( status == KF_NO_HIT )
            {
                continue;
            }
            expected[count].object = handles[j];
            expected[count].shape_id = shapes[j].id;
            expected[count].trigger = bodies[j].trigger;
            ++count;
        }
        qsort( expected, count, sizeof(*expected), hit_compare );
        CHECK( kf_world_raycast_all( world, &ray, &filter, actual, OBJECTS, &required ) == (count != 0 ? KF_OK : KF_NO_HIT) );
        CHECK( required == count );
        for( j = 0; j != count; ++j )
        {
            check_hit( &actual[j], &expected[j] );
        }
        CHECK( kf_world_raycast_first( world, &ray, &filter, actual ) == (count != 0 ? KF_OK : KF_NO_HIT) );
        if( count != 0 )
        {
            check_hit( actual, expected );
        }
        CHECK( kf_world_raycast_all( world, &ray, &filter, actual, 1, &required ) == (count > 1 ? KF_CAPACITY_EXCEEDED : count != 0 ? KF_OK : KF_NO_HIT) );
        CHECK( required == count );
        if( count != 0 )
        {
            check_hit( actual, expected );
        }
    }
    kf_world_destroy( world );
    puts( "query index matches direct scans: 2048 rays, six shape types" );
    return 0;
}
