#include "kinefix/kinefix.h"

#include <stdio.h>
#include <string.h>

#define F(_value) kf_fixed_from_double( _value )
#define CHECK(_condition) do { if( !(_condition) ) { fprintf( stderr, "%s:%d: %s\n", __FILE__, __LINE__, #_condition ); return 1; } } while( 0 )

static kf_world_t * make_world( void )
{
    kf_world_t * world = NULL;
    kf_world_config_t config = kf_world_config_default();
    kf_capacity_t capacity = {32, 64, 0, 0, 0};

    if( kf_world_create( &config, NULL, &world ) != KF_OK )
    {
        return NULL;
    }

    if( kf_world_reserve( world, capacity ) != KF_OK )
    {
        kf_world_destroy( world );
        return NULL;
    }

    return world;
}

static kf_status_t add_box( kf_world_t * _world, uint32_t _id, kf_vec3_t _minimum, kf_vec3_t _maximum, kf_body_type_t _type, kf_handle_t * _handle )
{
    kf_body_desc_t body = kf_body_desc_default( _id );
    kf_shape_desc_t shape;
    memset( &shape, 0, sizeof( shape ) );
    body.type = _type;
    shape.id = 1;
    shape.shape.type = KF_SHAPE_AABB;
    shape.shape.data.aabb = (kf_aabb_t){_minimum, _maximum};

    return kf_world_add_body( _world, &body, &shape, 1, _handle );
}

static kf_character_settings_t character_settings( void )
{
    kf_character_settings_t settings = kf_character_settings_default();
    settings.radius = F( .25 );
    settings.height = F( 2 );
    settings.gravity = 0;
    settings.step_height = 0;

    return settings;
}

static int test_diagonal_and_planes( void )
{
    kf_world_t * world = make_world();
    kf_handle_t object;
    kf_character_state_t character;
    kf_character_settings_t settings = character_settings();
    kf_character_move_result_t result;
    kf_contact_t contacts[64];
    CHECK( world != NULL );
    CHECK( add_box( world, 1, (kf_vec3_t){F( -.1 ), 0, F( -.1 )}, (kf_vec3_t){F( .1 ), F( 3 ), F( .1 )}, KF_BODY_STATIC, &object ) == KF_OK );
    memset( &character, 0, sizeof( character ) );
    character.position = (kf_vec3_t){F( -2 ), 0, F( -2 )};
    character.velocity = (kf_vec3_t){F( 100 ), 0, F( 100 )};
    CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
    CHECK( character.position.x < 0 );
    CHECK( character.position.z < 0 );
    CHECK( result.contact_count != 0 );
    CHECK( result.displacement.x > 0 && result.displacement.z > 0 );
    kf_world_destroy( world );

    world = make_world();
    CHECK( world != NULL );
    CHECK( add_box( world, 2, (kf_vec3_t){0, F( -1 ), F( -10 )}, (kf_vec3_t){F( .1 ), F( 3 ), F( 10 )}, KF_BODY_STATIC, &object ) == KF_OK );
    memset( &character, 0, sizeof( character ) );
    character.position = (kf_vec3_t){F( -2 ), 0, F( -2 )};
    character.velocity = (kf_vec3_t){F( 100 ), 0, F( 75 )};
    CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
    CHECK( character.position.x < F( -.24 ) );
    CHECK( character.position.z > F( .9 ) );
    CHECK( character.velocity.z == F( 75 ) );
    CHECK( add_box( world, 3, (kf_vec3_t){F( -10 ), F( -1 ), 0}, (kf_vec3_t){F( 10 ), F( 3 ), F( .1 )}, KF_BODY_STATIC, &object ) == KF_OK );
    character.position = (kf_vec3_t){F( -1 ), 0, F( -1 )};
    character.velocity = (kf_vec3_t){F( 50 ), 0, F( 50 )};
    CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
    CHECK( character.position.x < F( -.24 ) && character.position.z < F( -.24 ) );
    kf_world_destroy( world );

    return 0;
}

static int test_depenetration_and_character( void )
{
    kf_world_t * world = make_world();
    kf_handle_t object;
    kf_character_state_t character;
    kf_character_settings_t settings = character_settings();
    kf_character_move_result_t result;
    kf_contact_t contacts[64];
    kf_body_desc_t body = kf_body_desc_default( 2 );
    kf_shape_desc_t shape;
    CHECK( world != NULL );
    CHECK( add_box( world, 1, (kf_vec3_t){0, F( -1 ), F( -1 )}, (kf_vec3_t){F( 1 ), F( 3 ), F( 1 )}, KF_BODY_STATIC, &object ) == KF_OK );
    memset( &character, 0, sizeof( character ) );
    character.position.x = F( -.1 );
    character.velocity.x = F( -25 );
    CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
    CHECK( character.position.x < F( -1.2 ) );
    CHECK( result.contact_count != 0 );
    CHECK( kf_world_remove_body( world, object ) == KF_OK );
    memset( &shape, 0, sizeof( shape ) );
    shape.id = 1;
    shape.shape.type = KF_SHAPE_CAPSULE;
    shape.shape.data.capsule.radius = settings.radius;
    shape.shape.data.capsule.height = settings.height;
    CHECK( kf_world_add_body( world, &body, &shape, 1, &object ) == KF_OK );
    memset( &character, 0, sizeof( character ) );
    character.position = (kf_vec3_t){F( -2 ), 0, F( -1 )};
    character.velocity = (kf_vec3_t){F( 100 ), 0, F( 50 )};
    CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
    CHECK( result.displacement.x > F( 1 ) );
    CHECK( result.contact_count != 0 );
    {
        kf_shape_t moving = shape.shape;
        kf_contact_t contact;
        kf_status_t status = kf_shape_contact( &moving, (kf_pose_t){character.position, {0, 0, 0}}, &shape.shape, body.pose, &contact );
        CHECK( status == KF_NO_HIT || (status == KF_OK && contact.depth <= KF_GEOMETRY_TOLERANCE) );
    }
    kf_world_destroy( world );

    return 0;
}

static int test_floor_edge_hole_and_capacity( void )
{
    kf_world_t * world = make_world();
    kf_handle_t object;
    kf_character_state_t character;
    kf_character_state_t before;
    kf_character_settings_t settings = character_settings();
    kf_character_move_result_t result;
    kf_contact_t contacts[64];
    CHECK( world != NULL );
    settings.gravity = F( 20 );
    CHECK( add_box( world, 1, (kf_vec3_t){F( -5 ), F( -1 ), F( -5 )}, (kf_vec3_t){F( 5 ), 0, F( 5 )}, KF_BODY_STATIC, &object ) == KF_OK );
    memset( &character, 0, sizeof( character ) );
    before = character;
    CHECK( kf_character_move( world, &character, &settings, NULL, NULL, 0, &result ) == KF_CAPACITY_EXCEEDED );
    CHECK( result.contact_count > 0 );
    CHECK( memcmp( &character, &before, sizeof( character ) ) == 0 );
    CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
    CHECK( character.grounded != KF_FALSE && character.support.id == 1 );
    CHECK( character.support_normal.y > F( .99 ) );
    CHECK( result.landed != KF_FALSE );
    {
        kf_body_state_t floor;
        CHECK( kf_world_get_body( world, object, &floor ) == KF_OK );
        floor.velocity = (kf_vec3_t){F( 25 ), F( 50 ), F( 75 )};
        floor.angular_velocity.y = F( 2 );
        CHECK( kf_world_set_body( world, &floor ) == KF_OK );
        CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
        CHECK( character.support_velocity.x == 0 );
        CHECK( character.support_velocity.y == 0 );
        CHECK( character.support_velocity.z == 0 );
    }
    character.position.x = F( 6 );
    CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
    CHECK( character.grounded == KF_FALSE && character.position.y < 0 );
    CHECK( kf_world_remove_body( world, object ) == KF_OK );
    CHECK( add_box( world, 2, (kf_vec3_t){F( -5 ), F( -1 ), F( -5 )}, (kf_vec3_t){F( -1 ), 0, F( 5 )}, KF_BODY_STATIC, &object ) == KF_OK );
    CHECK( add_box( world, 3, (kf_vec3_t){F( 1 ), F( -1 ), F( -5 )}, (kf_vec3_t){F( 5 ), 0, F( 5 )}, KF_BODY_STATIC, &object ) == KF_OK );
    memset( &character, 0, sizeof( character ) );
    character.grounded = KF_TRUE;
    CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
    CHECK( character.position.y < 0 && character.grounded == KF_FALSE );
    kf_world_destroy( world );

    return 0;
}

static int test_steps_and_headroom( void )
{
    kf_world_t * world = make_world();
    kf_handle_t object;
    kf_character_state_t character;
    kf_character_settings_t settings = character_settings();
    kf_character_move_result_t result;
    kf_contact_t contacts[64];
    CHECK( world != NULL );
    settings.gravity = F( 20 );
    settings.step_height = F( .4 );
    CHECK( add_box( world, 1, (kf_vec3_t){F( -5 ), F( -1 ), F( -5 )}, (kf_vec3_t){F( 5 ), 0, F( 5 )}, KF_BODY_STATIC, &object ) == KF_OK );
    CHECK( add_box( world, 2, (kf_vec3_t){0, 0, F( -1 )}, (kf_vec3_t){F( 2 ), F( .25 ), F( 1 )}, KF_BODY_STATIC, &object ) == KF_OK );
    memset( &character, 0, sizeof( character ) );
    character.position.x = F( -1 );
    character.velocity.x = F( 25 );
    character.grounded = KF_TRUE;
    CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
    CHECK( result.stepped != KF_FALSE );
    CHECK( character.position.x > F( -.01 ) );
    CHECK( kf_fixed_abs( character.position.y - F( .25 ) ) < F( .01 ) );
    CHECK( character.velocity.x == F( 25 ) );
    CHECK( character.grounded != KF_FALSE );
    CHECK( add_box( world, 3, (kf_vec3_t){F( -2 ), F( 2.1 ), F( -2 )}, (kf_vec3_t){F( 2 ), F( 3 ), F( 2 )}, KF_BODY_STATIC, &object ) == KF_OK );
    memset( &character, 0, sizeof( character ) );
    character.position.x = F( -1 );
    character.velocity.x = F( 25 );
    character.grounded = KF_TRUE;
    CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
    CHECK( result.stepped == KF_FALSE );
    CHECK( character.position.x < F( -.15 ) );
    kf_world_destroy( world );

    return 0;
}

static int test_platform_carry_remove_change( void )
{
    kf_world_t * world = make_world();
    kf_handle_t object;
    kf_character_state_t character;
    kf_character_settings_t settings = character_settings();
    kf_character_move_result_t result;
    kf_contact_t contacts[64];
    kf_body_state_t platform;
    kf_shape_desc_t changed;
    CHECK( world != NULL );
    settings.gravity = F( 20 );
    CHECK( add_box( world, 1, (kf_vec3_t){F( -3 ), F( -1 ), F( -3 )}, (kf_vec3_t){F( 3 ), 0, F( 3 )}, KF_BODY_KINEMATIC, &object ) == KF_OK );
    memset( &character, 0, sizeof( character ) );
    character.position.x = F( 1 );
    CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
    CHECK( character.support.id == 1 );
    CHECK( kf_world_get_body( world, object, &platform ) == KF_OK );
    platform.pose.position.x = F( 1 );
    platform.pose.rotation = kf_rotation_from_yaw( KF_ANGLE16_QUARTER );
    platform.velocity.x = F( 25 );
    CHECK( kf_world_set_body( world, &platform ) == KF_OK );
    CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
    CHECK( kf_fixed_abs( character.position.x - F( 1 ) ) < F( .01 ) );
    CHECK( kf_fixed_abs( character.position.z - F( -1 ) ) < F( .01 ) );
    CHECK( character.support_velocity.x == F( 25 ) );
    CHECK( character.grounded != KF_FALSE );
    memset( &changed, 0, sizeof( changed ) );
    changed.id = 1;
    changed.shape.type = KF_SHAPE_AABB;
    changed.shape.data.aabb = (kf_aabb_t){{F( 10 ), F( -1 ), F( 10 )}, {F( 12 ), 0, F( 12 )}};
    CHECK( kf_world_update_shape( world, object, &changed ) == KF_OK );
    CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
    CHECK( character.grounded == KF_FALSE && character.support.id == 0 );
    CHECK( kf_world_remove_body( world, object ) == KF_OK );
    CHECK( add_box( world, 2, (kf_vec3_t){F( -3 ), F( -1 ), F( -3 )}, (kf_vec3_t){F( 3 ), 0, F( 3 )}, KF_BODY_KINEMATIC, &object ) == KF_OK );
    memset( &character, 0, sizeof( character ) );
    CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
    CHECK( character.grounded != KF_FALSE );
    CHECK( kf_world_remove_body( world, object ) == KF_OK );
    CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
    CHECK( character.grounded == KF_FALSE && character.position.y < 0 );
    kf_world_destroy( world );

    return 0;
}

static int test_ramp_and_remainder( void )
{
    kf_world_t * world = make_world();
    kf_character_state_t character;
    kf_character_settings_t settings = character_settings();
    kf_character_move_result_t result;
    kf_contact_t contacts[64];
    kf_body_desc_t body = kf_body_desc_default( 1 );
    kf_shape_desc_t triangles[2];
    kf_handle_t object;
    unsigned int tick;
    CHECK( world != NULL );
    memset( triangles, 0, sizeof( triangles ) );
    triangles[0].id = 1;
    triangles[0].shape.type = KF_SHAPE_TRIANGLE;
    triangles[0].shape.data.triangle.vertices[0] = (kf_vec3_t){F( -2 ), 0, F( -2 )};
    triangles[0].shape.data.triangle.vertices[1] = (kf_vec3_t){F( 2 ), 0, F( -2 )};
    triangles[0].shape.data.triangle.vertices[2] = (kf_vec3_t){F( 2 ), F( 2 ), F( 2 )};
    triangles[1] = triangles[0];
    triangles[1].id = 2;
    triangles[1].shape.data.triangle.vertices[1] = (kf_vec3_t){F( 2 ), F( 2 ), F( 2 )};
    triangles[1].shape.data.triangle.vertices[2] = (kf_vec3_t){F( -2 ), F( 2 ), F( 2 )};
    CHECK( kf_world_add_body( world, &body, triangles, 2, &object ) == KF_OK );
    memset( &character, 0, sizeof( character ) );
    settings.gravity = F( 20 );
    character.position = (kf_vec3_t){0, F( .54 ), F( -1 )};
    character.velocity.z = F( 25 );
    character.grounded = KF_TRUE;
    CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
    CHECK( character.position.y > F( .8 ) );
    CHECK( character.grounded != KF_FALSE );
    CHECK( character.support_normal.y > F( .8 ) );
    CHECK( kf_world_remove_body( world, object ) == KF_OK );
    triangles[0].shape.data.triangle.vertices[2].y = F( 8 );
    triangles[1].shape.data.triangle.vertices[1].y = F( 8 );
    triangles[1].shape.data.triangle.vertices[2].y = F( 8 );
    CHECK( kf_world_add_body( world, &body, triangles, 2, &object ) == KF_OK );
    memset( &character, 0, sizeof( character ) );
    character.position = (kf_vec3_t){0, F( 2.32 ), F( -1 )};
    character.velocity.z = F( 12.5 );
    CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
    CHECK( character.position.y < F( 2.33 ) );
    CHECK( character.position.z < F( -.8 ) );
    CHECK( character.grounded == KF_FALSE );
    CHECK( kf_world_remove_body( world, object ) == KF_OK );
    settings.gravity = 0;
    memset( &character, 0, sizeof( character ) );
    character.velocity.x = 1;

    for( tick = 0; tick != 25; ++tick )
    {
        CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
    }

    CHECK( character.position.x == 1 );
    CHECK( character.position_remainder[0] == 0 );
    kf_world_destroy( world );

    return 0;
}

static int test_standing_on_slope( void )
{
    kf_world_t * world = make_world();
    kf_character_state_t character;
    kf_character_settings_t settings = character_settings();
    kf_character_move_result_t result;
    kf_contact_t contacts[64];
    kf_body_desc_t body = kf_body_desc_default( 1 );
    kf_shape_desc_t triangles[2];
    kf_handle_t object;
    kf_vec3_t landed;
    unsigned int tick;
    CHECK( world != NULL );
    memset( triangles, 0, sizeof( triangles ) );
    /* A walkable 30 degree ramp rising along +X. */
    triangles[0].id = 1;
    triangles[0].shape.type = KF_SHAPE_TRIANGLE;
    triangles[0].shape.data.triangle.vertices[0] = (kf_vec3_t){F( -5 ), F( -2.8867513 ), F( -5 )};
    triangles[0].shape.data.triangle.vertices[1] = (kf_vec3_t){F( 5 ), F( 2.8867513 ), F( -5 )};
    triangles[0].shape.data.triangle.vertices[2] = (kf_vec3_t){F( 5 ), F( 2.8867513 ), F( 5 )};
    triangles[1] = triangles[0];
    triangles[1].id = 2;
    triangles[1].shape.data.triangle.vertices[1] = (kf_vec3_t){F( 5 ), F( 2.8867513 ), F( 5 )};
    triangles[1].shape.data.triangle.vertices[2] = (kf_vec3_t){F( -5 ), F( -2.8867513 ), F( 5 )};
    CHECK( kf_world_add_body( world, &body, triangles, 2, &object ) == KF_OK );
    memset( &character, 0, sizeof( character ) );
    settings.gravity = F( 20 );
    character.position = (kf_vec3_t){0, F( 1 ), 0};

    for( tick = 0; tick != 25; ++tick )
    {
        CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
    }

    CHECK( character.grounded != KF_FALSE );
    landed = character.position;

    /* Standing still on walkable ground: gravity does not slide the character downhill. */
    for( tick = 0; tick != 100; ++tick )
    {
        character.velocity.x = 0;
        character.velocity.z = 0;
        CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
        CHECK( character.grounded != KF_FALSE );
    }

    CHECK( character.position.x == landed.x && character.position.y == landed.y && character.position.z == landed.z );
    kf_world_destroy( world );

    return 0;
}

static int test_snap_and_order( void )
{
    kf_world_t * worlds[2] = {make_world(), make_world()};
    kf_character_state_t characters[2];
    kf_character_settings_t settings = character_settings();
    kf_character_move_result_t results[2];
    kf_contact_t contacts[2][64];
    kf_handle_t object;
    unsigned int order;

    settings.gravity = F( 20 );

    for( order = 0; order != 2; ++order )
    {
        CHECK( worlds[order] != NULL );

        if( order == 0 )
        {
            CHECK( add_box( worlds[order], 1, (kf_vec3_t){F( -3 ), F( -1 ), F( -2 )}, (kf_vec3_t){0, 0, F( 2 )}, KF_BODY_STATIC, &object ) == KF_OK );
            CHECK( add_box( worlds[order], 2, (kf_vec3_t){0, F( -1 ), F( -2 )}, (kf_vec3_t){F( 3 ), F( -.15 ), F( 2 )}, KF_BODY_STATIC, &object ) == KF_OK );
        }
        else
        {
            CHECK( add_box( worlds[order], 2, (kf_vec3_t){0, F( -1 ), F( -2 )}, (kf_vec3_t){F( 3 ), F( -.15 ), F( 2 )}, KF_BODY_STATIC, &object ) == KF_OK );
            CHECK( add_box( worlds[order], 1, (kf_vec3_t){F( -3 ), F( -1 ), F( -2 )}, (kf_vec3_t){0, 0, F( 2 )}, KF_BODY_STATIC, &object ) == KF_OK );
        }

        memset( &characters[order], 0, sizeof( characters[order] ) );
        characters[order].position.x = F( -.6 );
        characters[order].velocity.x = F( 25 );
        characters[order].grounded = KF_TRUE;
        CHECK( kf_character_move( worlds[order], &characters[order], &settings, NULL, contacts[order], 64, &results[order] ) == KF_OK );
        CHECK( characters[order].grounded != KF_FALSE );
        CHECK( kf_fixed_abs( characters[order].position.y - F( -.15 ) ) < F( .01 ) );
        CHECK( characters[order].support.id == 2 );
    }

    CHECK( memcmp( &characters[0], &characters[1], sizeof( characters[0] ) ) == 0 );
    CHECK( results[0].contact_count == results[1].contact_count );
    kf_world_destroy( worlds[0] );
    kf_world_destroy( worlds[1] );

    return 0;
}

static int test_ledge_guard_and_landing( void )
{
    kf_world_t * world = make_world();
    kf_handle_t object;
    kf_character_state_t character;
    kf_character_settings_t settings = character_settings();
    kf_character_move_result_t result;
    kf_contact_t contacts[64];
    unsigned index;
    CHECK( world != NULL );
    settings.gravity = F( 20 );
    settings.ledge_drop = F( 1 );
    /* A pit beyond x = 2 and a terrace half a unit lower beyond x = -2. */
    CHECK( add_box( world, 1, (kf_vec3_t){F( -2 ), F( -1 ), F( -5 )}, (kf_vec3_t){F( 2 ), 0, F( 5 )}, KF_BODY_STATIC, &object ) == KF_OK );
    CHECK( add_box( world, 2, (kf_vec3_t){F( -6 ), F( -2 ), F( -5 )}, (kf_vec3_t){F( -2 ), F( -.5 ), F( 5 )}, KF_BODY_STATIC, &object ) == KF_OK );
    memset( &character, 0, sizeof( character ) );
    CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
    CHECK( character.grounded != KF_FALSE );
    for( index = 0; index != 60; ++index )
    {
        character.velocity.x = F( 6 );
        CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
    }
    CHECK( result.held_at_ledge != KF_FALSE && character.grounded != KF_FALSE );
    CHECK( character.velocity.x == 0 );
    CHECK( character.position.x > F( 1.8 ) && character.position.x <= F( 2 ) );
    CHECK( character.position.y == 0 );
    /* The terrace drop is within the ledge drop, so the character walks off and lands. */
    for( index = 0; index != 120 && result.landed == KF_FALSE; ++index )
    {
        character.velocity.x = -F( 6 );
        CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
        CHECK( result.held_at_ledge == KF_FALSE );
    }
    CHECK( result.landed != KF_FALSE && character.support.id == 2 );
    CHECK( character.position.y >= F( -.5 ) && character.position.y <= F( -.5 ) + settings.skin );
    CHECK( result.landing_speed > F( 2 ) && result.landing_speed < F( 6 ) );
    CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
    CHECK( result.landed == KF_FALSE && result.landing_speed == 0 );
    settings.ledge_drop = -1;
    CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_INVALID_ARGUMENT );
    kf_world_destroy( world );

    return 0;
}

static int test_carry_subfraction_hit_order( void )
{
    kf_world_t * world = make_world();
    kf_handle_t platform;
    kf_handle_t obstacle;
    kf_character_state_t character = {0};
    kf_character_settings_t settings = character_settings();
    kf_character_move_result_t result;
    kf_contact_t contacts[32];
    kf_body_state_t state;
    size_t first_obstacle = SIZE_MAX;
    size_t index;
    CHECK( world != NULL );
    CHECK( add_box( world, 1, (kf_vec3_t){F(-200), F(-1), F(-5)}, (kf_vec3_t){F(200), 0, F(5)}, KF_BODY_KINEMATIC, &platform ) == KF_OK );
    CHECK( add_box( world, 2, (kf_vec3_t){F(50.001), 0, F(-2)}, (kf_vec3_t){F(51), F(3), F(2)}, KF_BODY_STATIC, &obstacle ) == KF_OK );
    CHECK( add_box( world, 3, (kf_vec3_t){F(50), 0, F(-2)}, (kf_vec3_t){F(51), F(3), F(2)}, KF_BODY_STATIC, &obstacle ) == KF_OK );
    CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 32, &result ) == KF_OK );
    CHECK( character.grounded == KF_TRUE );
    CHECK( kf_world_get_body( world, platform, &state ) == KF_OK );
    state.pose.position.x = F(100);
    CHECK( kf_world_set_body( world, &state ) == KF_OK );
    CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 32, &result ) == KF_OK );
    for( index = 0; index != result.contact_count; ++index )
    {
        if( contacts[index].object.id == platform.id )
        {
            continue;
        }
        first_obstacle = index;
        break;
    }
    CHECK( first_obstacle != SIZE_MAX );
    CHECK( contacts[first_obstacle].object.id == 3 );
    CHECK( character.position.x <= F(50) - settings.radius + KF_GEOMETRY_TOLERANCE );
    kf_world_destroy( world );
    return 0;
}

static int test_slide_subfraction_remainder( void )
{
    kf_world_t * world = make_world();
    kf_handle_t obstacle;
    kf_character_state_t character = {0};
    kf_character_settings_t settings = character_settings();
    kf_character_move_result_t result;
    kf_contact_t contacts[32];
    CHECK( world != NULL );
    settings.snap_distance = 0;
    character.velocity = (kf_vec3_t){F(1000), 0, F(500)};
    CHECK( add_box( world, 1, (kf_vec3_t){F(10.0003), F(-1), F(-100)}, (kf_vec3_t){F(11), F(3), F(100)}, KF_BODY_STATIC, &obstacle ) == KF_OK );
    CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 32, &result ) == KF_OK );
    CHECK( result.contact_count != 0 );
    CHECK( character.position.x <= F(10.0003) - settings.radius + KF_GEOMETRY_TOLERANCE );
    CHECK( kf_fixed_abs( character.position.z - F(20) ) <= 2 );
    kf_world_destroy( world );
    return 0;
}

static int test_invalid_saved_support_rotation( void )
{
    static const kf_rotation_t invalid[] = {
        {INT32_C(1073741825), 0, 0},
        {INT32_MIN, 0, 0},
        {INT32_MAX, INT32_MAX, 0},
        {INT32_MIN, INT32_MIN, INT32_MIN}
    };
    kf_world_t * world = make_world();
    kf_handle_t platform;
    kf_character_state_t character = {0};
    kf_character_state_t supported;
    kf_character_settings_t settings = character_settings();
    kf_character_move_result_t result;
    kf_contact_t contacts[32];
    size_t index;
    CHECK( world != NULL );
    CHECK( add_box( world, 1, (kf_vec3_t){F(-5), F(-1), F(-5)}, (kf_vec3_t){F(5), 0, F(5)}, KF_BODY_KINEMATIC, &platform ) == KF_OK );
    CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 32, &result ) == KF_OK );
    CHECK( character.grounded == KF_TRUE );
    supported = character;
    for( index = 0; index != sizeof(invalid) / sizeof(*invalid); ++index )
    {
        unsigned char before[sizeof(character)];
        character = supported;
        character.support_pose.rotation = invalid[index];
        memcpy( before, &character, sizeof(character) );
        CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 32, &result ) == KF_INVALID_ARGUMENT );
        CHECK( memcmp( before, &character, sizeof(character) ) == 0 );
        CHECK( kf_world_tick( world ) == 0 );
    }
    character = supported;
    CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 32, &result ) == KF_OK );
    CHECK( character.grounded == KF_TRUE );
    kf_world_destroy( world );
    return 0;
}

static int test_slope_velocity_limits( void )
{
    int direction;
    unsigned int ceiling;

    for( ceiling = 0; ceiling != 2; ++ceiling )
    {
        for( direction = -1; direction <= 1; direction += 2 )
        {
            kf_world_t * world = make_world();
            kf_body_desc_t body = kf_body_desc_default( 1 );
            kf_shape_desc_t shape = {0};
            kf_handle_t obstacle;
            kf_character_state_t character = {0};
            kf_character_settings_t settings = character_settings();
            kf_character_move_result_t result;
            kf_contact_t contacts[32];
            kf_fixed_t angle = direction * (KF_FIXED_PI / 8);
            CHECK( world != NULL );

            if( ceiling == 0 )
            {
                angle = -angle;
            }

            body.pose.rotation = kf_rotation_from_axis_angle( (kf_vec3_t){0, 0, KF_FIXED_SCALE}, angle );
            shape.id = 1;
            shape.shape.type = KF_SHAPE_AABB;
            shape.shape.data.aabb = (kf_aabb_t){{F(-100), ceiling != 0 ? 0 : F(-2), F(-100)}, {F(100), ceiling != 0 ? F(2) : 0, F(100)}};
            CHECK( kf_world_add_body( world, &body, &shape, 1, &obstacle ) == KF_OK );
            character.position = (kf_vec3_t){-direction * F(10), ceiling != 0 ? F(-12) : F(10), 0};
            character.velocity = (kf_vec3_t){direction * KF_SPEED_LIMIT, ceiling != 0 ? KF_SPEED_LIMIT : -KF_SPEED_LIMIT, KF_SPEED_LIMIT};
            CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 32, &result ) == KF_OK );
            CHECK( result.contact_count != 0 );
            CHECK( kf_fixed_abs( character.velocity.x ) <= KF_SPEED_LIMIT );
            CHECK( kf_fixed_abs( character.velocity.y ) <= KF_SPEED_LIMIT );
            CHECK( kf_fixed_abs( character.velocity.z ) <= KF_SPEED_LIMIT );
            CHECK( direction * character.velocity.x > 0 );
            CHECK( character.velocity.z > 0 && character.velocity.z < KF_SPEED_LIMIT );
            CHECK( kf_vec3_dot( character.velocity, contacts[0].normal ) >= -KF_GEOMETRY_TOLERANCE );
            CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 32, &result ) == KF_OK );
            kf_world_destroy( world );
        }
    }

    return 0;
}

static int test_boundary_ground_probes( void )
{
    kf_fixed_t margins[] = {F(.1), 32, 0};
    size_t index;

    for( index = 0; index != sizeof(margins) / sizeof(*margins); ++index )
    {
        kf_world_t * world = make_world();
        kf_character_state_t character = {0};
        kf_character_settings_t settings = character_settings();
        kf_character_move_result_t result;
        kf_contact_t contacts[32];
        kf_handle_t obstacle;
        kf_fixed_t floor = -KF_COORDINATE_LIMIT + margins[index];
        kf_fixed_t previous;
        unsigned char before[sizeof(character)];
        CHECK( world != NULL );
        /* A touching wall precedes the floor in canonical order. */
        CHECK( add_box( world, 1, (kf_vec3_t){settings.radius, floor, F(-2)}, (kf_vec3_t){F(1), floor + settings.height, F(2)}, KF_BODY_STATIC, &obstacle ) == KF_OK );
        CHECK( add_box( world, 2, (kf_vec3_t){F(-5), -KF_COORDINATE_LIMIT, F(-5)}, (kf_vec3_t){F(5), floor, F(5)}, KF_BODY_STATIC, &obstacle ) == KF_OK );
        character.position.y = floor;
        CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 32, &result ) == KF_OK );
        CHECK( character.grounded == KF_TRUE && character.support.id == 2 );
        character.velocity.x = -F(1);
        CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 32, &result ) == KF_OK );
        CHECK( character.position.x < 0 && character.position.y == floor );
        CHECK( character.grounded == KF_TRUE && character.support.id == 2 );
        settings.ledge_drop = F(1);
        previous = character.position.x;
        CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 32, &result ) == KF_OK );
        CHECK( character.position.x < previous && character.position.y == floor );
        CHECK( result.held_at_ledge == KF_FALSE && character.grounded == KF_TRUE );

        /* Only internal probes are shortened; invalid actual motion rolls back. */
        character.grounded = KF_FALSE;
        character.support = (kf_handle_t){0, 0};
        character.velocity.y = -F(100);
        memcpy( before, &character, sizeof(character) );
        CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 32, &result ) == KF_OUT_OF_RANGE );
        CHECK( memcmp( before, &character, sizeof(character) ) == 0 );
        kf_world_destroy( world );
    }

    {
        kf_world_t * world = make_world();
        kf_character_state_t character = {0};
        kf_character_settings_t settings = character_settings();
        kf_character_move_result_t result;
        CHECK( world != NULL );
        character.position.y = -KF_COORDINATE_LIMIT;
        CHECK( kf_character_move( world, &character, &settings, NULL, NULL, 0, &result ) == KF_OK );
        CHECK( character.grounded == KF_FALSE );
        kf_world_destroy( world );
    }

    return 0;
}

static int test_boundary_step_probes( void )
{
    unsigned int scenario;

    for( scenario = 0; scenario != 3; ++scenario )
    {
        kf_world_t * world = make_world();
        kf_character_state_t character = {0};
        kf_character_settings_t settings = character_settings();
        kf_character_move_result_t result;
        kf_contact_t contacts[32];
        kf_handle_t obstacle;
        kf_fixed_t floor = scenario == 0 ? -KF_COORDINATE_LIMIT + F(.1) : KF_COORDINATE_LIMIT - settings.height;
        kf_fixed_t step = F(.25);
        CHECK( world != NULL );

        if( scenario == 1 )
        {
            floor -= F(.3);
        }

        settings.step_height = F(.4);
        CHECK( add_box( world, 1, (kf_vec3_t){F(-5), floor - F(.1), F(-5)}, (kf_vec3_t){F(5), floor, F(5)}, KF_BODY_STATIC, &obstacle ) == KF_OK );
        CHECK( add_box( world, 2, (kf_vec3_t){0, floor, F(-1)}, (kf_vec3_t){F(2), floor + step, F(1)}, KF_BODY_STATIC, &obstacle ) == KF_OK );
        character.position = (kf_vec3_t){-F(1), floor, 0};
        CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 32, &result ) == KF_OK );
        CHECK( character.grounded == KF_TRUE );
        character.velocity.x = F(25);
        CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 32, &result ) == KF_OK );
        CHECK( character.position.y + settings.height <= KF_COORDINATE_LIMIT );
        CHECK( character.grounded == KF_TRUE );

        if( scenario == 2 )
        {
            CHECK( result.stepped == KF_FALSE );
            CHECK( character.position.x < 0 && character.position.y == floor );
        }
        else
        {
            CHECK( result.stepped == KF_TRUE );
            CHECK( character.position.x > -F(.01) );
            CHECK( kf_fixed_abs( character.position.y - floor - step ) < F(.01) );
        }

        kf_world_destroy( world );
    }

    return 0;
}

static int test_stairs_at_100_hz( void )
{
    kf_world_t * world = NULL;
    kf_world_config_t config = kf_world_config_default();
    kf_character_settings_t settings = kf_character_settings_default();
    kf_character_state_t character;
    kf_character_move_result_t result;
    kf_contact_t contacts[64];
    kf_handle_t handle;
    uint32_t tick;
    config.step = (kf_timestep_t){1, 100};
    CHECK( kf_world_create( &config, NULL, &world ) == KF_OK );
    CHECK( kf_world_reserve( world, (kf_capacity_t){8, 8, 0, 1, 1} ) == KF_OK );
    CHECK( add_box( world, 1, (kf_vec3_t){F( -3 ), F( -1 ), F( -3 )}, (kf_vec3_t){F( 3 ), 0, 0}, KF_BODY_STATIC, &handle ) == KF_OK );
    CHECK( add_box( world, 2, (kf_vec3_t){F( -3 ), F( -1 ), 0}, (kf_vec3_t){F( 3 ), F( .5 ), F( 1 )}, KF_BODY_STATIC, &handle ) == KF_OK );
    CHECK( add_box( world, 3, (kf_vec3_t){F( -3 ), F( -1 ), F( 1 )}, (kf_vec3_t){F( 3 ), F( 1 ), F( 2 )}, KF_BODY_STATIC, &handle ) == KF_OK );
    CHECK( add_box( world, 4, (kf_vec3_t){F( -3 ), F( -1 ), F( 2 )}, (kf_vec3_t){F( 3 ), F( 1.5 ), F( 8 )}, KF_BODY_STATIC, &handle ) == KF_OK );
    settings.radius = F( .35 );
    settings.height = F( 1.8 );
    settings.step_height = F( .5625 );
    settings.snap_distance = settings.step_height;
    settings.minimum_ground_normal_y = F( .694 );
    memset( &character, 0, sizeof( character ) );
    character.position.z = F( -1 );
    character.grounded = KF_TRUE;
    for( tick = 0; tick != 80; ++tick )
    {
        character.velocity.z = F( 8 );
        CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
    }
    CHECK( character.position.z > F( 5 ) && character.position.y >= F( 1.5 ) );
    CHECK( character.grounded == KF_TRUE );
    for( tick = 0; tick != 80; ++tick )
    {
        character.velocity.z = F( -8 );
        CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
    }
    CHECK( character.position.z < F( -.9 ) && character.position.y <= F( .01 ) );
    CHECK( character.grounded == KF_TRUE );
    {
        kf_shape_desc_t step;
        memset( &step, 0, sizeof( step ) );
        step.id = 1;
        step.shape.type = KF_SHAPE_AABB;
        step.shape.data.aabb = (kf_aabb_t){{F( -3 ), F( -1 ), 0}, {F( 3 ), F( .75 ), F( 1 )}};
        CHECK( kf_world_update_shape( world, (kf_handle_t){2, 1}, &step ) == KF_OK );
        memset( &character, 0, sizeof( character ) );
        character.position.z = F( -1 );
        character.grounded = KF_TRUE;
        for( tick = 0; tick != 40; ++tick )
        {
            character.velocity.z = F( 8 );
            CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
            CHECK( result.stepped == KF_FALSE );
        }
        CHECK( character.position.z < 0 && character.position.y == 0 );
        step.shape.data.aabb.maximum.y = F( .5 );
        CHECK( kf_world_update_shape( world, (kf_handle_t){2, 1}, &step ) == KF_OK );
        memset( &character, 0, sizeof( character ) );
        character.position.z = F( -1 );
        for( tick = 0; tick != 15; ++tick )
        {
            character.grounded = KF_FALSE;
            character.velocity = (kf_vec3_t){0, 0, F( 8 )};
            CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
            CHECK( result.stepped == KF_FALSE );
        }
    }
    CHECK( add_box( world, 5, (kf_vec3_t){F( -3 ), F( 1.9 ), F( -3 )}, (kf_vec3_t){F( 3 ), F( 2.5 ), F( 2 )}, KF_BODY_STATIC, &handle ) == KF_OK );
    memset( &character, 0, sizeof( character ) );
    character.position.z = F( -1 );
    character.grounded = KF_TRUE;
    for( tick = 0; tick != 40; ++tick )
    {
        character.velocity.z = F( 8 );
        CHECK( kf_character_move( world, &character, &settings, NULL, contacts, 64, &result ) == KF_OK );
        CHECK( result.stepped == KF_FALSE );
    }
    CHECK( character.position.z < 0 );
    kf_world_destroy( world );

    return 0;
}

int main( void )
{
    CHECK( test_stairs_at_100_hz() == 0 );
    CHECK( test_slope_velocity_limits() == 0 );
    CHECK( test_boundary_ground_probes() == 0 );
    CHECK( test_boundary_step_probes() == 0 );
    CHECK( test_invalid_saved_support_rotation() == 0 );
    CHECK( test_carry_subfraction_hit_order() == 0 );
    CHECK( test_slide_subfraction_remainder() == 0 );
    CHECK( test_diagonal_and_planes() == 0 );
    CHECK( test_depenetration_and_character() == 0 );
    CHECK( test_floor_edge_hole_and_capacity() == 0 );
    CHECK( test_steps_and_headroom() == 0 );
    CHECK( test_platform_carry_remove_change() == 0 );
    CHECK( test_ramp_and_remainder() == 0 );
    CHECK( test_standing_on_slope() == 0 );
    CHECK( test_snap_and_order() == 0 );
    CHECK( test_ledge_guard_and_landing() == 0 );
    puts( "kinefix character tests passed" );

    return 0;
}
