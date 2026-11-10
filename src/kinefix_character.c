#include "kinefix_internal.h"

#include <string.h>

#define KF_CHARACTER_MAX_ITERATIONS 32u
#define KF_CHARACTER_MAX_PLANES (KF_CHARACTER_MAX_ITERATIONS * 2u)

typedef struct kf_character_contacts_t
{
    kf_contact_t * output;
    size_t capacity;
    size_t count;
} kf_character_contacts_t;

static kf_status_t kf_character_validate( const kf_character_state_t * _state, const kf_character_settings_t * _settings )
{
    if( _state->grounded > KF_TRUE || _settings->constrain_to_region > KF_TRUE )
    {
        return KF_INVALID_ARGUMENT;
    }

    if( _settings->radius <= 0 )
    {
        return KF_INVALID_ARGUMENT;
    }

    if( _settings->height < (int64_t)_settings->radius * 2 )
    {
        return KF_INVALID_ARGUMENT;
    }

    if( _settings->height > KF_SIZE_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }

    if( _settings->step_height < 0 || _settings->snap_distance < 0 || _settings->skin < 0 || _settings->gravity < 0 || _settings->maximum_depenetration < 0 || _settings->ledge_drop < 0 )
    {
        return KF_INVALID_ARGUMENT;
    }

    if( _settings->step_height > KF_SIZE_LIMIT || _settings->snap_distance > KF_SIZE_LIMIT || _settings->skin > _settings->radius || _settings->gravity > KF_SPEED_LIMIT || _settings->maximum_depenetration > KF_SIZE_LIMIT || _settings->ledge_drop > KF_COORDINATE_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }

    if( _settings->minimum_ground_normal_y <= 0 || _settings->minimum_ground_normal_y > KF_FIXED_SCALE )
    {
        return KF_INVALID_ARGUMENT;
    }

    if( _settings->slide_iterations == 0 || _settings->slide_iterations > KF_CHARACTER_MAX_ITERATIONS || _settings->depenetration_iterations == 0 || _settings->depenetration_iterations > KF_CHARACTER_MAX_ITERATIONS )
    {
        return KF_INVALID_ARGUMENT;
    }

    if( _state->position.x < -KF_COORDINATE_LIMIT || _state->position.x > KF_COORDINATE_LIMIT || _state->position.y < -KF_COORDINATE_LIMIT || _state->position.y > KF_COORDINATE_LIMIT || _state->position.z < -KF_COORDINATE_LIMIT || _state->position.z > KF_COORDINATE_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }

    if( _state->velocity.x < -KF_SPEED_LIMIT || _state->velocity.x > KF_SPEED_LIMIT || _state->velocity.y < -KF_SPEED_LIMIT || _state->velocity.y > KF_SPEED_LIMIT || _state->velocity.z < -KF_SPEED_LIMIT || _state->velocity.z > KF_SPEED_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }

    if( _state->support.id != 0 )
    {
        if( _state->support_pose.position.x < -KF_COORDINATE_LIMIT || _state->support_pose.position.x > KF_COORDINATE_LIMIT || _state->support_pose.position.y < -KF_COORDINATE_LIMIT || _state->support_pose.position.y > KF_COORDINATE_LIMIT || _state->support_pose.position.z < -KF_COORDINATE_LIMIT || _state->support_pose.position.z > KF_COORDINATE_LIMIT )
        {
            return KF_OUT_OF_RANGE;
        }

        if( kf_rotation_valid( _state->support_pose.rotation ) == KF_FALSE )
        {
            return KF_INVALID_ARGUMENT;
        }
    }

    if( _settings->constrain_to_region != KF_FALSE )
    {
        if( _settings->region.minimum.x < -KF_COORDINATE_LIMIT || _settings->region.minimum.y < -KF_COORDINATE_LIMIT || _settings->region.minimum.z < -KF_COORDINATE_LIMIT || _settings->region.maximum.x > KF_COORDINATE_LIMIT || _settings->region.maximum.y > KF_COORDINATE_LIMIT || _settings->region.maximum.z > KF_COORDINATE_LIMIT )
        {
            return KF_OUT_OF_RANGE;
        }

        if( (int64_t)_settings->region.maximum.x - _settings->region.minimum.x < (int64_t)_settings->radius * 2 || (int64_t)_settings->region.maximum.y - _settings->region.minimum.y < _settings->height || (int64_t)_settings->region.maximum.z - _settings->region.minimum.z < (int64_t)_settings->radius * 2 )
        {
            return KF_INVALID_ARGUMENT;
        }
    }

    return KF_OK;
}

static uint64_t kf_character_hash_scalar( uint64_t _hash, uint32_t _value )
{
    _hash ^= _value;
    _hash *= UINT64_C(1099511628211);

    return _hash;
}

static uint64_t kf_character_hash_vector( uint64_t _hash, kf_vec3_t _vector )
{
    _hash = kf_character_hash_scalar( _hash, (uint32_t)_vector.x );
    _hash = kf_character_hash_scalar( _hash, (uint32_t)_vector.y );
    _hash = kf_character_hash_scalar( _hash, (uint32_t)_vector.z );

    return _hash;
}

/* Hash fields rather than structure bytes: padding is not part of saved state. */
static uint64_t kf_character_shape_signature( const kf_shape_t * _shape )
{
    uint64_t hash = kf_character_hash_scalar( UINT64_C(14695981039346656037), (uint32_t)_shape->type );

    switch( _shape->type )
    {
    case KF_SHAPE_AABB:
        hash = kf_character_hash_vector( hash, _shape->data.aabb.minimum );
        return kf_character_hash_vector( hash, _shape->data.aabb.maximum );
    case KF_SHAPE_SPHERE:
        hash = kf_character_hash_vector( hash, _shape->data.sphere.position );
        return kf_character_hash_scalar( hash, (uint32_t)_shape->data.sphere.radius );
    case KF_SHAPE_CAPSULE:
        hash = kf_character_hash_vector( hash, _shape->data.capsule.position );
        hash = kf_character_hash_scalar( hash, (uint32_t)_shape->data.capsule.radius );
        return kf_character_hash_scalar( hash, (uint32_t)_shape->data.capsule.height );
    case KF_SHAPE_BOX:
        hash = kf_character_hash_vector( hash, _shape->data.box.center );
        hash = kf_character_hash_vector( hash, _shape->data.box.half_extents );
        return kf_character_hash_scalar( hash, _shape->data.box.yaw );
    case KF_SHAPE_PRISM:
    {
        uint32_t index;
        hash = kf_character_hash_scalar( hash, _shape->data.prism.count );

        for( index = 0; index != _shape->data.prism.count; ++index )
        {
            hash = kf_character_hash_vector( hash, _shape->data.prism.vertices[index] );
        }

        hash = kf_character_hash_scalar( hash, (uint32_t)_shape->data.prism.minimum_y );
        return kf_character_hash_scalar( hash, (uint32_t)_shape->data.prism.maximum_y );
    }
    case KF_SHAPE_TRIANGLE:
        hash = kf_character_hash_vector( hash, _shape->data.triangle.vertices[0] );
        hash = kf_character_hash_vector( hash, _shape->data.triangle.vertices[1] );
        return kf_character_hash_vector( hash, _shape->data.triangle.vertices[2] );
    default:
        return 0;
    }
}

static const kf_shape_slot_t * kf_character_find_shape( const kf_world_t * _world, kf_handle_t _object, uint32_t _shapeId )
{
    size_t low = 0;
    size_t high = _world->shape_count;
    const kf_shape_slot_t * shape;

    while( low < high )
    {
        size_t middle = low + (high - low) / 2;
        const kf_shape_slot_t * candidate = &_world->shapes[middle];

        if( candidate->object_id < _object.id || (candidate->object_id == _object.id && candidate->desc.id < _shapeId) )
        {
            low = middle + 1;
        }
        else
        {
            high = middle;
        }
    }

    if( low == _world->shape_count )
    {
        return NULL;
    }

    shape = &_world->shapes[low];

    if( shape->object_id != _object.id || shape->desc.id != _shapeId )
    {
        return NULL;
    }

    return shape;
}

/* A capsule's rounded rim changes the contact normal at a stair lip. Classify
   ground using the adjacent obstacle surface, without admitting vertical walls. */
static kf_status_t kf_character_ground_normal( const kf_world_t * _world, kf_pose_t _pose, kf_sweep_hit_t * _hit )
{
    const kf_shape_slot_t * shape;
    const kf_object_slot_t * object;
    kf_ray_t ray;
    kf_sweep_hit_t surface;
    kf_vec3_t inward;
    kf_status_t status;

    if( _hit->normal.y <= 0 || _hit->normal.y == KF_FIXED_SCALE )
    {
        return KF_OK;
    }

    shape = kf_character_find_shape( _world, _hit->object, _hit->shape_id );
    object = kf_internal_object_const( _world, _hit->object );

    if( shape == NULL || object == NULL )
    {
        return KF_STALE_HANDLE;
    }

    inward = kf_vec3_sub( _hit->contact_point, _pose.position );
    inward.y = 0;
    inward = kf_vec3_mul( kf_vec3_normalize( inward ), KF_GEOMETRY_TOLERANCE * 2 );
    ray.origin = kf_vec3_add( _hit->contact_point, inward );
    ray.origin.x = kf_fixed_clamp( ray.origin.x, -KF_COORDINATE_LIMIT, KF_COORDINATE_LIMIT );
    ray.origin.y = kf_fixed_min( ray.origin.y + KF_GEOMETRY_TOLERANCE * 2, KF_COORDINATE_LIMIT );
    ray.origin.z = kf_fixed_clamp( ray.origin.z, -KF_COORDINATE_LIMIT, KF_COORDINATE_LIMIT );
    ray.direction = (kf_vec3_t){0, -KF_FIXED_SCALE, 0};
    ray.maximum_distance = kf_fixed_min( KF_GEOMETRY_TOLERANCE * 4, ray.origin.y + KF_COORDINATE_LIMIT );
    status = kf_shape_raycast( &ray, &shape->desc.shape, object->state.pose, &surface );

    if( status == KF_NO_HIT )
    {
        return KF_OK;
    }

    if( status != KF_OK )
    {
        return status;
    }

    _hit->normal = surface.normal;

    return KF_OK;
}

static void kf_character_clear_support( kf_character_state_t * _state )
{
    _state->grounded = KF_FALSE;
    _state->support = (kf_handle_t){0, 0};
    _state->support_shape_id = 0;
    _state->support_shape_signature = 0;
    _state->support_normal = (kf_vec3_t){0, 0, 0};
    _state->support_velocity = (kf_vec3_t){0, 0, 0};
    _state->support_pose = (kf_pose_t){{0, 0, 0}, {0, 0, 0}};
}

static kf_bool_t kf_character_filter_body( const kf_object_slot_t * _object, const kf_query_filter_t * _filter, kf_handle_t _ignored )
{
    size_t index;

    if( _object->active == KF_FALSE || _object->desc.trigger != KF_FALSE )
    {
        return KF_FALSE;
    }

    if( _object->desc.id == _ignored.id && _object->state.handle.generation == _ignored.generation )
    {
        return KF_FALSE;
    }

    if( (_object->desc.category & _filter->mask) == 0 || (_filter->category & _object->desc.mask) == 0 )
    {
        return KF_FALSE;
    }

    for( index = 0; index != _filter->exclude_count; ++index )
    {
        if( _filter->exclude[index].id != _object->desc.id )
        {
            continue;
        }

        if( _filter->exclude[index].generation != _object->state.handle.generation )
        {
            continue;
        }

        return KF_FALSE;
    }

    return KF_TRUE;
}

/* Carry excludes the supporting object without allocating an exclusion list. */
static kf_status_t kf_character_sweep( kf_world_t * _world, const kf_shape_t * _capsule, kf_pose_t _pose, kf_vec3_t _displacement, const kf_query_filter_t * _filter, kf_handle_t _ignored, kf_sweep_hit_t * _hit )
{
    size_t index;
    kf_bool_t found = KF_FALSE;

    if( _ignored.id == 0 )
    {
        return kf_world_sweep_first( _world, _capsule, _pose, _displacement, _filter, _hit );
    }

    for( index = 0; index != _world->shape_count; ++index )
    {
        const kf_shape_slot_t * shape = &_world->shapes[index];
        kf_object_slot_t * object = kf_internal_object_id( _world, shape->object_id );
        kf_sweep_hit_t hit;
        kf_status_t status;

        if( object == NULL )
        {
            continue;
        }

        if( kf_character_filter_body( object, _filter, _ignored ) == KF_FALSE )
        {
            continue;
        }

        status = kf_shape_sweep( _capsule, _pose, _displacement, &shape->desc.shape, object->state.pose, &hit );

        if( status == KF_NO_HIT )
        {
            continue;
        }

        if( status != KF_OK )
        {
            return status;
        }

        if( found != KF_FALSE && hit.fraction_q32 >= _hit->fraction_q32 )
        {
            continue;
        }

        hit.object = object->state.handle;
        hit.shape_id = shape->desc.id;
        *_hit = hit;
        found = KF_TRUE;
    }

    return found != KF_FALSE ? KF_OK : KF_NO_HIT;
}

static void kf_character_append_hit( kf_character_contacts_t * _contacts, const kf_sweep_hit_t * _hit )
{
    if( _contacts->count < _contacts->capacity )
    {
        kf_contact_t * contact = &_contacts->output[_contacts->count];
        contact->normal = _hit->normal;
        contact->depth = 0;
        contact->point_right = _hit->contact_point;
        contact->point_left = _hit->contact_point;
        contact->object = _hit->object;
        contact->shape_id = _hit->shape_id;
    }

    ++_contacts->count;
}

static kf_vec3_t kf_character_clip( kf_vec3_t _vector, const kf_vec3_t * _planes, uint32_t _count )
{
    kf_vec3_t result = _vector;
    uint32_t index;

    for( index = 0; index != _count; ++index )
    {
        kf_fixed_t into = kf_vec3_dot( result, _planes[index] );
        uint32_t previous;

        if( into >= 0 )
        {
            continue;
        }

        result = kf_vec3_sub_mul( result, _planes[index], into );

        for( previous = 0; previous != index; ++previous )
        {
            kf_vec3_t crease;
            kf_fixed_t along;
            uint32_t check;

            if( kf_vec3_dot( result, _planes[previous] ) >= -KF_GEOMETRY_TOLERANCE )
            {
                continue;
            }

            crease = kf_vec3_cross( _planes[index], _planes[previous] );
            crease = kf_vec3_normalize( crease );
            along = kf_vec3_dot( _vector, crease );
            result = kf_vec3_mul( crease, along );

            for( check = 0; check != _count; ++check )
            {
                if( kf_vec3_dot( result, _planes[check] ) >= -KF_GEOMETRY_TOLERANCE )
                {
                    continue;
                }

                return (kf_vec3_t){0, 0, 0};
            }
        }
    }

    return result;
}

static kf_vec3_t kf_character_limit_velocity( kf_vec3_t _velocity )
{
    kf_fixed_t maximum = kf_fixed_abs( _velocity.x );
    kf_fixed_t vertical = kf_fixed_abs( _velocity.y );
    kf_fixed_t depth = kf_fixed_abs( _velocity.z );
    maximum = kf_fixed_max( maximum, vertical );
    maximum = kf_fixed_max( maximum, depth );

    if( maximum <= KF_SPEED_LIMIT )
    {
        return _velocity;
    }

    /* Scaling all components preserves the clipped contact direction. */
    return kf_vec3_mul_div( _velocity, KF_SPEED_LIMIT, maximum );
}

static kf_status_t kf_character_clip_vertical_probe( const kf_shape_t * _capsule, kf_pose_t _pose, kf_fixed_t * _distance )
{
    kf_aabb_t bounds;
    kf_status_t status = kf_shape_bounds( _capsule, _pose, &bounds );

    if( status != KF_OK )
    {
        return status;
    }

    *_distance = kf_fixed_clamp( *_distance, -KF_COORDINATE_LIMIT - bounds.minimum.y, KF_COORDINATE_LIMIT - bounds.maximum.y );

    return KF_OK;
}

/* A floor can touch the capsule exactly at the lower world boundary, where a
   downward sweep has no admissible distance. Keep the normal and shape checks
   used by an ordinary ground probe, in canonical shape order. */
static kf_status_t kf_character_touching_support( kf_world_t * _world, const kf_shape_t * _capsule, kf_pose_t _pose, const kf_query_filter_t * _filter, kf_fixed_t _minimumNormalY, kf_sweep_hit_t * _hit )
{
    size_t index;

    for( index = 0; index != _world->shape_count; ++index )
    {
        const kf_shape_slot_t * shape = &_world->shapes[index];
        const kf_object_slot_t * object = kf_internal_object_id( _world, shape->object_id );
        kf_contact_t contact;
        kf_status_t status;

        if( object == NULL )
        {
            continue;
        }

        if( kf_character_filter_body( object, _filter, (kf_handle_t){0, 0} ) == KF_FALSE )
        {
            continue;
        }

        status = kf_shape_contact( _capsule, _pose, &shape->desc.shape, object->state.pose, &contact );

        if( status == KF_NO_HIT )
        {
            continue;
        }

        if( status != KF_OK )
        {
            return status;
        }

        if( contact.normal.y < _minimumNormalY )
        {
            continue;
        }

        memset( _hit, 0, sizeof(*_hit) );
        _hit->position = _pose.position;
        _hit->contact_point = contact.point_right;
        _hit->normal = contact.normal;
        _hit->object = object->state.handle;
        _hit->shape_id = shape->desc.id;

        return KF_OK;
    }

    return KF_NO_HIT;
}

static kf_status_t kf_character_slide( kf_world_t * _world, const kf_shape_t * _capsule, kf_pose_t * _pose, kf_vec3_t _displacement, kf_vec3_t * _velocity, const kf_character_settings_t * _settings, const kf_query_filter_t * _filter, kf_handle_t _ignored, kf_character_contacts_t * _contacts, kf_bool_t * _sideBlocked, kf_bool_t * _ceiling )
{
    kf_vec3_t planes[KF_CHARACTER_MAX_PLANES];
    uint32_t planeCount = 0;
    uint32_t iteration;

    for( iteration = 0; iteration != _settings->slide_iterations; ++iteration )
    {
        kf_sweep_hit_t hit;
        kf_vec3_t remaining;
        kf_vec3_t normal;
        kf_status_t status;

        if( _displacement.x == 0 && _displacement.y == 0 && _displacement.z == 0 )
        {
            return KF_OK;
        }

        status = kf_character_sweep( _world, _capsule, *_pose, _displacement, _filter, _ignored, &hit );

        if( status == KF_NO_HIT )
        {
            return kf_internal_vec_add( _pose->position, _displacement, &_pose->position );
        }

        if( status != KF_OK )
        {
            return status;
        }

        kf_character_append_hit( _contacts, &hit );
        _pose->position = hit.position;
        /* Rounded contact positions can round inward by a few fixed units. */
        if( (hit.normal.x != 0) + (hit.normal.y != 0) + (hit.normal.z != 0) > 1 )
        {
            status = kf_internal_vec_add( _pose->position, kf_vec3_mul( hit.normal, KF_GEOMETRY_TOLERANCE * 2 ), &_pose->position );

            if( status != KF_OK )
            {
                return status;
            }
        }

        {
            int64_t fraction = (INT64_C(1) << 32) - (int64_t)hit.fraction_q32;
            remaining.x = (kf_fixed_t)((int64_t)_displacement.x * fraction / (INT64_C(1) << 32));
            remaining.y = (kf_fixed_t)((int64_t)_displacement.y * fraction / (INT64_C(1) << 32));
            remaining.z = (kf_fixed_t)((int64_t)_displacement.z * fraction / (INT64_C(1) << 32));
        }
        normal = hit.normal;

        if( normal.y < _settings->minimum_ground_normal_y && normal.y >= 0 )
        {
            normal.y = 0;
            normal = kf_vec3_normalize( normal );
            *_sideBlocked = KF_TRUE;

            if( hit.normal.y > 0 )
            {
                planes[planeCount++] = normal;
                normal = hit.normal;
            }
        }

        if( hit.normal.y < 0 )
        {
            *_ceiling = KF_TRUE;
        }

        planes[planeCount++] = normal;
        remaining = kf_character_clip( remaining, planes, planeCount );
        *_velocity = kf_character_clip( *_velocity, planes, planeCount );
        *_velocity = kf_character_limit_velocity( *_velocity );

        if( kf_vec3_equal( remaining, _displacement ) != KF_FALSE )
        {
            return KF_ITERATION_LIMIT;
        }

        _displacement = remaining;
    }

    if( kf_vec3_length( _displacement ) > KF_GEOMETRY_TOLERANCE )
    {
        return KF_ITERATION_LIMIT;
    }

    return KF_OK;
}

static kf_status_t kf_character_support( kf_world_t * _world, kf_character_state_t * _state, const kf_sweep_hit_t * _hit )
{
    const kf_object_slot_t * object = kf_internal_object_const( _world, _hit->object );
    const kf_shape_slot_t * shape = kf_character_find_shape( _world, _hit->object, _hit->shape_id );
    kf_status_t status;

    if( object == NULL || shape == NULL )
    {
        return KF_STALE_HANDLE;
    }

    _state->support_velocity = (kf_vec3_t){0, 0, 0};

    if( object->desc.type != KF_BODY_STATIC )
    {
        /* Surface velocity at the character: v + w x r around the center of mass. */
        kf_mass_t mass;
        kf_vec3_t center;
        kf_vec3_t angular = object->state.angular_velocity;
        int64_t lever[3];
        int64_t surface[3];
        uint32_t axis;

        status = kf_internal_body_mass( _world, object, &mass );

        if( status != KF_OK )
        {
            return status;
        }

        center = kf_internal_body_center( object, &mass );
        lever[0] = (int64_t)_state->position.x - center.x;
        lever[1] = (int64_t)_state->position.y - center.y;
        lever[2] = (int64_t)_state->position.z - center.z;
        surface[0] = object->state.velocity.x + ( (int64_t)angular.y * lever[2] - (int64_t)angular.z * lever[1] ) / KF_FIXED_SCALE;
        surface[1] = object->state.velocity.y + ( (int64_t)angular.z * lever[0] - (int64_t)angular.x * lever[2] ) / KF_FIXED_SCALE;
        surface[2] = object->state.velocity.z + ( (int64_t)angular.x * lever[1] - (int64_t)angular.y * lever[0] ) / KF_FIXED_SCALE;

        for( axis = 0; axis != 3u; ++axis )
        {
            if( surface[axis] < INT32_MIN || surface[axis] > INT32_MAX )
            {
                return KF_OVERFLOW;
            }
        }

        _state->support_velocity = (kf_vec3_t){(kf_fixed_t)surface[0], (kf_fixed_t)surface[1], (kf_fixed_t)surface[2]};
    }

    _state->grounded = KF_TRUE;
    _state->support = _hit->object;
    _state->support_shape_id = _hit->shape_id;
    _state->support_shape_signature = kf_character_shape_signature( &shape->desc.shape );
    _state->support_normal = _hit->normal;
    _state->support_pose = object->state.pose;
    _state->velocity.y = 0;
    _state->position_remainder[1] = 0;

    return KF_OK;
}

static kf_status_t kf_character_carry( kf_world_t * _world, kf_character_state_t * _state, const kf_shape_t * _capsule, const kf_character_settings_t * _settings, const kf_query_filter_t * _filter, kf_character_contacts_t * _contacts, kf_bool_t * _ceiling )
{
    const kf_object_slot_t * object;
    const kf_shape_slot_t * shape;
    kf_vec3_t local;
    kf_vec3_t target;
    kf_pose_t pose;
    kf_sweep_hit_t support;
    kf_vec3_t down = {0, -(_settings->skin + KF_GEOMETRY_TOLERANCE), 0};
    kf_bool_t blocked = KF_FALSE;
    kf_status_t status;

    if( _state->grounded == KF_FALSE || _state->support.id == 0 || _state->velocity.y > 0 )
    {
        return KF_OK;
    }

    object = kf_internal_object_const( _world, _state->support );
    shape = kf_character_find_shape( _world, _state->support, _state->support_shape_id );

    if( object == NULL || shape == NULL )
    {
        kf_character_clear_support( _state );
        return KF_OK;
    }

    if( kf_character_filter_body( object, _filter, (kf_handle_t){0, 0} ) == KF_FALSE || kf_character_shape_signature( &shape->desc.shape ) != _state->support_shape_signature )
    {
        kf_character_clear_support( _state );
        return KF_OK;
    }

    local = kf_vec3_sub( _state->position, _state->support_pose.position );
    local = kf_rotation_apply_inverse( _state->support_pose.rotation, local );
    target = kf_rotation_apply( object->state.pose.rotation, local );
    status = kf_internal_vec_add( target, object->state.pose.position, &target );

    if( status != KF_OK )
    {
        return status;
    }

    pose = (kf_pose_t){target, {0, 0, 0}};
    status = kf_character_clip_vertical_probe( _capsule, pose, &down.y );

    if( status != KF_OK )
    {
        return status;
    }

    if( down.y == 0 )
    {
        kf_contact_t contact;
        status = kf_shape_contact( _capsule, pose, &shape->desc.shape, object->state.pose, &contact );

        if( status == KF_OK )
        {
            memset( &support, 0, sizeof( support ) );
            support.normal = contact.normal;
            support.contact_point = contact.point_right;
        }
    }
    else
    {
        status = kf_shape_sweep( _capsule, pose, down, &shape->desc.shape, object->state.pose, &support );
    }

    if( status == KF_NO_HIT )
    {
        kf_character_clear_support( _state );
        return KF_OK;
    }

    if( status != KF_OK )
    {
        return status;
    }

    support.object = _state->support;
    support.shape_id = _state->support_shape_id;
    status = kf_character_ground_normal( _world, pose, &support );

    if( status != KF_OK )
    {
        return status;
    }

    if( support.normal.y < _settings->minimum_ground_normal_y )
    {
        kf_character_clear_support( _state );
        return KF_OK;
    }

    pose.position = _state->position;
    target = kf_vec3_sub( target, pose.position );
    status = kf_character_slide( _world, _capsule, &pose, target, &_state->velocity, _settings, _filter, _state->support, _contacts, &blocked, _ceiling );

    if( status != KF_OK )
    {
        return status;
    }

    _state->position = pose.position;

    return KF_OK;
}

static kf_status_t kf_character_try_step( kf_world_t * _world, const kf_shape_t * _capsule, kf_pose_t _start, kf_vec3_t _displacement, const kf_character_settings_t * _settings, const kf_query_filter_t * _filter, kf_pose_t * _pose, kf_vec3_t * _velocity, kf_character_contacts_t * _contacts, kf_bool_t * _stepped )
{
    kf_vec3_t up = {0, _settings->step_height, 0};
    kf_vec3_t horizontal = {_displacement.x, 0, _displacement.z};
    kf_vec3_t down = {0, -(_settings->step_height + _settings->snap_distance), 0};
    kf_sweep_hit_t hit;
    kf_pose_t candidate = _start;
    kf_vec3_t velocity = *_velocity;
    size_t oldCount = _contacts->count;
    kf_bool_t blocked = KF_FALSE;
    kf_bool_t ceiling = KF_FALSE;
    kf_status_t status;
    int64_t ordinaryProgress;
    int64_t stepProgress;

    if( _settings->step_height == 0 || (_displacement.x == 0 && _displacement.z == 0) )
    {
        return KF_OK;
    }

    status = kf_character_clip_vertical_probe( _capsule, candidate, &up.y );

    if( status != KF_OK )
    {
        return status;
    }

    if( up.y == 0 )
    {
        return KF_OK;
    }

    down.y = -(up.y + _settings->snap_distance);
    status = kf_world_sweep_first( _world, _capsule, candidate, up, _filter, &hit );

    if( status == KF_OK )
    {
        return KF_OK;
    }

    if( status != KF_NO_HIT )
    {
        return status;
    }

    status = kf_internal_vec_add( candidate.position, up, &candidate.position );

    if( status != KF_OK )
    {
        return status;
    }

    status = kf_character_slide( _world, _capsule, &candidate, horizontal, &velocity, _settings, _filter, (kf_handle_t){0, 0}, _contacts, &blocked, &ceiling );

    if( status != KF_OK )
    {
        _contacts->count = oldCount;
        return status;
    }

    ordinaryProgress = ((int64_t)_pose->position.x - _start.position.x) * _displacement.x + ((int64_t)_pose->position.z - _start.position.z) * _displacement.z;
    stepProgress = ((int64_t)candidate.position.x - _start.position.x) * _displacement.x + ((int64_t)candidate.position.z - _start.position.z) * _displacement.z;

    if( stepProgress <= ordinaryProgress )
    {
        _contacts->count = oldCount;
        return KF_OK;
    }

    if( _displacement.y < 0 )
    {
        status = kf_checked_add( down.y, _displacement.y, &down.y );

        if( status != KF_OK )
        {
            _contacts->count = oldCount;
            return status;
        }
    }

    status = kf_character_clip_vertical_probe( _capsule, candidate, &down.y );

    if( status != KF_OK )
    {
        _contacts->count = oldCount;
        return status;
    }

    if( down.y == 0 )
    {
        status = kf_character_touching_support( _world, _capsule, candidate, _filter, _settings->minimum_ground_normal_y, &hit );
    }
    else
    {
        status = kf_world_sweep_first( _world, _capsule, candidate, down, _filter, &hit );
    }

    if( status == KF_NO_HIT )
    {
        _contacts->count = oldCount;
        return KF_OK;
    }

    if( status != KF_OK )
    {
        _contacts->count = oldCount;
        return status;
    }

    if( (int64_t)hit.contact_point.y - _start.position.y > _settings->step_height )
    {
        _contacts->count = oldCount;
        return KF_OK;
    }

    status = kf_character_ground_normal( _world, candidate, &hit );

    if( status != KF_OK )
    {
        _contacts->count = oldCount;
        return status;
    }

    if( hit.normal.y < _settings->minimum_ground_normal_y )
    {
        _contacts->count = oldCount;
        return KF_OK;
    }

    candidate.position = hit.position;
    kf_character_append_hit( _contacts, &hit );
    *_pose = candidate;
    *_velocity = velocity;
    *_stepped = KF_TRUE;

    return KF_OK;
}

/* Attaches the character to walkable ground found within a downward distance. */
static kf_status_t kf_character_snap( kf_world_t * _world, kf_character_state_t * _state, const kf_shape_t * _capsule, kf_fixed_t _distance, const kf_character_settings_t * _settings, const kf_query_filter_t * _filter, kf_character_contacts_t * _contacts )
{
    kf_pose_t pose = {_state->position, {0, 0, 0}};
    kf_sweep_hit_t hit;
    kf_fixed_t distance = -_distance;
    kf_status_t status = kf_character_clip_vertical_probe( _capsule, pose, &distance );

    if( status != KF_OK )
    {
        return status;
    }

    if( distance == 0 )
    {
        status = kf_character_touching_support( _world, _capsule, pose, _filter, _settings->minimum_ground_normal_y, &hit );
    }
    else
    {
        status = kf_world_sweep_first( _world, _capsule, pose, (kf_vec3_t){0, distance, 0}, _filter, &hit );
    }

    if( status == KF_OK )
    {
        status = kf_character_ground_normal( _world, pose, &hit );
    }

    if( status == KF_NO_HIT || (status == KF_OK && hit.normal.y < _settings->minimum_ground_normal_y) )
    {
        return KF_OK;
    }

    if( status != KF_OK )
    {
        return status;
    }

    _state->position = hit.position;
    status = kf_character_support( _world, _state, &hit );

    if( status != KF_OK )
    {
        return status;
    }

    kf_character_append_hit( _contacts, &hit );

    return KF_OK;
}

/* A grounded walk that would leave no walkable floor within the ledge drop under
   the character's center ends where the move started, as if blocked by a wall.
   The center ray ignores the rim the rounded capsule still hangs on. */
static kf_status_t kf_character_hold_ledge( kf_world_t * _world, kf_character_state_t * _state, kf_pose_t _start, const kf_character_settings_t * _settings, const kf_query_filter_t * _filter, kf_bool_t * _held )
{
    kf_ray_t ray;
    kf_sweep_hit_t hit;
    kf_status_t status;

    ray.origin = (kf_vec3_t){_state->position.x, _state->position.y + _settings->radius, _state->position.z};
    ray.direction = (kf_vec3_t){0, -KF_FIXED_SCALE, 0};
    /* The ray stops at the lower world boundary. */
    ray.maximum_distance = kf_fixed_min( _settings->radius + _settings->ledge_drop, ray.origin.y + KF_COORDINATE_LIMIT );
    status = kf_world_raycast_first( _world, &ray, _filter, &hit );

    if( status == KF_OK && hit.normal.y >= _settings->minimum_ground_normal_y )
    {
        return KF_OK;
    }

    if( status != KF_OK && status != KF_NO_HIT )
    {
        return status;
    }

    _state->position = _start.position;
    _state->velocity.x = 0;
    _state->velocity.z = 0;
    *_held = KF_TRUE;

    return KF_OK;
}

kf_character_settings_t kf_character_settings_default( void )
{
    kf_character_settings_t settings;
    memset( &settings, 0, sizeof( settings ) );
    settings.radius = KF_FIXED_SCALE / 3;
    settings.height = KF_FIXED_SCALE * 2;
    settings.step_height = KF_FIXED_SCALE / 3;
    settings.snap_distance = KF_FIXED_SCALE / 5;
    settings.minimum_ground_normal_y = 46341; /* cos(45 degrees) */
    settings.skin = 64;
    settings.gravity = KF_FIXED_SCALE * 20;
    settings.slide_iterations = 8;
    settings.depenetration_iterations = 8;
    settings.maximum_depenetration = KF_FIXED_SCALE * 2;

    return settings;
}

kf_status_t kf_character_move( kf_world_t * _world, kf_character_state_t * _state, const kf_character_settings_t * _settings, const kf_query_filter_t * _filter, kf_contact_t * _contacts, size_t _capacity, kf_character_move_result_t * _result )
{
    kf_character_state_t state;
    kf_character_move_result_t result;
    kf_character_contacts_t contacts = {_contacts, _capacity, 0};
    kf_query_filter_t filter;
    kf_shape_t capsule;
    kf_pose_t pose;
    kf_pose_t start;
    kf_vec3_t displacement;
    kf_vec3_t requestedVelocity;
    kf_vec3_t correction;
    kf_fixed_t gravityDelta;
    kf_bool_t wasGrounded;
    kf_bool_t standing;
    kf_bool_t sideBlocked = KF_FALSE;
    size_t initialCount = 0;
    kf_status_t status;

    if( _world == NULL || _state == NULL || _settings == NULL || _result == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }

    if( _capacity != 0 && _contacts == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }

    memset( &result, 0, sizeof( result ) );
    *_result = result;
    status = kf_character_validate( _state, _settings );

    if( status != KF_OK )
    {
        return status;
    }

    filter = _filter != NULL ? *_filter : kf_query_filter_default();
    filter.include_triggers = KF_FALSE;

    if( filter.exclude_count != 0 && filter.exclude == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }

    state = *_state;
    memset( &capsule, 0, sizeof( capsule ) );
    capsule.type = KF_SHAPE_CAPSULE;
    capsule.data.capsule.radius = _settings->radius;
    capsule.data.capsule.height = _settings->height;
    status = kf_character_carry( _world, &state, &capsule, _settings, &filter, &contacts, &result.hit_ceiling );

    if( status != KF_OK )
    {
        return status;
    }

    wasGrounded = state.grounded;
    pose = (kf_pose_t){state.position, {0, 0, 0}};
    {
        size_t available = contacts.count < contacts.capacity ? contacts.capacity - contacts.count : 0;
        kf_contact_t * output = available != 0 ? contacts.output + contacts.count : NULL;
        status = kf_world_contacts( _world, &capsule, pose, &filter, output, available, &initialCount );
    }

    if( status != KF_OK && status != KF_NO_HIT && status != KF_CAPACITY_EXCEEDED )
    {
        return status;
    }

    if( initialCount > SIZE_MAX - contacts.count )
    {
        return KF_OVERFLOW;
    }

    contacts.count += initialCount;
    status = kf_world_depenetrate( _world, &capsule, &pose, &filter, _settings->depenetration_iterations, _settings->maximum_depenetration, &correction );

    if( status != KF_OK && status != KF_NO_HIT )
    {
        return status;
    }

    /* A supported character does not accelerate into its ground: gravity along a
       walkable slope would otherwise slide it downhill. The final snap keeps it
       attached, and leaving the support resumes gravity on the next move. */
    standing = (kf_bool_t)( state.grounded != KF_FALSE && state.support.id != 0 && state.velocity.y <= 0 );

    if( standing != KF_FALSE )
    {
        state.velocity.y = 0;
    }

    kf_character_clear_support( &state );

    if( standing == KF_FALSE )
    {
        status = kf_integrate_scalar( -_settings->gravity, _world->config.step, &state.gravity_remainder, &gravityDelta );

        if( status != KF_OK )
        {
            return status;
        }

        status = kf_checked_add( state.velocity.y, gravityDelta, &state.velocity.y );

        if( status != KF_OK )
        {
            return status;
        }
    }

    if( state.velocity.y < -KF_SPEED_LIMIT || state.velocity.y > KF_SPEED_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }

    status = kf_integrate_scalar( state.velocity.x, _world->config.step, &state.position_remainder[0], &displacement.x );

    if( status != KF_OK )
    {
        return status;
    }

    status = kf_integrate_scalar( state.velocity.y, _world->config.step, &state.position_remainder[1], &displacement.y );

    if( status != KF_OK )
    {
        return status;
    }

    status = kf_integrate_scalar( state.velocity.z, _world->config.step, &state.position_remainder[2], &displacement.z );

    if( status != KF_OK )
    {
        return status;
    }

    if( displacement.x < -KF_COORDINATE_LIMIT * 2 || displacement.x > KF_COORDINATE_LIMIT * 2 || displacement.y < -KF_COORDINATE_LIMIT * 2 || displacement.y > KF_COORDINATE_LIMIT * 2 || displacement.z < -KF_COORDINATE_LIMIT * 2 || displacement.z > KF_COORDINATE_LIMIT * 2 )
    {
        return KF_OUT_OF_RANGE;
    }

    if( _settings->constrain_to_region != KF_FALSE )
    {
        kf_vec3_t target;
        status = kf_internal_vec_add( pose.position, displacement, &target );

        if( status != KF_OK )
        {
            return status;
        }

        target.x = kf_fixed_clamp( target.x, _settings->region.minimum.x + _settings->radius, _settings->region.maximum.x - _settings->radius );
        target.y = kf_fixed_clamp( target.y, _settings->region.minimum.y, _settings->region.maximum.y - _settings->height );
        target.z = kf_fixed_clamp( target.z, _settings->region.minimum.z + _settings->radius, _settings->region.maximum.z - _settings->radius );
        displacement = kf_vec3_sub( target, pose.position );
    }

    start = pose;
    requestedVelocity = state.velocity;
    status = kf_character_slide( _world, &capsule, &pose, displacement, &state.velocity, _settings, &filter, (kf_handle_t){0, 0}, &contacts, &sideBlocked, &result.hit_ceiling );

    if( status != KF_OK )
    {
        return status;
    }

    if( wasGrounded != KF_FALSE && sideBlocked != KF_FALSE && displacement.y <= 0 )
    {
        status = kf_character_try_step( _world, &capsule, start, displacement, _settings, &filter, &pose, &requestedVelocity, &contacts, &result.stepped );

        if( status != KF_OK )
        {
            return status;
        }

        if( result.stepped != KF_FALSE )
        {
            state.velocity = requestedVelocity;
        }
    }

    state.position = pose.position;

    if( requestedVelocity.y <= 0 )
    {
        kf_fixed_t distance = _settings->skin + KF_GEOMETRY_TOLERANCE;

        if( wasGrounded != KF_FALSE || result.stepped != KF_FALSE )
        {
            distance = kf_fixed_max( distance, _settings->snap_distance );
        }

        if( _settings->ledge_drop != 0 && wasGrounded != KF_FALSE )
        {
            status = kf_character_hold_ledge( _world, &state, start, _settings, &filter, &result.held_at_ledge );

            if( status != KF_OK )
            {
                return status;
            }

            if( result.held_at_ledge != KF_FALSE )
            {
                result.stepped = KF_FALSE;
            }
        }

        status = kf_character_snap( _world, &state, &capsule, distance, _settings, &filter, &contacts );

        if( status != KF_OK )
        {
            return status;
        }
    }

    pose.position = state.position;
    {
        kf_aabb_t bounds;
        status = kf_shape_bounds( &capsule, pose, &bounds );

        if( status != KF_OK )
        {
            return status;
        }
    }

    result.displacement = kf_vec3_sub( state.position, _state->position );
    result.landed = wasGrounded == KF_FALSE && state.grounded != KF_FALSE ? KF_TRUE : KF_FALSE;

    if( result.landed != KF_FALSE )
    {
        result.landing_speed = -requestedVelocity.y;
    }

    result.contact_count = contacts.count;
    *_result = result;

    if( contacts.count > _capacity )
    {
        return KF_CAPACITY_EXCEEDED;
    }

    *_state = state;

    return KF_OK;
}
