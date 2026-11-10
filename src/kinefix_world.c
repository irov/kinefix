#include "kinefix_internal.h"

#include <stdlib.h>
#include <string.h>

static void * kf_default_allocate( void * context, size_t bytes )
{
    (void)context;
    return malloc( bytes );
}

static void kf_default_free( void * context, void * allocation )
{
    (void)context;
    free( allocation );
}

kf_status_t kf_checked_add( kf_fixed_t left, kf_fixed_t right, kf_fixed_t * result )
{
    int64_t wide = (int64_t)left + right;
    if( result == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( wide < INT32_MIN )
    {
        return KF_OVERFLOW;
    }
    if( wide > INT32_MAX )
    {
        return KF_OVERFLOW;
    }
    *result = (kf_fixed_t)wide;
    return KF_OK;
}

kf_status_t kf_checked_mul( kf_fixed_t left, kf_fixed_t right, kf_fixed_t * result )
{
    int64_t wide = (int64_t)left * right / KF_FIXED_SCALE;
    if( result == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( wide < INT32_MIN )
    {
        return KF_OVERFLOW;
    }
    if( wide > INT32_MAX )
    {
        return KF_OVERFLOW;
    }
    *result = (kf_fixed_t)wide;
    return KF_OK;
}

kf_status_t kf_integrate_scalar( kf_fixed_t value, kf_timestep_t step, int64_t * remainder, kf_fixed_t * delta )
{
    int64_t numerator;
    int64_t quotient;
    if( remainder == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( delta == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( step.numerator == 0 )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( step.denominator == 0 )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( step.numerator > step.denominator )
    {
        return KF_OUT_OF_RANGE;
    }
    if( step.denominator > 1000000u )
    {
        return KF_OUT_OF_RANGE;
    }
    if( *remainder <= -(int64_t)step.denominator )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( *remainder >= (int64_t)step.denominator )
    {
        return KF_INVALID_ARGUMENT;
    }
    numerator = (int64_t)value * step.numerator + *remainder;
    quotient = numerator / step.denominator;
    if( quotient < INT32_MIN )
    {
        return KF_OVERFLOW;
    }
    if( quotient > INT32_MAX )
    {
        return KF_OVERFLOW;
    }
    *delta = (kf_fixed_t)quotient;
    *remainder = numerator % step.denominator;
    return KF_OK;
}

kf_status_t kf_internal_vec_add( kf_vec3_t left, kf_vec3_t right, kf_vec3_t * result )
{
    kf_vec3_t value;
    kf_status_t status = kf_checked_add( left.x, right.x, &value.x );
    if( status != KF_OK )
    {
        return status;
    }
    status = kf_checked_add( left.y, right.y, &value.y );
    if( status != KF_OK )
    {
        return status;
    }
    status = kf_checked_add( left.z, right.z, &value.z );
    if( status != KF_OK )
    {
        return status;
    }
    *result = value;
    return KF_OK;
}

static kf_bool_t kf_vector_in_range( kf_vec3_t value, kf_fixed_t limit )
{
    if( value.x < -limit || value.x > limit )
    {
        return KF_FALSE;
    }
    if( value.y < -limit || value.y > limit )
    {
        return KF_FALSE;
    }
    if( value.z < -limit || value.z > limit )
    {
        return KF_FALSE;
    }
    return KF_TRUE;
}

kf_bool_t kf_internal_bounds_overlap( kf_aabb_t left, kf_aabb_t right )
{
    if( left.minimum.x > right.maximum.x || left.maximum.x < right.minimum.x )
    {
        return KF_FALSE;
    }
    if( left.minimum.y > right.maximum.y || left.maximum.y < right.minimum.y )
    {
        return KF_FALSE;
    }
    if( left.minimum.z > right.maximum.z || left.maximum.z < right.minimum.z )
    {
        return KF_FALSE;
    }
    return KF_TRUE;
}

kf_status_t kf_internal_validate_config( const kf_world_config_t * config )
{
    if( config == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( config->step.numerator == 0 || config->step.denominator == 0 )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( config->step.numerator > config->step.denominator || config->step.denominator > 1000000u )
    {
        return KF_OUT_OF_RANGE;
    }
    if( kf_vector_in_range( config->gravity, KF_SPEED_LIMIT ) == KF_FALSE )
    {
        return KF_OUT_OF_RANGE;
    }
    if( config->solver_iterations == 0 || config->solver_iterations > 64u )
    {
        return KF_OUT_OF_RANGE;
    }
    if( config->ccd_iterations == 0 || config->ccd_iterations > 64u )
    {
        return KF_OUT_OF_RANGE;
    }
    if( config->sleep_speed < 0 || config->sleep_speed > KF_SPEED_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }
    if( config->contact_slop < 0 || config->contact_slop > KF_FIXED_SCALE )
    {
        return KF_OUT_OF_RANGE;
    }
    if( config->substeps == 0 || config->substeps > 16u )
    {
        return KF_OUT_OF_RANGE;
    }
    if( config->restitution_threshold < 0 || config->restitution_threshold > KF_SPEED_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }
    if( config->contact_hertz <= 0 || config->contact_hertz > KF_SPEED_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }
    if( config->contact_damping_ratio < 0 || config->contact_damping_ratio > KF_SPEED_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }
    if( config->contact_push_speed < 0 || config->contact_push_speed > KF_SPEED_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }
    return KF_OK;
}

kf_status_t kf_internal_validate_body( const kf_body_desc_t * desc )
{
    if( desc == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( desc->id == 0 )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( desc->type < KF_BODY_STATIC || desc->type > KF_BODY_DYNAMIC )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( kf_vector_in_range( desc->pose.position, KF_COORDINATE_LIMIT ) == KF_FALSE )
    {
        return KF_OUT_OF_RANGE;
    }
    if( kf_rotation_valid( desc->pose.rotation ) == KF_FALSE )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( kf_vector_in_range( desc->velocity, KF_SPEED_LIMIT ) == KF_FALSE )
    {
        return KF_OUT_OF_RANGE;
    }
    if( kf_vector_in_range( desc->angular_velocity, KF_SPEED_LIMIT ) == KF_FALSE )
    {
        return KF_OUT_OF_RANGE;
    }
    if( desc->mass < 0 || desc->mass > KF_SIZE_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }
    if( desc->type == KF_BODY_DYNAMIC && desc->mass < 128 )
    {
        return KF_OUT_OF_RANGE;
    }
    if( desc->friction < 0 || desc->friction > KF_FIXED_SCALE )
    {
        return KF_OUT_OF_RANGE;
    }
    if( desc->restitution < 0 || desc->restitution > KF_FIXED_SCALE )
    {
        return KF_OUT_OF_RANGE;
    }
    if( desc->linear_damping < 0 || desc->linear_damping > KF_SPEED_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }
    if( desc->angular_damping < 0 || desc->angular_damping > KF_SPEED_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }
    if( desc->gravity_scale < -16 * KF_FIXED_SCALE || desc->gravity_scale > 16 * KF_FIXED_SCALE )
    {
        return KF_OUT_OF_RANGE;
    }
    if( desc->rotation_locks > (KF_LOCK_ROTATION_X | KF_LOCK_ROTATION_Y | KF_LOCK_ROTATION_Z) )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( desc->trigger > KF_TRUE )
    {
        return KF_INVALID_ARGUMENT;
    }
    return KF_OK;
}

kf_world_config_t kf_world_config_default( void )
{
    kf_world_config_t config;
    memset( &config, 0, sizeof(config) );
    config.step = (kf_timestep_t){1u, 25u};
    config.gravity = (kf_vec3_t){0, -20 * KF_FIXED_SCALE, 0};
    config.solver_iterations = 2u;
    config.ccd_iterations = 8u;
    config.sleep_ticks = 25u;
    config.sleep_speed = KF_FIXED_SCALE / 20;
    config.contact_slop = KF_FIXED_SCALE / 200;
    config.substeps = 8u;
    config.restitution_threshold = KF_FIXED_SCALE;
    config.contact_hertz = 30 * KF_FIXED_SCALE;
    config.contact_damping_ratio = 10 * KF_FIXED_SCALE;
    config.contact_push_speed = 3 * KF_FIXED_SCALE;
    return config;
}

kf_body_desc_t kf_body_desc_default( uint32_t id )
{
    kf_body_desc_t desc = {0};
    desc.id = id;
    desc.mass = KF_FIXED_SCALE;
    desc.friction = KF_FIXED_SCALE / 2;
    desc.gravity_scale = KF_FIXED_SCALE;
    desc.category = UINT32_MAX;
    desc.mask = UINT32_MAX;
    return desc;
}

kf_query_filter_t kf_query_filter_default( void )
{
    kf_query_filter_t filter = {UINT32_MAX, UINT32_MAX, NULL, 0u, KF_FALSE};
    return filter;
}

kf_status_t kf_world_create( const kf_world_config_t * config, const kf_allocator_t * allocator, kf_world_t ** world )
{
    kf_allocator_t selected = {kf_default_allocate, kf_default_free, NULL};
    kf_world_config_t settings = kf_world_config_default();
    kf_world_t * created;
    kf_status_t status;
    if( world == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( config != NULL )
    {
        settings = *config;
    }
    status = kf_internal_validate_config( &settings );
    if( status != KF_OK )
    {
        return status;
    }
    if( allocator != NULL )
    {
        if( allocator->allocate == NULL )
        {
            return KF_INVALID_ARGUMENT;
        }
        if( allocator->free == NULL )
        {
            return KF_INVALID_ARGUMENT;
        }
        selected = *allocator;
    }
    created = selected.allocate( selected.context, sizeof(*created) );
    if( created == NULL )
    {
        return KF_OUT_OF_MEMORY;
    }
    memset( created, 0, sizeof(*created) );
    created->allocator = selected;
    created->config = settings;
    kf_internal_invalidate( created );
    *world = created;
    return KF_OK;
}

static void kf_world_free_arrays( kf_world_t * world )
{
    void * allocations[18] = {world->objects, world->shapes, world->constraints, world->leaves, world->nodes,
        world->shape_objects, world->scratch_bounds, world->candidates, world->bodies, world->paths, world->manifolds, world->joints,
        world->cache, world->next_cache, world->shape_bounds, world->masses, world->triggers, world->previous_triggers};
    size_t index;
    for( index = 0; index != 18u; ++index )
    {
        if( allocations[index] != NULL )
        {
            world->allocator.free( world->allocator.context, allocations[index] );
        }
    }
}

void kf_world_destroy( kf_world_t * world )
{
    kf_allocator_t allocator;
    if( world == NULL )
    {
        return;
    }
    allocator = world->allocator;
    kf_world_free_arrays( world );
    allocator.free( allocator.context, world );
}

static void * kf_world_allocate_array( kf_world_t * world, size_t count, size_t size )
{
    void * allocation;
    if( count == 0 )
    {
        return NULL;
    }
    allocation = world->allocator.allocate( world->allocator.context, count * size );
    if( allocation != NULL )
    {
        memset( allocation, 0, count * size );
    }
    return allocation;
}

kf_status_t kf_world_reserve( kf_world_t * world, kf_capacity_t capacity )
{
    kf_world_t replacement;
    if( world == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( capacity.objects < world->capacity.objects )
    {
        capacity.objects = world->capacity.objects;
    }
    if( capacity.shapes < world->capacity.shapes )
    {
        capacity.shapes = world->capacity.shapes;
    }
    if( capacity.constraints < world->capacity.constraints )
    {
        capacity.constraints = world->capacity.constraints;
    }
    if( capacity.shapes > SIZE_MAX / 4u )
    {
        return KF_OVERFLOW;
    }
    if( capacity.contacts == 0 )
    {
        capacity.contacts = capacity.shapes * 4u;
    }
    if( capacity.contacts < world->capacity.contacts )
    {
        capacity.contacts = world->capacity.contacts;
    }
    if( capacity.triggers == 0 )
    {
        capacity.triggers = capacity.shapes;
    }
    if( capacity.triggers < world->capacity.triggers )
    {
        capacity.triggers = world->capacity.triggers;
    }
    if( capacity.objects == world->capacity.objects && capacity.shapes == world->capacity.shapes && capacity.constraints == world->capacity.constraints && capacity.contacts == world->capacity.contacts && capacity.triggers == world->capacity.triggers )
    {
        return KF_OK;
    }
    if( capacity.objects > SIZE_MAX / sizeof(kf_object_slot_t) || capacity.objects > SIZE_MAX / kf_internal_solver_body_size() || capacity.objects > SIZE_MAX / kf_internal_solver_path_size() || capacity.objects > SIZE_MAX / sizeof(kf_mass_t) )
    {
        return KF_OVERFLOW;
    }
    if( capacity.shapes > SIZE_MAX / (2u * sizeof(kf_index_node_t)) )
    {
        return KF_OVERFLOW;
    }
    if( capacity.shapes > SIZE_MAX / sizeof(kf_shape_slot_t) )
    {
        return KF_OVERFLOW;
    }
    if( capacity.constraints > SIZE_MAX / sizeof(kf_constraint_slot_t) || capacity.constraints > SIZE_MAX / kf_internal_solver_joint_size() )
    {
        return KF_OVERFLOW;
    }
    if( capacity.contacts > SIZE_MAX / kf_internal_solver_manifold_size() || capacity.contacts > SIZE_MAX / sizeof(kf_contact_snapshot_t) )
    {
        return KF_OVERFLOW;
    }
    if( capacity.triggers > SIZE_MAX / sizeof(kf_trigger_overlap_t) )
    {
        return KF_OVERFLOW;
    }
    memset( &replacement, 0, sizeof(replacement) );
    replacement.allocator = world->allocator;
    replacement.objects = kf_world_allocate_array( world, capacity.objects, sizeof(*world->objects) );
    replacement.shapes = kf_world_allocate_array( world, capacity.shapes, sizeof(*world->shapes) );
    replacement.constraints = kf_world_allocate_array( world, capacity.constraints, sizeof(*world->constraints) );
    replacement.leaves = kf_world_allocate_array( world, capacity.shapes, sizeof(*world->leaves) );
    replacement.nodes = kf_world_allocate_array( world, capacity.shapes * 2u, sizeof(*world->nodes) );
    replacement.shape_objects = kf_world_allocate_array( world, capacity.shapes, sizeof(*world->shape_objects) );
    replacement.scratch_bounds = kf_world_allocate_array( world, capacity.shapes, sizeof(*world->scratch_bounds) );
    replacement.candidates = kf_world_allocate_array( world, capacity.shapes, sizeof(*world->candidates) );
    replacement.bodies = kf_world_allocate_array( world, capacity.objects, kf_internal_solver_body_size() );
    replacement.paths = kf_world_allocate_array( world, capacity.objects, kf_internal_solver_path_size() );
    replacement.manifolds = kf_world_allocate_array( world, capacity.contacts, kf_internal_solver_manifold_size() );
    replacement.joints = kf_world_allocate_array( world, capacity.constraints, kf_internal_solver_joint_size() );
    replacement.cache = kf_world_allocate_array( world, capacity.contacts, sizeof(*world->cache) );
    replacement.next_cache = kf_world_allocate_array( world, capacity.contacts, sizeof(*world->next_cache) );
    replacement.shape_bounds = kf_world_allocate_array( world, capacity.shapes, sizeof(*world->shape_bounds) );
    replacement.masses = kf_world_allocate_array( world, capacity.objects, sizeof(*world->masses) );
    replacement.triggers = kf_world_allocate_array( world, capacity.triggers, sizeof(*world->triggers) );
    replacement.previous_triggers = kf_world_allocate_array( world, capacity.triggers, sizeof(*world->previous_triggers) );
    if( (capacity.objects != 0 && (replacement.objects == NULL || replacement.bodies == NULL || replacement.paths == NULL || replacement.masses == NULL)) ||
        (capacity.shapes != 0 && (replacement.shapes == NULL || replacement.leaves == NULL || replacement.nodes == NULL || replacement.shape_objects == NULL || replacement.scratch_bounds == NULL || replacement.candidates == NULL || replacement.shape_bounds == NULL)) ||
        (capacity.constraints != 0 && (replacement.constraints == NULL || replacement.joints == NULL)) ||
        (capacity.contacts != 0 && (replacement.manifolds == NULL || replacement.cache == NULL || replacement.next_cache == NULL)) ||
        (capacity.triggers != 0 && (replacement.triggers == NULL || replacement.previous_triggers == NULL)) )
    {
        kf_world_free_arrays( &replacement );
        return KF_OUT_OF_MEMORY;
    }
    if( world->object_count != 0 )
    {
        memcpy( replacement.objects, world->objects, world->object_count * sizeof(*world->objects) );
    }
    if( world->shape_count != 0 )
    {
        memcpy( replacement.shapes, world->shapes, world->shape_count * sizeof(*world->shapes) );
    }
    if( world->constraint_count != 0 )
    {
        memcpy( replacement.constraints, world->constraints, world->constraint_count * sizeof(*world->constraints) );
    }
    if( world->cache_count != 0 )
    {
        memcpy( replacement.cache, world->cache, world->cache_count * sizeof(*world->cache) );
    }
    if( world->trigger_count != 0 )
    {
        memcpy( replacement.triggers, world->triggers, world->trigger_count * sizeof(*world->triggers) );
    }
    if( world->previous_trigger_count != 0 )
    {
        memcpy( replacement.previous_triggers, world->previous_triggers, world->previous_trigger_count * sizeof(*world->previous_triggers) );
    }
    kf_world_free_arrays( world );
    world->objects = replacement.objects;
    world->shapes = replacement.shapes;
    world->constraints = replacement.constraints;
    world->leaves = replacement.leaves;
    world->nodes = replacement.nodes;
    world->shape_objects = replacement.shape_objects;
    world->scratch_bounds = replacement.scratch_bounds;
    world->candidates = replacement.candidates;
    world->bodies = replacement.bodies;
    world->paths = replacement.paths;
    world->manifolds = replacement.manifolds;
    world->joints = replacement.joints;
    world->cache = replacement.cache;
    world->next_cache = replacement.next_cache;
    world->shape_bounds = replacement.shape_bounds;
    world->masses = replacement.masses;
    world->triggers = replacement.triggers;
    world->previous_triggers = replacement.previous_triggers;
    world->capacity = capacity;
    kf_internal_invalidate( world );
    return KF_OK;
}

kf_object_slot_t * kf_internal_object_id( kf_world_t * world, uint32_t id )
{
    size_t low = 0;
    size_t high = world->object_count;
    while( low < high )
    {
        size_t middle = low + (high - low) / 2u;
        if( world->objects[middle].desc.id == id )
        {
            return world->objects + middle;
        }
        if( world->objects[middle].desc.id < id )
        {
            low = middle + 1u;
        }
        else
        {
            high = middle;
        }
    }
    return NULL;
}

const kf_object_slot_t * kf_internal_object_const( const kf_world_t * world, kf_handle_t handle )
{
    size_t low = 0;
    size_t high;
    if( world == NULL )
    {
        return NULL;
    }
    high = world->object_count;
    while( low < high )
    {
        size_t middle = low + (high - low) / 2u;
        const kf_object_slot_t * object = world->objects + middle;
        if( object->desc.id < handle.id )
        {
            low = middle + 1u;
            continue;
        }
        if( object->desc.id > handle.id )
        {
            high = middle;
            continue;
        }
        if( object->active == KF_FALSE )
        {
            return NULL;
        }
        if( object->state.handle.generation != handle.generation )
        {
            return NULL;
        }
        return object;
    }
    return NULL;
}

kf_object_slot_t * kf_internal_object( kf_world_t * world, kf_handle_t handle )
{
    kf_object_slot_t * object;
    if( world == NULL )
    {
        return NULL;
    }
    object = kf_internal_object_id( world, handle.id );
    if( object == NULL )
    {
        return NULL;
    }
    if( object->active == KF_FALSE )
    {
        return NULL;
    }
    if( object->state.handle.generation != handle.generation )
    {
        return NULL;
    }
    return object;
}

static kf_status_t kf_validate_shapes( const kf_body_desc_t * desc, const kf_shape_desc_t * shapes, size_t count )
{
    size_t index;
    if( count != 0 && shapes == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    for( index = 0; index != count; ++index )
    {
        size_t other;
        kf_aabb_t bounds;
        kf_status_t status;
        if( shapes[index].id == 0 )
        {
            return KF_INVALID_ARGUMENT;
        }
        if( desc->type != KF_BODY_STATIC && shapes[index].shape.type == KF_SHAPE_TRIANGLE )
        {
            return KF_UNSUPPORTED;
        }
        status = kf_shape_bounds( &shapes[index].shape, desc->pose, &bounds );
        if( status != KF_OK )
        {
            return status;
        }
        for( other = 0; other != index; ++other )
        {
            if( shapes[index].id == shapes[other].id )
            {
                return KF_DUPLICATE_ID;
            }
        }
    }
    return KF_OK;
}

/* Contact keys contain IDs rather than generations; remove them before a new
   body can reuse the application ID. */
static void kf_world_forget_contacts( kf_world_t * world, uint32_t id )
{
    size_t index;
    size_t output = 0;
    for( index = 0; index != world->cache_count; ++index )
    {
        const kf_contact_snapshot_t * contact = world->cache + index;
        if( contact->left_object == id || contact->right_object == id )
        {
            continue;
        }
        world->cache[output++] = *contact;
    }
    world->cache_count = output;
}

/* Recomputes one object's derived data after its descriptor, pose or shapes changed. */
static void kf_world_refresh_object( kf_world_t * world, const kf_object_slot_t * object )
{
    if( world->masses_dirty == KF_FALSE )
    {
        kf_mass_t * mass = world->masses + (object - world->objects);
        memset( mass, 0, sizeof(*mass) );
        if( object->active == KF_TRUE && object->desc.type != KF_BODY_STATIC && kf_internal_body_mass( world, object, mass ) != KF_OK )
        {
            world->masses_dirty = KF_TRUE;
        }
    }
    if( world->bounds_dirty == KF_FALSE )
    {
        (void)kf_internal_update_object_bounds( world, object );
    }
    world->index_dirty = KF_TRUE;
}

static kf_aabb_t kf_bounds_union( kf_aabb_t left, kf_aabb_t right );

void kf_internal_shape_range( const kf_world_t * world, uint32_t id, size_t * begin, size_t * end )
{
    size_t low = 0;
    size_t high = world->shape_count;
    while( low != high )
    {
        size_t middle = low + (high - low) / 2u;
        if( world->shapes[middle].object_id < id )
        {
            low = middle + 1u;
        }
        else
        {
            high = middle;
        }
    }
    *begin = low;
    while( low != world->shape_count && world->shapes[low].object_id == id )
    {
        ++low;
    }
    *end = low;
}

/* Union of an object's shape bounds at a pose; returns KF_FALSE without shapes. */
static kf_bool_t kf_world_object_bounds( const kf_world_t * world, uint32_t id, kf_pose_t pose, kf_aabb_t * bounds )
{
    kf_bool_t found = KF_FALSE;
    size_t begin;
    size_t end;
    kf_internal_shape_range( world, id, &begin, &end );
    for( ; begin != end; ++begin )
    {
        kf_aabb_t shape_bounds;
        if( kf_internal_shape_bounds( &world->shapes[begin].desc.shape, pose, &shape_bounds ) != KF_OK )
        {
            continue;
        }
        *bounds = found == KF_TRUE ? kf_bounds_union( *bounds, shape_bounds ) : shape_bounds;
        found = KF_TRUE;
    }
    return found;
}

/* Geometry edits wake only sleeping dynamic bodies with a shape within contact
   range of the edited region; the step propagates waking through their islands. */
static void kf_world_wake_region( kf_world_t * world, kf_aabb_t region )
{
    kf_fixed_t margin = world->config.contact_slop * 4 + KF_GEOMETRY_TOLERANCE;
    size_t index;
    region.minimum = (kf_vec3_t){region.minimum.x - margin, region.minimum.y - margin, region.minimum.z - margin};
    region.maximum = (kf_vec3_t){region.maximum.x + margin, region.maximum.y + margin, region.maximum.z + margin};
    for( index = 0; index != world->shape_count; ++index )
    {
        kf_object_slot_t * object = kf_internal_object_id( world, world->shapes[index].object_id );
        kf_aabb_t bounds;
        if( object == NULL || object->active == KF_FALSE || object->desc.type != KF_BODY_DYNAMIC || object->state.sleeping == KF_FALSE )
        {
            continue;
        }
        if( world->bounds_dirty == KF_FALSE )
        {
            bounds = world->shape_bounds[index];
        }
        else if( kf_internal_shape_bounds( &world->shapes[index].desc.shape, object->state.pose, &bounds ) != KF_OK )
        {
            continue;
        }
        if( kf_internal_bounds_overlap( bounds, region ) == KF_FALSE )
        {
            continue;
        }
        object->state.sleeping = KF_FALSE;
        object->state.sleep_counter = 0;
    }
}

static void kf_world_wake_object_region( kf_world_t * world, uint32_t id, kf_pose_t pose )
{
    kf_aabb_t bounds;
    if( kf_world_object_bounds( world, id, pose, &bounds ) == KF_TRUE )
    {
        kf_world_wake_region( world, bounds );
    }
}

kf_status_t kf_world_add_body( kf_world_t * world, const kf_body_desc_t * body, const kf_shape_desc_t * shapes, size_t count, kf_handle_t * handle )
{
    kf_status_t status;
    kf_object_slot_t * slot;
    uint32_t generation = 1u;
    size_t index;
    if( world == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( handle == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    status = kf_internal_validate_body( body );
    if( status != KF_OK )
    {
        return status;
    }
    if( count > world->capacity.shapes - world->shape_count )
    {
        return KF_CAPACITY_EXCEEDED;
    }
    slot = kf_internal_object_id( world, body->id );
    if( slot != NULL )
    {
        if( slot->active == KF_TRUE )
        {
            return KF_DUPLICATE_ID;
        }
        if( slot->state.handle.generation == UINT32_MAX )
        {
            return KF_OVERFLOW;
        }
        generation = slot->state.handle.generation + 1u;
    }
    else if( world->object_count == world->capacity.objects )
    {
        return KF_CAPACITY_EXCEEDED;
    }
    status = kf_validate_shapes( body, shapes, count );
    if( status != KF_OK )
    {
        return status;
    }
    if( slot == NULL )
    {
        size_t insert = world->object_count;
        while( insert != 0 && world->objects[insert - 1u].desc.id > body->id )
        {
            world->objects[insert] = world->objects[insert - 1u];
            world->masses[insert] = world->masses[insert - 1u];
            --insert;
        }
        slot = world->objects + insert;
        ++world->object_count;
    }
    memset( slot, 0, sizeof(*slot) );
    slot->active = KF_TRUE;
    slot->desc = *body;
    slot->state.handle = (kf_handle_t){body->id, generation};
    slot->state.pose = body->pose;
    slot->state.velocity = body->velocity;
    slot->state.angular_velocity = body->angular_velocity;
    slot->state.inverse_mass = body->type == KF_BODY_DYNAMIC ? kf_fixed_div( KF_FIXED_SCALE, body->mass ) : 0;
    /* The object's shapes form one block in canonical order; derived bounds
       shift with the shapes. */
    {
        size_t begin;
        size_t end;
        kf_internal_shape_range( world, body->id, &begin, &end );
        memmove( world->shapes + begin + count, world->shapes + begin, (world->shape_count - begin) * sizeof(*world->shapes) );
        memmove( world->shape_bounds + begin + count, world->shape_bounds + begin, (world->shape_count - begin) * sizeof(*world->shape_bounds) );
        world->shape_count += count;
        for( index = 0; index != count; ++index )
        {
            size_t insert = begin + index;
            while( insert != begin && world->shapes[insert - 1u].desc.id > shapes[index].id )
            {
                world->shapes[insert] = world->shapes[insert - 1u];
                --insert;
            }
            world->shapes[insert] = (kf_shape_slot_t){body->id, shapes[index]};
        }
    }
    world->reports_valid = KF_FALSE;
    kf_world_refresh_object( world, slot );
    kf_world_wake_object_region( world, body->id, body->pose );
    *handle = slot->state.handle;
    return KF_OK;
}

kf_status_t kf_world_remove_body( kf_world_t * world, kf_handle_t handle )
{
    kf_object_slot_t * object;
    size_t index;
    size_t output = 0;
    if( world == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    object = kf_internal_object( world, handle );
    if( object == NULL )
    {
        return KF_STALE_HANDLE;
    }
    kf_world_wake_object_region( world, handle.id, object->state.pose );
    kf_world_forget_contacts( world, handle.id );
    for( index = 0; index != world->constraint_count; ++index )
    {
        kf_constraint_slot_t * constraint = world->constraints + index;
        if( constraint->active == KF_FALSE )
        {
            continue;
        }
        if( constraint->state.desc.left.id != handle.id && constraint->state.desc.right.id != handle.id )
        {
            continue;
        }
        /* Wake the surviving endpoint without searching the constraint list
           again for every link attached to this body. */
        (void)kf_world_wake_body( world, constraint->state.desc.left.id == handle.id ? constraint->state.desc.right : constraint->state.desc.left );
        constraint->active = KF_FALSE;
    }
    object->active = KF_FALSE;
    for( index = 0; index != world->shape_count; ++index )
    {
        if( world->shapes[index].object_id == handle.id )
        {
            continue;
        }
        world->shape_bounds[output] = world->shape_bounds[index];
        world->shapes[output++] = world->shapes[index];
    }
    world->shape_count = output;
    world->reports_valid = KF_FALSE;
    kf_world_refresh_object( world, object );
    return KF_OK;
}

kf_status_t kf_world_get_body( const kf_world_t * world, kf_handle_t handle, kf_body_state_t * state )
{
    const kf_object_slot_t * object;
    if( world == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( state == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    object = kf_internal_object_const( world, handle );
    if( object == NULL )
    {
        return KF_STALE_HANDLE;
    }
    *state = object->state;
    return KF_OK;
}

static kf_status_t kf_validate_body_state( const kf_body_desc_t * desc, const kf_body_state_t * state, const kf_world_config_t * config )
{
    kf_body_desc_t copy = *desc;
    kf_fixed_t inverse = desc->type == KF_BODY_DYNAMIC ? kf_fixed_div( KF_FIXED_SCALE, desc->mass ) : 0;
    int64_t bound = (int64_t)config->step.denominator * config->substeps;
    size_t index;
    copy.pose = state->pose;
    copy.velocity = state->velocity;
    copy.angular_velocity = state->angular_velocity;
    if( state->handle.id != desc->id )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( state->handle.generation == 0 )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( state->inverse_mass != inverse )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( state->sleeping > KF_TRUE || state->has_target > KF_TRUE )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( state->has_target == KF_TRUE )
    {
        if( desc->type != KF_BODY_KINEMATIC )
        {
            return KF_INVALID_ARGUMENT;
        }
        if( kf_vector_in_range( state->target.position, KF_COORDINATE_LIMIT ) == KF_FALSE )
        {
            return KF_OUT_OF_RANGE;
        }
        if( kf_rotation_valid( state->target.rotation ) == KF_FALSE )
        {
            return KF_INVALID_ARGUMENT;
        }
    }
    for( index = 0; index != 3u; ++index )
    {
        if( state->position_remainder[index] <= -bound || state->position_remainder[index] >= bound )
        {
            return KF_INVALID_ARGUMENT;
        }
        if( state->velocity_remainder[index] <= -bound || state->velocity_remainder[index] >= bound )
        {
            return KF_INVALID_ARGUMENT;
        }
    }
    return kf_internal_validate_body( &copy );
}

static kf_status_t kf_validate_object_bounds( const kf_world_t * world, uint32_t id, kf_pose_t pose, kf_body_type_t type )
{
    size_t index;
    for( index = 0; index != world->shape_count; ++index )
    {
        kf_aabb_t bounds;
        kf_status_t status;
        if( world->shapes[index].object_id != id )
        {
            continue;
        }
        if( type != KF_BODY_STATIC && world->shapes[index].desc.shape.type == KF_SHAPE_TRIANGLE )
        {
            return KF_UNSUPPORTED;
        }
        status = kf_shape_bounds( &world->shapes[index].desc.shape, pose, &bounds );
        if( status != KF_OK )
        {
            return status;
        }
    }
    return KF_OK;
}

static void kf_world_refit_body( kf_world_t * world, const kf_object_slot_t * object );

kf_status_t kf_world_set_body( kf_world_t * world, const kf_body_state_t * state )
{
    kf_object_slot_t * object;
    kf_status_t status;
    kf_bool_t geometry_changed;
    if( world == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( state == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    object = kf_internal_object( world, state->handle );
    if( object == NULL )
    {
        return KF_STALE_HANDLE;
    }
    status = kf_validate_body_state( &object->desc, state, &world->config );
    if( status != KF_OK )
    {
        return status;
    }
    status = kf_validate_object_bounds( world, object->desc.id, state->pose, object->desc.type );
    if( status != KF_OK )
    {
        return status;
    }
    if( state->has_target == KF_TRUE )
    {
        kf_mass_t mass;
        status = kf_validate_object_bounds( world, object->desc.id, state->target, object->desc.type );
        if( status != KF_OK )
        {
            return status;
        }
        status = kf_internal_body_mass( world, object, &mass );
        if( status != KF_OK )
        {
            return status;
        }
        status = kf_internal_validate_kinematic_target( &world->config, state, &mass );
        if( status != KF_OK )
        {
            return status;
        }
    }
    geometry_changed = (kf_bool_t)(kf_vec3_equal( object->state.pose.position, state->pose.position ) == KF_FALSE ||
        object->state.pose.rotation.x != state->pose.rotation.x || object->state.pose.rotation.y != state->pose.rotation.y || object->state.pose.rotation.z != state->pose.rotation.z);
    if( geometry_changed == KF_TRUE )
    {
        kf_world_wake_object_region( world, object->desc.id, object->state.pose );
    }
    object->state = *state;
    if( geometry_changed == KF_TRUE )
    {
        kf_world_wake_object_region( world, object->desc.id, object->state.pose );
        object->state.sleeping = state->sleeping;
        object->state.sleep_counter = state->sleep_counter;
        kf_world_refit_body( world, object );
    }
    return KF_OK;
}

kf_status_t kf_world_update_body( kf_world_t * world, kf_handle_t handle, const kf_body_desc_t * body )
{
    kf_object_slot_t * object;
    kf_status_t status;
    if( world == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    status = kf_internal_validate_body( body );
    if( status != KF_OK )
    {
        return status;
    }
    object = kf_internal_object( world, handle );
    if( object == NULL )
    {
        return KF_STALE_HANDLE;
    }
    if( body->id != handle.id )
    {
        return KF_INVALID_ARGUMENT;
    }
    status = kf_validate_object_bounds( world, handle.id, body->pose, body->type );
    if( status != KF_OK )
    {
        return status;
    }
    kf_world_wake_object_region( world, handle.id, object->state.pose );
    kf_world_wake_object_region( world, handle.id, body->pose );
    object->desc = *body;
    memset( &object->state, 0, sizeof(object->state) );
    object->state.handle = handle;
    object->state.pose = body->pose;
    object->state.velocity = body->velocity;
    object->state.angular_velocity = body->angular_velocity;
    object->state.inverse_mass = body->type == KF_BODY_DYNAMIC ? kf_fixed_div( KF_FIXED_SCALE, body->mass ) : 0;
    kf_world_refresh_object( world, object );
    return KF_OK;
}

kf_status_t kf_world_modify_body( kf_world_t * world, kf_handle_t handle, const kf_body_desc_t * body )
{
    kf_object_slot_t * object;
    kf_status_t status;
    if( world == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    status = kf_internal_validate_body( body );
    if( status != KF_OK )
    {
        return status;
    }
    object = kf_internal_object( world, handle );
    if( object == NULL )
    {
        return KF_STALE_HANDLE;
    }
    if( body->id != handle.id )
    {
        return KF_INVALID_ARGUMENT;
    }
    status = kf_validate_object_bounds( world, handle.id, object->state.pose, body->type );
    if( status != KF_OK )
    {
        return status;
    }
    object->desc = *body;
    object->desc.pose = object->state.pose;
    object->state.inverse_mass = body->type == KF_BODY_DYNAMIC ? kf_fixed_div( KF_FIXED_SCALE, body->mass ) : 0;
    if( body->type != KF_BODY_KINEMATIC )
    {
        object->state.has_target = KF_FALSE;
    }
    if( body->type == KF_BODY_STATIC )
    {
        object->state.velocity = (kf_vec3_t){0, 0, 0};
        object->state.angular_velocity = (kf_vec3_t){0, 0, 0};
    }
    if( body->type == KF_BODY_DYNAMIC )
    {
        object->state.sleeping = KF_FALSE;
        object->state.sleep_counter = 0;
    }
    /* A body that stops colliding or holding others up wakes its neighbors. */
    kf_world_wake_object_region( world, handle.id, object->state.pose );
    kf_world_refresh_object( world, object );
    return KF_OK;
}

kf_status_t kf_world_update_shape( kf_world_t * world, kf_handle_t handle, const kf_shape_desc_t * shape )
{
    kf_object_slot_t * object;
    kf_body_desc_t desc;
    kf_status_t status;
    size_t index;
    if( world == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( shape == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    object = kf_internal_object( world, handle );
    if( object == NULL )
    {
        return KF_STALE_HANDLE;
    }
    desc = object->desc;
    desc.pose = object->state.pose;
    status = kf_validate_shapes( &desc, shape, 1u );
    if( status != KF_OK )
    {
        return status;
    }
    if( object->state.has_target == KF_TRUE )
    {
        desc.pose = object->state.target;
        status = kf_validate_shapes( &desc, shape, 1u );
        if( status != KF_OK )
        {
            return status;
        }
    }
    for( index = 0; index != world->shape_count; ++index )
    {
        kf_shape_slot_t * slot = world->shapes + index;
        if( slot->object_id != handle.id )
        {
            continue;
        }
        if( slot->desc.id != shape->id )
        {
            continue;
        }
        if( object->state.has_target == KF_TRUE )
        {
            kf_shape_desc_t previous = slot->desc;
            kf_mass_t mass;
            /* Mass calculation is read-only and has no callbacks. Restore the
               descriptor before reporting any validation error or waking. */
            slot->desc = *shape;
            status = kf_internal_body_mass( world, object, &mass );
            slot->desc = previous;
            if( status != KF_OK )
            {
                return status;
            }
            status = kf_internal_validate_kinematic_target( &world->config, &object->state, &mass );
            if( status != KF_OK )
            {
                return status;
            }
        }
        kf_world_wake_object_region( world, handle.id, object->state.pose );
        slot->desc = *shape;
        kf_world_wake_object_region( world, handle.id, object->state.pose );
        object->state.sleeping = KF_FALSE;
        object->state.sleep_counter = 0;
        kf_world_refresh_object( world, object );
        return KF_OK;
    }
    return KF_STALE_HANDLE;
}

static kf_aabb_t kf_bounds_union( kf_aabb_t left, kf_aabb_t right )
{
    kf_aabb_t bounds = {{kf_fixed_min( left.minimum.x, right.minimum.x ), kf_fixed_min( left.minimum.y, right.minimum.y ), kf_fixed_min( left.minimum.z, right.minimum.z )},
        {kf_fixed_max( left.maximum.x, right.maximum.x ), kf_fixed_max( left.maximum.y, right.maximum.y ), kf_fixed_max( left.maximum.z, right.maximum.z )}};
    return bounds;
}

void kf_internal_invalidate( kf_world_t * world )
{
    world->index_dirty = KF_TRUE;
    world->bounds_dirty = KF_TRUE;
    world->masses_dirty = KF_TRUE;
    world->reports_valid = KF_FALSE;
}

kf_status_t kf_internal_update_object_bounds( kf_world_t * world, const kf_object_slot_t * object )
{
    size_t begin;
    size_t end;
    kf_internal_shape_range( world, object->desc.id, &begin, &end );
    for( ; begin != end; ++begin )
    {
        kf_status_t status = kf_internal_shape_bounds( &world->shapes[begin].desc.shape, object->state.pose, world->shape_bounds + begin );
        if( status != KF_OK )
        {
            world->bounds_dirty = KF_TRUE;
            return status;
        }
    }
    return KF_OK;
}

kf_status_t kf_internal_refresh_bounds( kf_world_t * world )
{
    size_t index = 0;
    if( world->bounds_dirty == KF_FALSE )
    {
        return KF_OK;
    }
    while( index != world->shape_count )
    {
        const kf_object_slot_t * object = kf_internal_object_id( world, world->shapes[index].object_id );
        size_t begin;
        size_t end;
        kf_status_t status;
        if( object == NULL || object->active == KF_FALSE )
        {
            return KF_INVALID_ARGUMENT;
        }
        status = kf_internal_update_object_bounds( world, object );
        if( status != KF_OK )
        {
            return status;
        }
        kf_internal_shape_range( world, object->desc.id, &begin, &end );
        index = end;
    }
    world->bounds_dirty = KF_FALSE;
    return KF_OK;
}

/* Updates a moved body's cached bounds and refits the current tree around them. */
static void kf_world_refit_body( kf_world_t * world, const kf_object_slot_t * object )
{
    size_t begin;
    size_t end;
    size_t index;
    if( world->bounds_dirty == KF_TRUE || kf_internal_update_object_bounds( world, object ) != KF_OK )
    {
        world->index_dirty = KF_TRUE;
        return;
    }
    if( world->index_dirty == KF_TRUE )
    {
        return;
    }
    kf_internal_shape_range( world, object->desc.id, &begin, &end );
    for( index = 0; index != world->shape_count; ++index )
    {
        kf_index_leaf_t * leaf = world->leaves + index;
        if( leaf->shape_index >= begin && leaf->shape_index < end )
        {
            leaf->bounds = world->shape_bounds[leaf->shape_index];
        }
    }
    index = world->node_count;
    while( index != 0 )
    {
        kf_index_node_t * node = world->nodes + --index;
        if( node->shape_index == SIZE_MAX )
        {
            node->bounds = kf_bounds_union( world->nodes[node->left].bounds, world->nodes[node->right].bounds );
        }
        else if( node->shape_index >= begin && node->shape_index < end )
        {
            node->bounds = world->shape_bounds[node->shape_index];
        }
    }
}

static kf_bool_t kf_leaf_less( kf_index_leaf_t left, kf_index_leaf_t right )
{
    if( left.bounds.minimum.x != right.bounds.minimum.x )
    {
        return (kf_bool_t)(left.bounds.minimum.x < right.bounds.minimum.x);
    }
    return (kf_bool_t)(left.shape_index < right.shape_index);
}

static void kf_leaf_sift( kf_index_leaf_t * leaves, size_t root, size_t count )
{
    while( root < count / 2u )
    {
        size_t child = root * 2u + 1u;
        kf_index_leaf_t temporary;
        if( child + 1u < count && kf_leaf_less( leaves[child], leaves[child + 1u] ) == KF_TRUE )
        {
            ++child;
        }
        if( kf_leaf_less( leaves[root], leaves[child] ) == KF_FALSE )
        {
            return;
        }
        temporary = leaves[root];
        leaves[root] = leaves[child];
        leaves[child] = temporary;
        root = child;
    }

}

void kf_internal_sort_leaves( kf_index_leaf_t * leaves, size_t count )
{
    size_t root = count / 2u;
    size_t end = count;
    while( root != 0 )
    {
        kf_leaf_sift( leaves, --root, count );
    }
    while( end > 1u )
    {
        kf_index_leaf_t temporary = leaves[0];
        leaves[0] = leaves[--end];
        leaves[end] = temporary;
        kf_leaf_sift( leaves, 0, end );
    }
}

static size_t kf_index_build( kf_world_t * world, size_t begin, size_t end )
{
    size_t node_index = world->node_count++;
    size_t middle = begin + (end - begin) / 2u;
    kf_index_node_t * node = world->nodes + node_index;
    uint32_t left_owner;
    node->shape_index = SIZE_MAX;
    if( end - begin == 1u )
    {
        node->bounds = world->leaves[begin].bounds;
        node->shape_index = world->leaves[begin].shape_index;
        node->object_id = world->shapes[node->shape_index].object_id;
        node->left = SIZE_MAX;
        node->right = SIZE_MAX;
        return node_index;
    }
    node->left = kf_index_build( world, begin, middle );
    node->right = kf_index_build( world, middle, end );
    node->bounds = kf_bounds_union( world->nodes[node->left].bounds, world->nodes[node->right].bounds );
    left_owner = world->nodes[node->left].object_id;
    node->object_id = left_owner == world->nodes[node->right].object_id ? left_owner : 0;
    return node_index;
}

void kf_internal_build_tree( kf_world_t * world )
{
    world->node_count = 0;
    if( world->shape_count != 0 )
    {
        (void)kf_index_build( world, 0, world->shape_count );
    }
}

kf_status_t kf_world_rebuild_index( kf_world_t * world )
{
    size_t index;
    kf_status_t status;
    if( world == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    status = kf_internal_refresh_bounds( world );
    if( status != KF_OK )
    {
        return status;
    }
    for( index = 0; index != world->shape_count; ++index )
    {
        world->leaves[index].shape_index = index;
        world->leaves[index].bounds = world->shape_bounds[index];
    }
    /* Heap sorting bounds rebuild cost without hidden allocations. */
    kf_internal_sort_leaves( world->leaves, world->shape_count );
    kf_internal_build_tree( world );
    world->index_dirty = KF_FALSE;
    return KF_OK;
}

kf_status_t kf_world_save( const kf_world_t * world, kf_snapshot_t * snapshot )
{
    if( world == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( snapshot == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    snapshot->version = KF_STATE_VERSION;
    snapshot->config = world->config;
    snapshot->tick = world->tick;
    snapshot->object_count = world->object_count;
    snapshot->shape_count = world->shape_count;
    snapshot->constraint_count = world->constraint_count;
    snapshot->contact_count = world->cache_count;
    snapshot->trigger_count = world->trigger_count;
    if( snapshot->object_capacity < world->object_count )
    {
        return KF_CAPACITY_EXCEEDED;
    }
    if( snapshot->shape_capacity < world->shape_count )
    {
        return KF_CAPACITY_EXCEEDED;
    }
    if( snapshot->constraint_capacity < world->constraint_count )
    {
        return KF_CAPACITY_EXCEEDED;
    }
    if( snapshot->contact_capacity < world->cache_count )
    {
        return KF_CAPACITY_EXCEEDED;
    }
    if( snapshot->trigger_capacity < world->trigger_count )
    {
        return KF_CAPACITY_EXCEEDED;
    }
    if( world->object_count != 0 && snapshot->objects == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( world->shape_count != 0 && snapshot->shapes == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( world->constraint_count != 0 && snapshot->constraints == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( world->cache_count != 0 && snapshot->contacts == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( world->trigger_count != 0 && snapshot->triggers == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( world->object_count != 0 )
    {
        memcpy( snapshot->objects, world->objects, world->object_count * sizeof(*world->objects) );
    }
    if( world->cache_count != 0 )
    {
        memcpy( snapshot->contacts, world->cache, world->cache_count * sizeof(*world->cache) );
    }
    if( world->trigger_count != 0 )
    {
        memcpy( snapshot->triggers, world->triggers, world->trigger_count * sizeof(*world->triggers) );
    }
    if( world->shape_count != 0 )
    {
        memcpy( snapshot->shapes, world->shapes, world->shape_count * sizeof(*world->shapes) );
    }
    if( world->constraint_count != 0 )
    {
        memcpy( snapshot->constraints, world->constraints, world->constraint_count * sizeof(*world->constraints) );
    }
    return KF_OK;
}

static const kf_object_snapshot_t * kf_snapshot_object( const kf_snapshot_t * snapshot, kf_handle_t handle )
{
    size_t low = 0;
    size_t high = snapshot->object_count;
    while( low != high )
    {
        size_t middle = low + (high - low) / 2u;
        const kf_object_snapshot_t * object = snapshot->objects + middle;
        if( object->desc.id < handle.id )
        {
            low = middle + 1u;
            continue;
        }
        if( object->desc.id > handle.id )
        {
            high = middle;
            continue;
        }
        if( object->state.handle.generation != handle.generation )
        {
            return NULL;
        }
        if( object->active == KF_FALSE )
        {
            return NULL;
        }
        return object;
    }
    return NULL;
}

static kf_bool_t kf_snapshot_has_shape( const kf_snapshot_t * snapshot, uint32_t object_id, uint32_t shape_id )
{
    size_t low = 0;
    size_t high = snapshot->shape_count;
    while( low != high )
    {
        size_t middle = low + (high - low) / 2u;
        const kf_shape_snapshot_t * shape = snapshot->shapes + middle;
        if( shape->object_id < object_id || (shape->object_id == object_id && shape->desc.id < shape_id) )
        {
            low = middle + 1u;
            continue;
        }
        if( shape->object_id > object_id || shape->desc.id != shape_id )
        {
            high = middle;
            continue;
        }
        return KF_TRUE;
    }
    return KF_FALSE;
}

kf_bool_t kf_internal_contact_less( const kf_contact_snapshot_t * left, const kf_contact_snapshot_t * right )
{
    uint32_t left_key[4] = {left->left_object, left->left_shape, left->right_object, left->right_shape};
    uint32_t right_key[4] = {right->left_object, right->left_shape, right->right_object, right->right_shape};
    size_t index;
    for( index = 0; index != 4u; ++index )
    {
        if( left_key[index] != right_key[index] )
        {
            return (kf_bool_t)(left_key[index] < right_key[index]);
        }
    }
    return KF_FALSE;
}

int32_t kf_internal_trigger_compare( const kf_trigger_overlap_t * left, const kf_trigger_overlap_t * right )
{
    uint32_t left_key[6] = {left->trigger.id, left->trigger_shape, left->visitor.id, left->visitor_shape, left->trigger.generation, left->visitor.generation};
    uint32_t right_key[6] = {right->trigger.id, right->trigger_shape, right->visitor.id, right->visitor_shape, right->trigger.generation, right->visitor.generation};
    size_t index;
    for( index = 0; index != 6u; ++index )
    {
        if( left_key[index] != right_key[index] )
        {
            return left_key[index] < right_key[index] ? -1 : 1;
        }
    }
    return 0;
}

static kf_status_t kf_validate_snapshot( const kf_snapshot_t * snapshot )
{
    kf_status_t status;
    size_t index;
    size_t object_cursor = 0;
    if( snapshot->version != KF_STATE_VERSION )
    {
        return KF_UNSUPPORTED;
    }
    status = kf_internal_validate_config( &snapshot->config );
    if( status != KF_OK )
    {
        return status;
    }
    if( snapshot->object_count > snapshot->object_capacity )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( snapshot->shape_count > snapshot->shape_capacity )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( snapshot->constraint_count > snapshot->constraint_capacity )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( snapshot->object_count != 0 && snapshot->objects == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( snapshot->shape_count != 0 && snapshot->shapes == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( snapshot->constraint_count != 0 && snapshot->constraints == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( snapshot->contact_count > snapshot->contact_capacity )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( snapshot->contact_count != 0 && snapshot->contacts == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    for( index = 0; index != snapshot->contact_count; ++index )
    {
        const kf_contact_snapshot_t * contact = snapshot->contacts + index;
        if( contact->count == 0 || contact->count > KF_MANIFOLD_MAX_POINTS || contact->touching > KF_TRUE )
        {
            return KF_INVALID_ARGUMENT;
        }
        if( index != 0 && kf_internal_contact_less( contact - 1, contact ) == KF_FALSE )
        {
            return KF_INVALID_ARGUMENT;
        }
    }
    if( snapshot->trigger_count > snapshot->trigger_capacity )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( snapshot->trigger_count != 0 && snapshot->triggers == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    for( index = 1; index < snapshot->trigger_count; ++index )
    {
        if( kf_internal_trigger_compare( snapshot->triggers + index - 1u, snapshot->triggers + index ) >= 0 )
        {
            return KF_INVALID_ARGUMENT;
        }
    }
    for( index = 0; index != snapshot->object_count; ++index )
    {
        const kf_object_snapshot_t * object = snapshot->objects + index;
        if( index != 0 && snapshot->objects[index - 1u].desc.id >= object->desc.id )
        {
            return KF_DUPLICATE_ID;
        }
        if( object->active > KF_TRUE )
        {
            return KF_INVALID_ARGUMENT;
        }
        status = kf_internal_validate_body( &object->desc );
        if( status != KF_OK )
        {
            return status;
        }
        status = kf_validate_body_state( &object->desc, &object->state, &snapshot->config );
        if( status != KF_OK )
        {
            return status;
        }
    }
    for( index = 0; index != snapshot->shape_count; ++index )
    {
        const kf_shape_snapshot_t * shape = snapshot->shapes + index;
        const kf_object_snapshot_t * object = NULL;
        kf_aabb_t bounds;
        if( shape->desc.id == 0 )
        {
            return KF_INVALID_ARGUMENT;
        }
        if( index != 0 )
        {
            const kf_shape_snapshot_t * previous = snapshot->shapes + index - 1u;
            if( previous->object_id > shape->object_id )
            {
                return KF_INVALID_ARGUMENT;
            }
            if( previous->object_id == shape->object_id && previous->desc.id >= shape->desc.id )
            {
                return KF_DUPLICATE_ID;
            }
        }
        while( object_cursor != snapshot->object_count && snapshot->objects[object_cursor].desc.id < shape->object_id )
        {
            ++object_cursor;
        }
        if( object_cursor != snapshot->object_count && snapshot->objects[object_cursor].desc.id == shape->object_id )
        {
            object = snapshot->objects + object_cursor;
        }
        if( object == NULL )
        {
            return KF_STALE_HANDLE;
        }
        if( object->active == KF_FALSE )
        {
            return KF_STALE_HANDLE;
        }
        if( object->desc.type != KF_BODY_STATIC && shape->desc.shape.type == KF_SHAPE_TRIANGLE )
        {
            return KF_UNSUPPORTED;
        }
        status = kf_shape_bounds( &shape->desc.shape, object->state.pose, &bounds );
        if( status != KF_OK )
        {
            return status;
        }
        if( object->state.has_target == KF_TRUE )
        {
            status = kf_shape_bounds( &shape->desc.shape, object->state.target, &bounds );
            if( status != KF_OK )
            {
                return status;
            }
        }
    }
    for( index = 0; index != snapshot->object_count; ++index )
    {
        const kf_object_snapshot_t * object = snapshot->objects + index;
        kf_world_t view = {0};
        kf_mass_t mass;
        if( object->active == KF_FALSE || object->state.has_target == KF_FALSE )
        {
            continue;
        }
        /* The existing mass calculation only reads this view's sorted shapes. */
        view.shapes = snapshot->shapes;
        view.shape_count = snapshot->shape_count;
        status = kf_internal_body_mass( &view, object, &mass );
        if( status != KF_OK )
        {
            return status;
        }
        status = kf_internal_validate_kinematic_target( &snapshot->config, &object->state, &mass );
        if( status != KF_OK )
        {
            return status;
        }
    }
    for( index = 0; index != snapshot->contact_count; ++index )
    {
        const kf_contact_snapshot_t * contact = snapshot->contacts + index;
        uint32_t point;
        if( contact->left_object >= contact->right_object )
        {
            return KF_INVALID_ARGUMENT;
        }
        if( kf_snapshot_has_shape( snapshot, contact->left_object, contact->left_shape ) == KF_FALSE )
        {
            return KF_STALE_HANDLE;
        }
        if( kf_snapshot_has_shape( snapshot, contact->right_object, contact->right_shape ) == KF_FALSE )
        {
            return KF_STALE_HANDLE;
        }
        for( point = 0; point != contact->count; ++point )
        {
            const kf_contact_point_snapshot_t * value = contact->points + point;
            size_t axis;
            /* A contact and a body's center lie within the world. Rotation
               can enlarge each body-frame component by at most sqrt(3). */
            if( kf_vector_in_range( value->anchor, KF_COORDINATE_LIMIT * 4 ) == KF_FALSE )
            {
                return KF_OUT_OF_RANGE;
            }
            if( value->normal_impulse < 0 )
            {
                return KF_OUT_OF_RANGE;
            }
            for( axis = 0; axis != 3u; ++axis )
            {
                if( value->tangent_impulse[axis] == INT64_MIN )
                {
                    return KF_OUT_OF_RANGE;
                }
            }
        }
    }
    for( index = 0; index != snapshot->constraint_count; ++index )
    {
        const kf_constraint_snapshot_t * slot = snapshot->constraints + index;
        const kf_constraint_state_t * state = &slot->state;
        const kf_constraint_desc_t * desc = &state->desc;
        if( index != 0 && snapshot->constraints[index - 1u].state.desc.id >= desc->id )
        {
            return KF_DUPLICATE_ID;
        }
        if( slot->active > KF_TRUE )
        {
            return KF_INVALID_ARGUMENT;
        }
        if( desc->id == 0 || state->handle.id != desc->id || state->handle.generation == 0 )
        {
            return KF_INVALID_ARGUMENT;
        }
        if( desc->type != KF_CONSTRAINT_DISTANCE && desc->type != KF_CONSTRAINT_MAX_DISTANCE )
        {
            return KF_INVALID_ARGUMENT;
        }
        if( desc->length < 0 || desc->length > KF_SIZE_LIMIT )
        {
            return KF_OUT_OF_RANGE;
        }
        if( desc->hertz < 0 || desc->hertz > KF_SPEED_LIMIT || desc->damping_ratio < 0 || desc->damping_ratio > KF_SPEED_LIMIT )
        {
            return KF_OUT_OF_RANGE;
        }
        if( kf_vector_in_range( desc->anchor_left, KF_SIZE_LIMIT ) == KF_FALSE )
        {
            return KF_OUT_OF_RANGE;
        }
        if( kf_vector_in_range( desc->anchor_right, KF_SIZE_LIMIT ) == KF_FALSE )
        {
            return KF_OUT_OF_RANGE;
        }
        if( slot->active == KF_FALSE )
        {
            continue;
        }
        if( desc->left.id == desc->right.id )
        {
            return KF_INVALID_ARGUMENT;
        }
        if( kf_snapshot_object( snapshot, desc->left ) == NULL )
        {
            return KF_STALE_HANDLE;
        }
        if( kf_snapshot_object( snapshot, desc->right ) == NULL )
        {
            return KF_STALE_HANDLE;
        }
    }
    return KF_OK;
}

kf_status_t kf_world_restore( kf_world_t * world, const kf_snapshot_t * snapshot )
{
    kf_status_t status;
    if( world == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( snapshot == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( snapshot->object_count > world->capacity.objects )
    {
        return KF_CAPACITY_EXCEEDED;
    }
    if( snapshot->shape_count > world->capacity.shapes )
    {
        return KF_CAPACITY_EXCEEDED;
    }
    if( snapshot->constraint_count > world->capacity.constraints )
    {
        return KF_CAPACITY_EXCEEDED;
    }
    if( snapshot->contact_count > world->capacity.contacts )
    {
        return KF_CAPACITY_EXCEEDED;
    }
    if( snapshot->trigger_count > world->capacity.triggers )
    {
        return KF_CAPACITY_EXCEEDED;
    }
    status = kf_validate_snapshot( snapshot );
    if( status != KF_OK )
    {
        return status;
    }
    if( snapshot->object_count != 0 )
    {
        memcpy( world->objects, snapshot->objects, snapshot->object_count * sizeof(*world->objects) );
    }
    if( snapshot->shape_count != 0 )
    {
        memcpy( world->shapes, snapshot->shapes, snapshot->shape_count * sizeof(*world->shapes) );
    }
    if( snapshot->constraint_count != 0 )
    {
        memcpy( world->constraints, snapshot->constraints, snapshot->constraint_count * sizeof(*world->constraints) );
    }
    if( snapshot->contact_count != 0 )
    {
        memcpy( world->cache, snapshot->contacts, snapshot->contact_count * sizeof(*world->cache) );
    }
    world->cache_count = snapshot->contact_count;
    if( snapshot->trigger_count != 0 )
    {
        memcpy( world->triggers, snapshot->triggers, snapshot->trigger_count * sizeof(*world->triggers) );
    }
    world->trigger_count = snapshot->trigger_count;
    world->triggers_valid = KF_FALSE;
    world->config = snapshot->config;
    world->tick = snapshot->tick;
    world->object_count = snapshot->object_count;
    world->shape_count = snapshot->shape_count;
    world->constraint_count = snapshot->constraint_count;
    kf_internal_invalidate( world );
    status = kf_world_rebuild_index( world );
    return status;
}
