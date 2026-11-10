#include "kinefix_internal.h"

#include <string.h>

typedef enum kf_query_mode_t { KF_MODE_SWEEP, KF_MODE_RAY, KF_MODE_CONTACT, KF_MODE_DEEP_CONTACT } kf_query_mode_t;
typedef struct kf_query_context_t
{
    kf_world_t * world;
    kf_query_mode_t mode;
    const kf_shape_t * shape;
    kf_pose_t pose;
    kf_vec3_t displacement;
    const kf_ray_t * ray;
    const kf_query_filter_t * filter;
    kf_aabb_t bounds;
    void * output;
    size_t capacity;
    size_t count;
    kf_bool_t moving_targets;
    kf_contact_t deepest;
} kf_query_context_t;

static kf_status_t kf_filter_validate( const kf_query_filter_t * filter )
{
    if( filter == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( filter->exclude_count != 0 && filter->exclude == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( filter->include_triggers > KF_TRUE )
    {
        return KF_INVALID_ARGUMENT;
    }
    return KF_OK;
}

static kf_bool_t kf_filter_accept( const kf_object_slot_t * object, const kf_query_filter_t * filter )
{
    size_t index;
    if( object->active == KF_FALSE )
    {
        return KF_FALSE;
    }
    if( (object->desc.category & filter->mask) == 0u )
    {
        return KF_FALSE;
    }
    if( (filter->category & object->desc.mask) == 0u )
    {
        return KF_FALSE;
    }
    if( object->desc.trigger == KF_TRUE && filter->include_triggers == KF_FALSE )
    {
        return KF_FALSE;
    }
    for( index = 0; index != filter->exclude_count; ++index )
    {
        if( filter->exclude[index].id != object->state.handle.id )
        {
            continue;
        }
        if( filter->exclude[index].generation != object->state.handle.generation )
        {
            continue;
        }
        return KF_FALSE;
    }
    return KF_TRUE;
}

static kf_bool_t kf_hit_before( const kf_sweep_hit_t * left, const kf_sweep_hit_t * right )
{
    if( left->fraction_q32 != right->fraction_q32 )
    {
        return (kf_bool_t)(left->fraction_q32 < right->fraction_q32);
    }
    if( left->object.id != right->object.id )
    {
        return (kf_bool_t)(left->object.id < right->object.id);
    }
    return (kf_bool_t)(left->shape_id < right->shape_id);
}

static kf_bool_t kf_contact_before( const kf_contact_t * left, const kf_contact_t * right )
{
    if( left->object.id != right->object.id )
    {
        return (kf_bool_t)(left->object.id < right->object.id);
    }
    return (kf_bool_t)(left->shape_id < right->shape_id);
}

static void kf_query_insert_hit( kf_query_context_t * context, kf_sweep_hit_t hit )
{
    size_t stored = context->count < context->capacity ? context->count : context->capacity;
    kf_sweep_hit_t * output = context->output;
    size_t insert = 0;
    ++context->count;
    while( insert < stored && kf_hit_before( &hit, output + insert ) == KF_FALSE )
    {
        ++insert;
    }
    if( insert >= context->capacity )
    {
        return;
    }
    if( stored == context->capacity )
    {
        --stored;
    }
    while( stored > insert )
    {
        output[stored] = output[stored - 1u];
        --stored;
    }
    output[insert] = hit;
}

static void kf_query_insert_contact( kf_query_context_t * context, kf_contact_t contact )
{
    size_t stored = context->count < context->capacity ? context->count : context->capacity;
    kf_contact_t * output = context->output;
    size_t insert = 0;
    ++context->count;
    while( insert < stored && kf_contact_before( &contact, output + insert ) == KF_FALSE )
    {
        ++insert;
    }
    if( insert >= context->capacity )
    {
        return;
    }
    if( stored == context->capacity )
    {
        --stored;
    }
    while( stored > insert )
    {
        output[stored] = output[stored - 1u];
        --stored;
    }
    output[insert] = contact;
}

static kf_status_t kf_query_shape( kf_query_context_t * context, size_t shape_index )
{
    const kf_shape_slot_t * slot = context->world->shapes + shape_index;
    kf_object_slot_t * object = kf_internal_object_id( context->world, slot->object_id );
    kf_sweep_hit_t hit = {0};
    kf_status_t status;
    if( object == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( kf_filter_accept( object, context->filter ) == KF_FALSE )
    {
        return KF_OK;
    }
    if( context->mode == KF_MODE_CONTACT || context->mode == KF_MODE_DEEP_CONTACT )
    {
        kf_contact_t contact = {0};
        status = kf_shape_contact( context->shape, context->pose, &slot->desc.shape, object->state.pose, &contact );
        if( status == KF_NO_HIT )
        {
            return KF_OK;
        }
        if( status != KF_OK )
        {
            return status;
        }
        contact.object = object->state.handle;
        contact.shape_id = slot->desc.id;
        if( context->mode == KF_MODE_DEEP_CONTACT )
        {
            if( context->count == 0 || contact.depth > context->deepest.depth ||
                (contact.depth == context->deepest.depth && kf_contact_before( &contact, &context->deepest ) == KF_TRUE) )
            {
                context->deepest = contact;
            }
            ++context->count;
        }
        else
        {
            kf_query_insert_contact( context, contact );
        }
        return KF_OK;
    }
    if( context->mode == KF_MODE_RAY )
    {
        status = kf_shape_raycast( context->ray, &slot->desc.shape, object->state.pose, &hit );
    }
    else if( context->moving_targets == KF_TRUE )
    {
        kf_vec3_t motion = {0, 0, 0};
        if( object->desc.type != KF_BODY_STATIC )
        {
            /* One tick of substeps; remainders use 1 / (denominator * substeps) units. */
            int64_t numerator = (int64_t)context->world->config.step.numerator * context->world->config.substeps;
            int64_t denominator = (int64_t)context->world->config.step.denominator * context->world->config.substeps;
            motion.x = (kf_fixed_t)(((int64_t)object->state.velocity.x * numerator + object->state.position_remainder[0]) / denominator);
            motion.y = (kf_fixed_t)(((int64_t)object->state.velocity.y * numerator + object->state.position_remainder[1]) / denominator);
            motion.z = (kf_fixed_t)(((int64_t)object->state.velocity.z * numerator + object->state.position_remainder[2]) / denominator);
        }
        status = kf_shape_sweep_relative( context->shape, context->pose, context->displacement, &slot->desc.shape, object->state.pose, motion, &hit );
    }
    else
    {
        status = kf_shape_sweep( context->shape, context->pose, context->displacement, &slot->desc.shape, object->state.pose, &hit );
    }
    if( status == KF_NO_HIT )
    {
        return KF_OK;
    }
    if( status != KF_OK )
    {
        return status;
    }
    hit.object = object->state.handle;
    hit.shape_id = slot->desc.id;
    hit.trigger = object->desc.trigger;
    kf_query_insert_hit( context, hit );
    return KF_OK;
}

/* The segment's AABB alone also admits nodes far away from a diagonal ray.
   Expand the slabs for narrow-phase tolerance and fixed-point rounding. */
static kf_bool_t kf_query_ray_bounds( const kf_query_context_t * context, kf_aabb_t bounds )
{
    int64_t enter = 0;
    int64_t leave = INT64_C(1) << 32;
    int64_t origin[3] = {context->ray->origin.x, context->ray->origin.y, context->ray->origin.z};
    int64_t motion[3] = {context->displacement.x, context->displacement.y, context->displacement.z};
    int64_t minimum[3] = {bounds.minimum.x - 2 * KF_GEOMETRY_TOLERANCE, bounds.minimum.y - 2 * KF_GEOMETRY_TOLERANCE, bounds.minimum.z - 2 * KF_GEOMETRY_TOLERANCE};
    int64_t maximum[3] = {bounds.maximum.x + 2 * KF_GEOMETRY_TOLERANCE, bounds.maximum.y + 2 * KF_GEOMETRY_TOLERANCE, bounds.maximum.z + 2 * KF_GEOMETRY_TOLERANCE};
    size_t axis;
    for( axis = 0; axis != 3; ++axis )
    {
        int64_t low;
        int64_t high;
        if( motion[axis] == 0 )
        {
            if( origin[axis] < minimum[axis] || origin[axis] > maximum[axis] )
            {
                return KF_FALSE;
            }
            continue;
        }
        /* Valid coordinates keep these Q32 products within int64_t. */
        low = (minimum[axis] - origin[axis]) * (INT64_C(1) << 32) / motion[axis];
        high = (maximum[axis] - origin[axis]) * (INT64_C(1) << 32) / motion[axis];
        if( low > high )
        {
            int64_t swap = low;
            low = high;
            high = swap;
        }
        if( low > enter )
        {
            enter = low;
        }
        if( high < leave )
        {
            leave = high;
        }
        if( enter > leave )
        {
            return KF_FALSE;
        }
    }
    return KF_TRUE;
}

static kf_status_t kf_query_node( kf_query_context_t * context, size_t index )
{
    const kf_index_node_t * node = context->world->nodes + index;
    kf_status_t status;
    if( kf_internal_bounds_overlap( node->bounds, context->bounds ) == KF_FALSE )
    {
        return KF_OK;
    }
    if( context->mode == KF_MODE_RAY && kf_query_ray_bounds( context, node->bounds ) == KF_FALSE )
    {
        return KF_OK;
    }
    if( node->shape_index != SIZE_MAX )
    {
        status = kf_query_shape( context, node->shape_index );
        return status;
    }
    status = kf_query_node( context, node->left );
    if( status != KF_OK )
    {
        return status;
    }
    status = kf_query_node( context, node->right );
    return status;
}

static kf_status_t kf_query_run( kf_query_context_t * context )
{
    kf_status_t status;
    if( context->world->index_dirty == KF_TRUE )
    {
        status = kf_world_rebuild_index( context->world );
        if( status != KF_OK )
        {
            return status;
        }
    }
    if( context->moving_targets == KF_TRUE )
    {
        size_t index;
        /* Moving candidates can cross outside their current tree bounds. */
        for( index = 0; index != context->world->shape_count; ++index )
        {
            status = kf_query_shape( context, index );
            if( status != KF_OK )
            {
                return status;
            }
        }
    }
    else if( context->world->node_count != 0 )
    {
        status = kf_query_node( context, 0 );
        if( status != KF_OK )
        {
            return status;
        }
    }
    if( context->count > context->capacity && context->mode != KF_MODE_DEEP_CONTACT )
    {
        return KF_CAPACITY_EXCEEDED;
    }
    if( context->count == 0 )
    {
        return KF_NO_HIT;
    }
    return KF_OK;
}

static kf_status_t kf_query_initialize( kf_world_t * world, const kf_query_filter_t * filter, void * output, size_t capacity, size_t * required, kf_query_context_t * context )
{
    kf_status_t status;
    if( world == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( required == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    *required = 0;
    if( capacity != 0 && output == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    status = kf_filter_validate( filter );
    if( status != KF_OK )
    {
        return status;
    }
    memset( context, 0, sizeof(*context) );
    context->world = world;
    context->filter = filter;
    context->output = output;
    context->capacity = capacity;
    return KF_OK;
}

static kf_status_t kf_motion_bounds( const kf_shape_t * shape, kf_pose_t pose, kf_vec3_t motion, kf_aabb_t * bounds )
{
    kf_aabb_t start;
    kf_aabb_t end;
    kf_status_t status = kf_shape_bounds( shape, pose, &start );
    if( status != KF_OK )
    {
        return status;
    }
    status = kf_internal_vec_add( pose.position, motion, &pose.position );
    if( status != KF_OK )
    {
        return status;
    }
    status = kf_shape_bounds( shape, pose, &end );
    if( status != KF_OK )
    {
        return status;
    }
    bounds->minimum = (kf_vec3_t){kf_fixed_min( start.minimum.x, end.minimum.x ), kf_fixed_min( start.minimum.y, end.minimum.y ), kf_fixed_min( start.minimum.z, end.minimum.z )};
    bounds->maximum = (kf_vec3_t){kf_fixed_max( start.maximum.x, end.maximum.x ), kf_fixed_max( start.maximum.y, end.maximum.y ), kf_fixed_max( start.maximum.z, end.maximum.z )};
    /* A narrow-phase tolerance must not be culled by an exact broad phase. */
    bounds->minimum.x -= KF_GEOMETRY_TOLERANCE;
    bounds->minimum.y -= KF_GEOMETRY_TOLERANCE;
    bounds->minimum.z -= KF_GEOMETRY_TOLERANCE;
    bounds->maximum.x += KF_GEOMETRY_TOLERANCE;
    bounds->maximum.y += KF_GEOMETRY_TOLERANCE;
    bounds->maximum.z += KF_GEOMETRY_TOLERANCE;
    return KF_OK;
}

static kf_status_t kf_world_sweep_mode( kf_world_t * world, const kf_shape_t * shape, kf_pose_t pose, kf_vec3_t displacement, const kf_query_filter_t * filter, kf_sweep_hit_t * hits, size_t capacity, size_t * required, kf_bool_t moving_targets )
{
    kf_query_filter_t default_filter = kf_query_filter_default();
    kf_query_context_t context;
    kf_status_t status = kf_query_initialize( world, filter != NULL ? filter : &default_filter, hits, capacity, required, &context );
    if( status != KF_OK )
    {
        return status;
    }
    context.mode = KF_MODE_SWEEP;
    context.shape = shape;
    context.pose = pose;
    context.displacement = displacement;
    context.moving_targets = moving_targets;
    status = kf_motion_bounds( shape, pose, displacement, &context.bounds );
    if( status != KF_OK )
    {
        return status;
    }
    status = kf_query_run( &context );
    *required = context.count;
    return status;
}

kf_status_t kf_world_sweep_all( kf_world_t * world, const kf_shape_t * shape, kf_pose_t pose, kf_vec3_t displacement, const kf_query_filter_t * filter, kf_sweep_hit_t * hits, size_t capacity, size_t * required )
{
    kf_status_t status = kf_world_sweep_mode( world, shape, pose, displacement, filter, hits, capacity, required, KF_FALSE );
    return status;
}

kf_status_t kf_world_sweep_moving( kf_world_t * world, const kf_shape_t * shape, kf_pose_t pose, kf_vec3_t displacement, const kf_query_filter_t * filter, kf_sweep_hit_t * hits, size_t capacity, size_t * required )
{
    kf_status_t status = kf_world_sweep_mode( world, shape, pose, displacement, filter, hits, capacity, required, KF_TRUE );
    return status;
}

kf_status_t kf_world_sweep_first( kf_world_t * world, const kf_shape_t * shape, kf_pose_t pose, kf_vec3_t displacement, const kf_query_filter_t * filter, kf_sweep_hit_t * hit )
{
    size_t required;
    kf_status_t status;
    if( hit == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    status = kf_world_sweep_all( world, shape, pose, displacement, filter, hit, 1u, &required );
    if( status == KF_CAPACITY_EXCEEDED )
    {
        return KF_OK;
    }
    return status;
}

kf_status_t kf_world_contacts( kf_world_t * world, const kf_shape_t * shape, kf_pose_t pose, const kf_query_filter_t * filter, kf_contact_t * contacts, size_t capacity, size_t * required )
{
    kf_query_filter_t default_filter = kf_query_filter_default();
    kf_query_context_t context;
    kf_status_t status = kf_query_initialize( world, filter != NULL ? filter : &default_filter, contacts, capacity, required, &context );
    if( status != KF_OK )
    {
        return status;
    }
    context.mode = KF_MODE_CONTACT;
    context.shape = shape;
    context.pose = pose;
    status = kf_motion_bounds( shape, pose, (kf_vec3_t){0, 0, 0}, &context.bounds );
    if( status != KF_OK )
    {
        return status;
    }
    status = kf_query_run( &context );
    *required = context.count;
    return status;
}

kf_status_t kf_world_raycast_all( kf_world_t * world, const kf_ray_t * ray, const kf_query_filter_t * filter, kf_sweep_hit_t * hits, size_t capacity, size_t * required )
{
    kf_query_filter_t default_filter = kf_query_filter_default();
    kf_query_context_t context;
    kf_shape_t point = {0};
    kf_pose_t pose = {{0, 0, 0}, {0, 0, 0}};
    kf_vec3_t displacement;
    kf_vec3_t direction;
    kf_fixed_t maximum;
    kf_fixed_t length;
    kf_status_t status = kf_query_initialize( world, filter != NULL ? filter : &default_filter, hits, capacity, required, &context );
    if( status != KF_OK )
    {
        return status;
    }
    if( ray == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( ray->maximum_distance <= 0 )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( ray->maximum_distance > KF_COORDINATE_LIMIT * 2 )
    {
        return KF_OUT_OF_RANGE;
    }
    if( ray->direction.x < -KF_COORDINATE_LIMIT || ray->direction.x > KF_COORDINATE_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }
    if( ray->direction.y < -KF_COORDINATE_LIMIT || ray->direction.y > KF_COORDINATE_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }
    if( ray->direction.z < -KF_COORDINATE_LIMIT || ray->direction.z > KF_COORDINATE_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }
    maximum = kf_fixed_max( kf_fixed_abs( ray->direction.x ), kf_fixed_max( kf_fixed_abs( ray->direction.y ), kf_fixed_abs( ray->direction.z ) ) );
    if( maximum == 0 )
    {
        return KF_INVALID_ARGUMENT;
    }
    direction = kf_vec3_mul_div( ray->direction, KF_FIXED_SCALE, maximum );
    length = kf_vec3_length( direction );
    displacement = kf_vec3_mul_div( direction, ray->maximum_distance, length );
    point.type = KF_SHAPE_SPHERE;
    pose.position = ray->origin;
    context.mode = KF_MODE_RAY;
    context.ray = ray;
    context.displacement = displacement;
    status = kf_motion_bounds( &point, pose, displacement, &context.bounds );
    if( status != KF_OK )
    {
        return status;
    }
    status = kf_query_run( &context );
    *required = context.count;
    return status;
}

kf_status_t kf_world_raycast_first( kf_world_t * world, const kf_ray_t * ray, const kf_query_filter_t * filter, kf_sweep_hit_t * hit )
{
    size_t required;
    kf_status_t status;
    if( hit == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    status = kf_world_raycast_all( world, ray, filter, hit, 1u, &required );
    if( status == KF_CAPACITY_EXCEEDED )
    {
        return KF_OK;
    }
    return status;
}

kf_status_t kf_world_query_region( kf_world_t * world, kf_aabb_t region, const kf_query_filter_t * filter, kf_handle_t * objects, size_t capacity, size_t * required )
{
    kf_query_filter_t default_filter = kf_query_filter_default();
    kf_query_context_t context;
    size_t index;
    kf_status_t status = kf_query_initialize( world, filter != NULL ? filter : &default_filter, objects, capacity, required, &context );
    if( status != KF_OK )
    {
        return status;
    }
    if( region.minimum.x > region.maximum.x )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( region.minimum.y > region.maximum.y )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( region.minimum.z > region.maximum.z )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( region.minimum.x < -KF_COORDINATE_LIMIT || region.maximum.x > KF_COORDINATE_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }
    if( region.minimum.y < -KF_COORDINATE_LIMIT || region.maximum.y > KF_COORDINATE_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }
    if( region.minimum.z < -KF_COORDINATE_LIMIT || region.maximum.z > KF_COORDINATE_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }
    if( world->index_dirty == KF_TRUE )
    {
        status = kf_world_rebuild_index( world );
        if( status != KF_OK )
        {
            return status;
        }
    }
    for( index = 0; index != world->object_count; ++index )
    {
        const kf_object_slot_t * object = world->objects + index;
        size_t leaf;
        if( kf_filter_accept( object, context.filter ) == KF_FALSE )
        {
            continue;
        }
        for( leaf = 0; leaf != world->shape_count; ++leaf )
        {
            const kf_index_leaf_t * candidate = world->leaves + leaf;
            if( world->shapes[candidate->shape_index].object_id != object->desc.id )
            {
                continue;
            }
            if( kf_internal_bounds_overlap( candidate->bounds, region ) == KF_FALSE )
            {
                continue;
            }
            if( *required < capacity )
            {
                objects[*required] = object->state.handle;
            }
            ++*required;
            break;
        }
    }
    if( *required > capacity )
    {
        return KF_CAPACITY_EXCEEDED;
    }
    if( *required == 0 )
    {
        return KF_NO_HIT;
    }
    return KF_OK;
}

kf_status_t kf_world_query_batch( kf_world_t * world, kf_query_t * queries, size_t count )
{
    size_t index;
    if( world == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( count != 0 && queries == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    for( index = 0; index != count; ++index )
    {
        kf_query_t * query = queries + index;
        query->required = 0;
        switch( query->type )
        {
        case KF_QUERY_SWEEP:
            query->status = kf_world_sweep_all( world, &query->shape, query->pose, query->displacement, &query->filter, query->output, query->capacity, &query->required );
            break;
        case KF_QUERY_RAY:
            query->status = kf_world_raycast_all( world, &query->ray, &query->filter, query->output, query->capacity, &query->required );
            break;
        case KF_QUERY_CONTACTS:
            query->status = kf_world_contacts( world, &query->shape, query->pose, &query->filter, query->output, query->capacity, &query->required );
            break;
        case KF_QUERY_REGION:
            query->status = kf_world_query_region( world, query->region, &query->filter, query->output, query->capacity, &query->required );
            break;
        default:
            query->status = KF_INVALID_ARGUMENT;
            break;
        }
    }
    return KF_OK;
}

kf_status_t kf_world_depenetrate( kf_world_t * world, const kf_shape_t * shape, kf_pose_t * pose, const kf_query_filter_t * filter, uint32_t iterations, kf_fixed_t maximum_distance, kf_vec3_t * correction )
{
    kf_query_filter_t default_filter = kf_query_filter_default();
    kf_pose_t initial;
    uint32_t iteration;
    kf_status_t status;
    size_t ignored;
    kf_query_context_t context;
    if( pose == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( correction == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( iterations == 0 || iterations > 64u )
    {
        return KF_OUT_OF_RANGE;
    }
    if( maximum_distance < 0 || maximum_distance > KF_SIZE_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }
    initial = *pose;
    *correction = (kf_vec3_t){0, 0, 0};
    status = kf_query_initialize( world, filter != NULL ? filter : &default_filter, NULL, 0, &ignored, &context );
    if( status != KF_OK )
    {
        return status;
    }
    context.mode = KF_MODE_DEEP_CONTACT;
    context.shape = shape;
    for( iteration = 0; iteration <= iterations; ++iteration )
    {
        kf_vec3_t delta;
        kf_fixed_t remaining;
        context.pose = *pose;
        context.count = 0;
        status = kf_motion_bounds( shape, *pose, (kf_vec3_t){0, 0, 0}, &context.bounds );
        if( status != KF_OK )
        {
            return status;
        }
        status = kf_query_run( &context );
        if( status == KF_NO_HIT )
        {
            return KF_OK;
        }
        if( status != KF_OK )
        {
            return status;
        }
        if( context.deepest.depth <= world->config.contact_slop )
        {
            return KF_OK;
        }
        if( iteration == iterations )
        {
            return KF_ITERATION_LIMIT;
        }
        remaining = maximum_distance - kf_vec3_length( *correction );
        if( remaining <= 0 )
        {
            return KF_ITERATION_LIMIT;
        }
        delta = kf_vec3_mul( context.deepest.normal, kf_fixed_min( context.deepest.depth + KF_GEOMETRY_TOLERANCE, remaining ) );
        status = kf_internal_vec_add( pose->position, delta, &pose->position );
        if( status != KF_OK )
        {
            return status;
        }
        *correction = kf_vec3_sub( pose->position, initial.position );
    }
    return KF_ITERATION_LIMIT;
}

static kf_vec3_t kf_query_moved( kf_vec3_t _position, kf_vec3_t _displacement, uint64_t _fraction )
{
    return (kf_vec3_t){_position.x + (kf_fixed_t)((int64_t)_displacement.x * (int64_t)_fraction / (INT64_C(1) << 32)), _position.y + (kf_fixed_t)((int64_t)_displacement.y * (int64_t)_fraction / (INT64_C(1) << 32)), _position.z + (kf_fixed_t)((int64_t)_displacement.z * (int64_t)_fraction / (INT64_C(1) << 32))};
}

static kf_status_t kf_query_validate_motion( const kf_shape_t * _shape, kf_pose_t _pose, kf_vec3_t _motion )
{
    kf_aabb_t bounds;
    kf_status_t status;
    if( _motion.x < -KF_COORDINATE_LIMIT * 2 || _motion.x > KF_COORDINATE_LIMIT * 2 )
    {
        return KF_OUT_OF_RANGE;
    }
    if( _motion.y < -KF_COORDINATE_LIMIT * 2 || _motion.y > KF_COORDINATE_LIMIT * 2 )
    {
        return KF_OUT_OF_RANGE;
    }
    if( _motion.z < -KF_COORDINATE_LIMIT * 2 || _motion.z > KF_COORDINATE_LIMIT * 2 )
    {
        return KF_OUT_OF_RANGE;
    }
    status = kf_shape_bounds( _shape, _pose, &bounds );
    if( status != KF_OK )
    {
        return status;
    }
    _pose.position = kf_query_moved( _pose.position, _motion, UINT64_C(1) << 32 );
    return kf_shape_bounds( _shape, _pose, &bounds );
}

kf_status_t kf_shape_sweep_relative( const kf_shape_t * moving, kf_pose_t pose, kf_vec3_t displacement, const kf_shape_t * target, kf_pose_t target_pose, kf_vec3_t target_displacement, kf_sweep_hit_t * hit )
{
    kf_vec3_t relative;
    kf_status_t status;
    uint64_t fraction;
    if( hit == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    status = kf_query_validate_motion( moving, pose, displacement );
    if( status != KF_OK )
    {
        return status;
    }
    status = kf_query_validate_motion( target, target_pose, target_displacement );
    if( status != KF_OK )
    {
        return status;
    }
    relative = (kf_vec3_t){displacement.x - target_displacement.x, displacement.y - target_displacement.y, displacement.z - target_displacement.z};
    status = kf_internal_shape_sweep_fraction( moving, pose, relative, target, target_pose, hit, &fraction );
    if( status != KF_OK )
    {
        return status;
    }
    hit->contact_point = kf_query_moved( hit->contact_point, target_displacement, fraction );
    hit->position = kf_query_moved( pose.position, displacement, fraction );
    hit->distance = (kf_fixed_t)((int64_t)kf_vec3_length( displacement ) * (int64_t)fraction / (INT64_C(1) << 32));
    return KF_OK;
}

static kf_vec3_t kf_curve_point( kf_vec3_t start, kf_vec3_t control, kf_vec3_t end, uint32_t numerator, uint32_t denominator )
{
    /* De Casteljau at rational fractions, with truncation toward zero per lerp. */
    kf_vec3_t first = {
        (kf_fixed_t)((int64_t)start.x + ((int64_t)control.x - start.x) * numerator / denominator),
        (kf_fixed_t)((int64_t)start.y + ((int64_t)control.y - start.y) * numerator / denominator),
        (kf_fixed_t)((int64_t)start.z + ((int64_t)control.z - start.z) * numerator / denominator)};
    kf_vec3_t second = {
        (kf_fixed_t)((int64_t)control.x + ((int64_t)end.x - control.x) * numerator / denominator),
        (kf_fixed_t)((int64_t)control.y + ((int64_t)end.y - control.y) * numerator / denominator),
        (kf_fixed_t)((int64_t)control.z + ((int64_t)end.z - control.z) * numerator / denominator)};
    kf_vec3_t point = {
        (kf_fixed_t)((int64_t)first.x + ((int64_t)second.x - first.x) * numerator / denominator),
        (kf_fixed_t)((int64_t)first.y + ((int64_t)second.y - first.y) * numerator / denominator),
        (kf_fixed_t)((int64_t)first.z + ((int64_t)second.z - first.z) * numerator / denominator)};
    return point;
}

kf_status_t kf_world_sweep_curve( kf_world_t * world, const kf_shape_t * moving, kf_pose_t pose, kf_vec3_t control, kf_vec3_t end, kf_fixed_t tolerance, uint32_t maximum_segments, const kf_query_filter_t * filter, kf_sweep_hit_t * hits, size_t capacity, size_t * required )
{
    kf_query_filter_t default_filter = kf_query_filter_default();
    kf_query_context_t output;
    kf_vec3_t curvature;
    kf_vec3_t start = pose.position;
    kf_vec3_t previous = start;
    kf_fixed_t arc_distance = 0;
    uint32_t segments = 1;
    uint32_t segment;
    kf_status_t status = kf_query_initialize( world, filter != NULL ? filter : &default_filter, hits, capacity, required, &output );
    if( status != KF_OK )
    {
        return status;
    }
    if( tolerance <= 4 )
    {
        return KF_OUT_OF_RANGE;
    }
    if( maximum_segments == 0 || maximum_segments > 65536u )
    {
        return KF_OUT_OF_RANGE;
    }
    if( control.x < -KF_COORDINATE_LIMIT || control.x > KF_COORDINATE_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }
    if( control.y < -KF_COORDINATE_LIMIT || control.y > KF_COORDINATE_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }
    if( control.z < -KF_COORDINATE_LIMIT || control.z > KF_COORDINATE_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }
    if( end.x < -KF_COORDINATE_LIMIT || end.x > KF_COORDINATE_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }
    if( end.y < -KF_COORDINATE_LIMIT || end.y > KF_COORDINATE_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }
    if( end.z < -KF_COORDINATE_LIMIT || end.z > KF_COORDINATE_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }
    status = kf_shape_bounds( moving, pose, &output.bounds );
    if( status != KF_OK )
    {
        return status;
    }
    curvature = (kf_vec3_t){(kf_fixed_t)((int64_t)start.x - 2 * (int64_t)control.x + end.x),
        (kf_fixed_t)((int64_t)start.y - 2 * (int64_t)control.y + end.y),
        (kf_fixed_t)((int64_t)start.z - 2 * (int64_t)control.z + end.z)};
    /* Quadratic chord deviation <= |P0-2P1+P2|/(4*n*n), plus 4 raw rounding units. */
    while( (int64_t)kf_vec3_length( curvature ) > 4 * (int64_t)(tolerance - 4) * segments * segments )
    {
        if( segments > maximum_segments / 2u )
        {
            return KF_ITERATION_LIMIT;
        }
        segments *= 2u;
    }
    /* Validate every chord endpoint independently of candidates and filters. */
    for( segment = 1; segment <= segments; ++segment )
    {
        kf_pose_t endpoint = pose;
        kf_aabb_t endpoint_bounds;
        endpoint.position = kf_curve_point( start, control, end, segment, segments );
        status = kf_shape_bounds( moving, endpoint, &endpoint_bounds );
        if( status != KF_OK )
        {
            return status;
        }
    }
    for( segment = 0; segment != segments; ++segment )
    {
        kf_vec3_t next = kf_curve_point( start, control, end, segment + 1u, segments );
        kf_vec3_t displacement = kf_vec3_sub( next, previous );
        kf_fixed_t segment_length = kf_vec3_length( displacement );
        size_t shape_index;
        pose.position = previous;
        /* Keep every crossing, including later returns to the same obstacle. */
        for( shape_index = 0; shape_index != world->shape_count; ++shape_index )
        {
            const kf_shape_slot_t * slot = world->shapes + shape_index;
            kf_object_slot_t * object = kf_internal_object_id( world, slot->object_id );
            kf_sweep_hit_t hit = {0};
            if( kf_filter_accept( object, output.filter ) == KF_FALSE )
            {
                continue;
            }
            status = kf_shape_sweep( moving, pose, displacement, &slot->desc.shape, object->state.pose, &hit );
            if( status == KF_NO_HIT )
            {
                continue;
            }
            if( status != KF_OK )
            {
                return status;
            }
            hit.object = object->state.handle;
            hit.shape_id = slot->desc.id;
            hit.trigger = object->desc.trigger;
            status = kf_checked_add( arc_distance, hit.distance, &hit.distance );
            if( status != KF_OK )
            {
                return status;
            }
            hit.fraction_q32 = (((uint64_t)segment << 32) + hit.fraction_q32) / segments;
            hit.fraction = (kf_fixed_t)(hit.fraction_q32 >> 16);
            kf_query_insert_hit( &output, hit );
        }
        status = kf_checked_add( arc_distance, segment_length, &arc_distance );
        if( status != KF_OK )
        {
            return status;
        }
        previous = next;
    }
    *required = output.count;
    if( output.count > capacity )
    {
        return KF_CAPACITY_EXCEEDED;
    }
    if( output.count == 0 )
    {
        return KF_NO_HIT;
    }
    return KF_OK;
}
