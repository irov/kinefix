#include "kinefix_internal.h"

#include <limits.h>
#include <string.h>

/* Narrow phase uses raw Q32.32 dot products and bounded convex feature sets. */
#define KF_GEOMETRY_MAX_VERTICES ( KF_PRISM_MAX_VERTICES * 2u )
#define KF_GEOMETRY_MAX_EDGES ( KF_PRISM_MAX_VERTICES * 3u )
#define KF_GEOMETRY_MAX_AXES ( KF_PRISM_MAX_VERTICES + 2u )
#define KF_GEOMETRY_MAX_FACES ( KF_PRISM_MAX_VERTICES + 2u )
#define KF_GEOMETRY_MAX_INDICES ( KF_PRISM_MAX_VERTICES * 6u )
#define KF_GEOMETRY_MAX_CLIP ( KF_PRISM_MAX_VERTICES * 3u )
#define KF_GEOMETRY_SWEEP_ITERATIONS 96u
/* A face feature is kept unless another axis separates further by this amount. */
#define KF_GEOMETRY_FEATURE_TOLERANCE ( KF_GEOMETRY_TOLERANCE * 4 )
/* 2 * sqrt(2) in Q16.16; see kf_internal_quat_chord. */
#define KF_GEOMETRY_CHORD_SPEED INT64_C(185364)

#define KF_FEATURE_OTHER 0u
#define KF_FEATURE_LEFT_FACE 1u
#define KF_FEATURE_RIGHT_FACE 2u

typedef struct kf_geometry_edge_t
{
    uint32_t first;
    uint32_t second;
} kf_geometry_edge_t;

typedef struct kf_geometry_face_t
{
    kf_vec3_t normal;
    kf_vec3_t planeNormal; /* Q30 feature plane; public normals remain Q16 */
    uint32_t first;
    uint32_t count;
} kf_geometry_face_t;

typedef struct kf_geometry_t
{
    kf_bool_t rounded;
    kf_bool_t surface;
    kf_fixed_t radius;
    kf_vec3_t vertices[KF_GEOMETRY_MAX_VERTICES];
    uint32_t vertexCount;
    kf_geometry_edge_t edges[KF_GEOMETRY_MAX_EDGES];
    uint32_t edgeCount;
    kf_vec3_t axes[KF_GEOMETRY_MAX_AXES]; /* face normal directions */
    uint32_t axisCount;
    kf_vec3_t directions[KF_GEOMETRY_MAX_AXES]; /* distinct edge directions */
    uint32_t directionCount;
    kf_geometry_face_t faces[KF_GEOMETRY_MAX_FACES];
    uint32_t faceCount;
    uint8_t indices[KF_GEOMETRY_MAX_INDICES];
    uint32_t indexCount;
    kf_vec3_t center;
    kf_bool_t box; /* retained rigid frame and local bounds describe the polytope */
    kf_vec3_t boxPosition;
    kf_matrix_t boxRotation; /* rigid transform; Q16 face normals are not its inverse */
    kf_vec3_t boxMinimum;
    kf_vec3_t boxMaximum;
} kf_geometry_t;

typedef struct kf_geometry_separation_t
{
    kf_fixed_t distance;
    kf_vec3_t normal; /* from right toward left */
    kf_bool_t initialized;
    uint32_t feature;
    kf_bool_t witness;
    kf_vec3_t pointLeft; /* core witness for a rounded shape */
    kf_vec3_t pointRight;
} kf_geometry_separation_t;

typedef struct kf_geometry_frame_t
{
    kf_matrix_t matrix;
    kf_vec3_t position;
    kf_bool_t identity;
} kf_geometry_frame_t;

/* floor((numerator << bits) / denominator) for a quotient below 2^63; operands
   above INT64_MAX fall back to bitwise long division with the same quotient. */
static uint64_t kf_geometry_ratio( uint64_t _numerator, uint64_t _denominator, uint32_t _bits )
{
    uint64_t quotient;
    uint64_t remainder;
    uint32_t bit;
    if( _numerator <= (uint64_t)INT64_MAX && _denominator <= (uint64_t)INT64_MAX )
    {
        return (uint64_t)kf_internal_div_shift( (int64_t)_numerator, (int64_t)_denominator, _bits );
    }
    quotient = _numerator / _denominator;
    remainder = _numerator % _denominator;
    for( bit = 0; bit != _bits; ++bit )
    {
        quotient <<= 1;
        if( remainder >= _denominator - remainder )
        {
            remainder -= _denominator - remainder;
            quotient |= UINT64_C(1);
        }
        else
        {
            remainder += remainder;
        }
    }
    return quotient;
}

static kf_fixed_t kf_geometry_ratio_fixed( int64_t _numerator, int64_t _denominator )
{
    uint64_t quotient = kf_geometry_ratio( kf_internal_magnitude( _numerator ), kf_internal_magnitude( _denominator ), KF_FIXED_FRACTION_BITS );
    return ( _numerator < 0 ) != ( _denominator < 0 ) ? -(kf_fixed_t)quotient : (kf_fixed_t)quotient;
}

static int64_t kf_geometry_dot( kf_vec3_t _left, kf_vec3_t _right )
{
    return (int64_t)_left.x * _right.x + (int64_t)_left.y * _right.y + (int64_t)_left.z * _right.z;
}

static kf_fixed_t kf_geometry_length( kf_vec3_t _value )
{
    return (kf_fixed_t)kf_internal_sqrt( (uint64_t)kf_geometry_dot( _value, _value ) );
}

static kf_vec3_t kf_geometry_normalize( kf_vec3_t _value )
{
    uint64_t maximum = kf_internal_magnitude( _value.x );
    kf_vec3_t scaled;
    kf_fixed_t length;
    if( kf_internal_magnitude( _value.y ) > maximum )
    {
        maximum = kf_internal_magnitude( _value.y );
    }
    if( kf_internal_magnitude( _value.z ) > maximum )
    {
        maximum = kf_internal_magnitude( _value.z );
    }
    if( maximum == 0 )
    {
        return (kf_vec3_t){ 0, 0, 0 };
    }
    scaled.x = kf_geometry_ratio_fixed( _value.x, (int64_t)maximum );
    scaled.y = kf_geometry_ratio_fixed( _value.y, (int64_t)maximum );
    scaled.z = kf_geometry_ratio_fixed( _value.z, (int64_t)maximum );
    length = kf_geometry_length( scaled );
    return (kf_vec3_t){ (kf_fixed_t)( (int64_t)scaled.x * KF_FIXED_SCALE / length ), (kf_fixed_t)( (int64_t)scaled.y * KF_FIXED_SCALE / length ), (kf_fixed_t)( (int64_t)scaled.z * KF_FIXED_SCALE / length ) };
}

static kf_vec3_t kf_geometry_cross_normal( kf_vec3_t _left, kf_vec3_t _right )
{
    int64_t x = (int64_t)_left.y * _right.z - (int64_t)_left.z * _right.y;
    int64_t y = (int64_t)_left.z * _right.x - (int64_t)_left.x * _right.z;
    int64_t z = (int64_t)_left.x * _right.y - (int64_t)_left.y * _right.x;
    uint64_t maximum = kf_internal_magnitude( x );
    kf_vec3_t scaled;
    if( kf_internal_magnitude( y ) > maximum )
    {
        maximum = kf_internal_magnitude( y );
    }
    if( kf_internal_magnitude( z ) > maximum )
    {
        maximum = kf_internal_magnitude( z );
    }
    if( maximum == 0 )
    {
        return (kf_vec3_t){ 0, 0, 0 };
    }
    scaled.x = kf_geometry_ratio_fixed( x, (int64_t)maximum );
    scaled.y = kf_geometry_ratio_fixed( y, (int64_t)maximum );
    scaled.z = kf_geometry_ratio_fixed( z, (int64_t)maximum );
    return kf_geometry_normalize( scaled );
}

/* Internal directions retain Q30 ratios until their supporting projection.
   They need not be unit vectors: support radii and final distances use the
   retained direction's length. Only the public normal is narrowed to Q16. */
static kf_vec3_t kf_geometry_direction_q30( int64_t _x, int64_t _y, int64_t _z )
{
    int64_t values[3] = { _x, _y, _z };
    uint64_t maximum = kf_internal_magnitude( _x );
    kf_fixed_t components[3];
    uint32_t index;
    for( index = 1; index != 3u; ++index )
    {
        if( kf_internal_magnitude( values[index] ) > maximum )
        {
            maximum = kf_internal_magnitude( values[index] );
        }
    }
    if( maximum == 0 )
    {
        return (kf_vec3_t){ 0, 0, 0 };
    }
    for( index = 0; index != 3u; ++index )
    {
        int64_t ratio = (int64_t)kf_geometry_ratio( kf_internal_magnitude( values[index] ), maximum, 30 );
        components[index] = (kf_fixed_t)( values[index] < 0 ? -ratio : ratio );
    }
    return (kf_vec3_t){ components[0], components[1], components[2] };
}

static kf_vec3_t kf_geometry_cross_plane( kf_vec3_t _left, kf_vec3_t _right )
{
    return kf_geometry_direction_q30( (int64_t)_left.y * _right.z - (int64_t)_left.z * _right.y, (int64_t)_left.z * _right.x - (int64_t)_left.x * _right.z, (int64_t)_left.x * _right.y - (int64_t)_left.y * _right.x );
}

static kf_fixed_t kf_geometry_saturate( int64_t _value )
{
    if( _value > INT32_MAX )
    {
        return INT32_MAX;
    }
    if( _value < INT32_MIN )
    {
        return INT32_MIN;
    }
    return (kf_fixed_t)_value;
}

/* Base plus a Q16.16 multiple of a direction, truncated toward zero. */
static kf_vec3_t kf_geometry_offset( kf_vec3_t _base, kf_vec3_t _direction, int64_t _scalar )
{
    return (kf_vec3_t){ kf_geometry_saturate( _base.x + kf_internal_mul_shift( _direction.x, _scalar, 16 ) ), kf_geometry_saturate( _base.y + kf_internal_mul_shift( _direction.y, _scalar, 16 ) ), kf_geometry_saturate( _base.z + kf_internal_mul_shift( _direction.z, _scalar, 16 ) ) };
}

static kf_geometry_frame_t kf_geometry_frame( kf_pose_t _pose )
{
    kf_geometry_frame_t frame;
    memset( &frame, 0, sizeof( frame ) );
    frame.position = _pose.position;
    frame.identity = (kf_bool_t)( _pose.rotation.x == 0 && _pose.rotation.y == 0 && _pose.rotation.z == 0 );
    if( frame.identity == KF_FALSE )
    {
        frame.matrix = kf_internal_matrix( kf_internal_quat( _pose.rotation ) );
    }
    return frame;
}

static kf_vec3_t kf_geometry_direction( const kf_geometry_frame_t * _frame, kf_vec3_t _local )
{
    if( _frame->identity == KF_TRUE )
    {
        return _local;
    }
    return kf_internal_matrix_apply( &_frame->matrix, _local );
}

static kf_vec3_t kf_geometry_point( const kf_geometry_frame_t * _frame, kf_vec3_t _local )
{
    kf_vec3_t rotated = kf_geometry_direction( _frame, _local );
    return (kf_vec3_t){ kf_geometry_saturate( (int64_t)rotated.x + _frame->position.x ), kf_geometry_saturate( (int64_t)rotated.y + _frame->position.y ), kf_geometry_saturate( (int64_t)rotated.z + _frame->position.z ) };
}

static kf_bool_t kf_geometry_vector_valid( kf_vec3_t _value, kf_fixed_t _limit )
{
    if( _value.x < -_limit || _value.x > _limit )
    {
        return KF_FALSE;
    }
    if( _value.y < -_limit || _value.y > _limit )
    {
        return KF_FALSE;
    }
    if( _value.z < -_limit || _value.z > _limit )
    {
        return KF_FALSE;
    }
    return KF_TRUE;
}

static kf_bool_t kf_geometry_size_valid( kf_fixed_t _size )
{
    if( _size < 0 )
    {
        return KF_FALSE;
    }
    if( _size > KF_SIZE_LIMIT )
    {
        return KF_FALSE;
    }
    return KF_TRUE;
}

kf_status_t kf_shape_validate( const kf_shape_t * _shape )
{
    uint32_t index;
    if( _shape == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    switch( _shape->type )
    {
    case KF_SHAPE_AABB:
        if( kf_geometry_vector_valid( _shape->data.aabb.minimum, KF_COORDINATE_LIMIT ) == KF_FALSE )
        {
            return KF_OUT_OF_RANGE;
        }
        if( kf_geometry_vector_valid( _shape->data.aabb.maximum, KF_COORDINATE_LIMIT ) == KF_FALSE )
        {
            return KF_OUT_OF_RANGE;
        }
        if( _shape->data.aabb.minimum.x > _shape->data.aabb.maximum.x )
        {
            return KF_INVALID_ARGUMENT;
        }
        if( _shape->data.aabb.minimum.y > _shape->data.aabb.maximum.y )
        {
            return KF_INVALID_ARGUMENT;
        }
        if( _shape->data.aabb.minimum.z > _shape->data.aabb.maximum.z )
        {
            return KF_INVALID_ARGUMENT;
        }
        if( (int64_t)_shape->data.aabb.maximum.x - _shape->data.aabb.minimum.x > (int64_t)KF_SIZE_LIMIT * 2 )
        {
            return KF_OUT_OF_RANGE;
        }
        if( (int64_t)_shape->data.aabb.maximum.y - _shape->data.aabb.minimum.y > (int64_t)KF_SIZE_LIMIT * 2 )
        {
            return KF_OUT_OF_RANGE;
        }
        if( (int64_t)_shape->data.aabb.maximum.z - _shape->data.aabb.minimum.z > (int64_t)KF_SIZE_LIMIT * 2 )
        {
            return KF_OUT_OF_RANGE;
        }
        return KF_OK;
    case KF_SHAPE_SPHERE:
        if( kf_geometry_vector_valid( _shape->data.sphere.position, KF_COORDINATE_LIMIT ) == KF_FALSE )
        {
            return KF_OUT_OF_RANGE;
        }
        if( kf_geometry_size_valid( _shape->data.sphere.radius ) == KF_FALSE )
        {
            return KF_OUT_OF_RANGE;
        }
        return KF_OK;
    case KF_SHAPE_CAPSULE:
        if( kf_geometry_vector_valid( _shape->data.capsule.position, KF_COORDINATE_LIMIT ) == KF_FALSE )
        {
            return KF_OUT_OF_RANGE;
        }
        if( kf_geometry_size_valid( _shape->data.capsule.radius ) == KF_FALSE )
        {
            return KF_OUT_OF_RANGE;
        }
        if( kf_geometry_size_valid( _shape->data.capsule.height ) == KF_FALSE )
        {
            return KF_OUT_OF_RANGE;
        }
        if( (int64_t)_shape->data.capsule.height < (int64_t)_shape->data.capsule.radius * 2 )
        {
            return KF_INVALID_ARGUMENT;
        }
        return KF_OK;
    case KF_SHAPE_BOX:
        if( kf_geometry_vector_valid( _shape->data.box.center, KF_COORDINATE_LIMIT ) == KF_FALSE )
        {
            return KF_OUT_OF_RANGE;
        }
        if( kf_geometry_size_valid( _shape->data.box.half_extents.x ) == KF_FALSE )
        {
            return KF_OUT_OF_RANGE;
        }
        if( kf_geometry_size_valid( _shape->data.box.half_extents.y ) == KF_FALSE )
        {
            return KF_OUT_OF_RANGE;
        }
        if( kf_geometry_size_valid( _shape->data.box.half_extents.z ) == KF_FALSE )
        {
            return KF_OUT_OF_RANGE;
        }
        return KF_OK;
    case KF_SHAPE_PRISM:
    {
        int32_t winding = 0;
        const kf_prism_t * prism = &_shape->data.prism;
        if( prism->count < 3 || prism->count > KF_PRISM_MAX_VERTICES )
        {
            return KF_INVALID_ARGUMENT;
        }
        if( prism->minimum_y > prism->maximum_y )
        {
            return KF_INVALID_ARGUMENT;
        }
        if( prism->minimum_y < -KF_COORDINATE_LIMIT || prism->maximum_y > KF_COORDINATE_LIMIT )
        {
            return KF_OUT_OF_RANGE;
        }
        if( (int64_t)prism->maximum_y - prism->minimum_y > (int64_t)KF_SIZE_LIMIT * 2 )
        {
            return KF_OUT_OF_RANGE;
        }
        for( index = 0; index != prism->count; ++index )
        {
            if( kf_geometry_vector_valid( prism->vertices[index], KF_SIZE_LIMIT ) == KF_FALSE )
            {
                return KF_OUT_OF_RANGE;
            }
            if( prism->vertices[index].y != 0 )
            {
                return KF_INVALID_ARGUMENT;
            }
        }
        for( index = 0; index != prism->count; ++index )
        {
            uint32_t vertex;
            kf_vec3_t a = prism->vertices[index];
            kf_vec3_t b = prism->vertices[( index + 1 ) % prism->count];
            for( vertex = 0; vertex != prism->count; ++vertex )
            {
                kf_vec3_t c = prism->vertices[vertex];
                int64_t turn;
                int32_t sign;
                if( vertex == index || vertex == ( index + 1 ) % prism->count )
                {
                    continue;
                }
                turn = ( (int64_t)b.x - a.x ) * ( (int64_t)c.z - a.z ) - ( (int64_t)b.z - a.z ) * ( (int64_t)c.x - a.x );
                if( turn == 0 )
                {
                    return KF_INVALID_ARGUMENT;
                }
                sign = turn < 0 ? -1 : 1;
                if( winding != 0 && winding != sign )
                {
                    return KF_INVALID_ARGUMENT;
                }
                winding = sign;
            }
        }
        return KF_OK;
    }
    case KF_SHAPE_TRIANGLE:
        for( index = 0; index != 3; ++index )
        {
            if( kf_geometry_vector_valid( _shape->data.triangle.vertices[index], KF_COORDINATE_LIMIT ) == KF_FALSE )
            {
                return KF_OUT_OF_RANGE;
            }
        }
        {
            kf_vec3_t first = kf_vec3_sub( _shape->data.triangle.vertices[1], _shape->data.triangle.vertices[0] );
            kf_vec3_t second = kf_vec3_sub( _shape->data.triangle.vertices[2], _shape->data.triangle.vertices[0] );
            kf_vec3_t normal = kf_geometry_cross_normal( first, second );
            if( normal.x == 0 && normal.y == 0 && normal.z == 0 )
            {
                return KF_INVALID_ARGUMENT;
            }
        }
        return KF_OK;
    default:
        return KF_INVALID_ARGUMENT;
    }
}

static void kf_geometry_add_edge( kf_geometry_t * _geometry, uint32_t _first, uint32_t _second )
{
    _geometry->edges[_geometry->edgeCount++] = (kf_geometry_edge_t){ _first, _second };
}

/* Internal axes retain their raw directions until the supporting projection. */
static void kf_geometry_add_axis( kf_geometry_t * _geometry, kf_vec3_t _unit )
{
    _geometry->axes[_geometry->axisCount++] = _unit;
}

static void kf_geometry_add_direction( kf_geometry_t * _geometry, kf_vec3_t _unit )
{
    _geometry->directions[_geometry->directionCount++] = _unit;
}

static void kf_geometry_add_face( kf_geometry_t * _geometry, kf_vec3_t _unit, const uint32_t * _indices, uint32_t _count )
{
    kf_geometry_face_t * face = _geometry->faces + _geometry->faceCount++;
    uint32_t index;
    face->normal = _unit;
    face->planeNormal = kf_geometry_cross_plane( kf_vec3_sub( _geometry->vertices[_indices[1]], _geometry->vertices[_indices[0]] ), kf_vec3_sub( _geometry->vertices[_indices[2]], _geometry->vertices[_indices[0]] ) );
    if( face->planeNormal.x == 0 && face->planeNormal.y == 0 && face->planeNormal.z == 0 )
    {
        /* Zero-width boxes and zero-height extrusions retain their limiting
           face directions even when a face polygon collapses to an edge. */
        face->planeNormal = kf_geometry_direction_q30( _unit.x, _unit.y, _unit.z );
    }
    if( kf_geometry_dot( face->planeNormal, _unit ) < 0 )
    {
        face->planeNormal = kf_vec3_neg( face->planeNormal );
    }
    face->first = _geometry->indexCount;
    face->count = _count;
    for( index = 0; index != _count; ++index )
    {
        _geometry->indices[_geometry->indexCount++] = (uint8_t)_indices[index];
    }
}

static void kf_geometry_box( kf_geometry_t * _geometry, kf_vec3_t _minimum, kf_vec3_t _maximum, kf_angle16_t _yaw, kf_vec3_t _center, const kf_geometry_frame_t * _frame, kf_bool_t _features )
{
    static const uint32_t faces[6][4] = { { 0, 2, 6, 4 }, { 1, 3, 7, 5 }, { 0, 1, 5, 4 }, { 2, 3, 7, 6 }, { 0, 1, 3, 2 }, { 4, 5, 7, 6 } };
    kf_vec3_t axes[3];
    kf_geometry_frame_t frame = *_frame;
    uint32_t index;
    frame.position = kf_geometry_point( _frame, _center );
    if( _yaw != 0 )
    {
        kf_matrix_t yaw = kf_internal_matrix( kf_internal_quat( kf_rotation_from_yaw( _yaw ) ) );
        frame.matrix = yaw;
        frame.identity = KF_FALSE;
        if( _frame->identity == KF_FALSE )
        {
            uint32_t row;
            uint32_t column;
            for( row = 0; row != 3u; ++row )
            {
                for( column = 0; column != 3u; ++column )
                {
                    frame.matrix.m[row][column] = (int32_t)( ( (int64_t)_frame->matrix.m[row][0] * yaw.m[0][column] + (int64_t)_frame->matrix.m[row][1] * yaw.m[1][column] + (int64_t)_frame->matrix.m[row][2] * yaw.m[2][column] ) / KF_ROTATION_SCALE );
                }
            }
        }
    }
    if( frame.identity == KF_TRUE )
    {
        frame.matrix = kf_internal_matrix( (kf_quat_t){ 0, 0, 0, KF_ROTATION_SCALE } );
    }
    for( index = 0; index != 8; ++index )
    {
        kf_vec3_t local = { ( index & 1u ) == 0 ? _minimum.x : _maximum.x, ( index & 2u ) == 0 ? _minimum.y : _maximum.y, ( index & 4u ) == 0 ? _minimum.z : _maximum.z };
        _geometry->vertices[index] = kf_geometry_point( &frame, local );
    }
    _geometry->vertexCount = 8;
    if( _features == KF_FALSE )
    {
        return;
    }
    for( index = 0; index != 8; ++index )
    {
        if( ( index & 1u ) == 0 )
        {
            kf_geometry_add_edge( _geometry, index, index | 1u );
        }
        if( ( index & 2u ) == 0 )
        {
            kf_geometry_add_edge( _geometry, index, index | 2u );
        }
        if( ( index & 4u ) == 0 )
        {
            kf_geometry_add_edge( _geometry, index, index | 4u );
        }
    }
    axes[0] = kf_geometry_normalize( kf_geometry_direction( &frame, (kf_vec3_t){ KF_FIXED_SCALE, 0, 0 } ) );
    axes[1] = kf_geometry_normalize( kf_geometry_direction( &frame, (kf_vec3_t){ 0, KF_FIXED_SCALE, 0 } ) );
    axes[2] = kf_geometry_normalize( kf_geometry_direction( &frame, (kf_vec3_t){ 0, 0, KF_FIXED_SCALE } ) );
    for( index = 0; index != 3; ++index )
    {
        kf_vec3_t direction = { frame.matrix.m[0][index], frame.matrix.m[1][index], frame.matrix.m[2][index] };
        kf_geometry_add_axis( _geometry, direction );
        kf_geometry_add_direction( _geometry, direction );
    }
    _geometry->box = KF_TRUE;
    _geometry->boxRotation = frame.matrix;
    _geometry->boxPosition = frame.position;
    _geometry->boxMinimum = _minimum;
    _geometry->boxMaximum = _maximum;
    for( index = 0; index != 6; ++index )
    {
        kf_vec3_t normal = axes[index / 2u];
        kf_geometry_add_face( _geometry, ( index & 1u ) == 0 ? kf_vec3_neg( normal ) : normal, faces[index], 4 );
    }
}

static void kf_geometry_prism( kf_geometry_t * _geometry, const kf_prism_t * _prism, const kf_geometry_frame_t * _frame, kf_bool_t _features )
{
    uint32_t count = _prism->count;
    uint32_t bottom[KF_PRISM_MAX_VERTICES];
    uint32_t top[KF_PRISM_MAX_VERTICES];
    int64_t centerX = 0;
    int64_t centerZ = 0;
    kf_vec3_t up = kf_geometry_direction( _frame, (kf_vec3_t){ 0, KF_ROTATION_SCALE, 0 } );
    uint32_t index;
    for( index = 0; index != count; ++index )
    {
        kf_vec3_t low = _prism->vertices[index];
        kf_vec3_t high = low;
        low.y = _prism->minimum_y;
        high.y = _prism->maximum_y;
        _geometry->vertices[index] = kf_geometry_point( _frame, low );
        _geometry->vertices[index + count] = kf_geometry_point( _frame, high );
        centerX += _prism->vertices[index].x;
        centerZ += _prism->vertices[index].z;
    }
    _geometry->vertexCount = count * 2u;
    if( _features == KF_FALSE )
    {
        return;
    }
    centerX /= (int64_t)count;
    centerZ /= (int64_t)count;
    for( index = 0; index != count; ++index )
    {
        uint32_t next = ( index + 1u ) % count;
        uint32_t side[4] = { index, next, next + count, index + count };
        kf_vec3_t low = _prism->vertices[index];
        kf_vec3_t edge = kf_vec3_sub( _prism->vertices[next], low );
        kf_vec3_t outward = { edge.z, 0, -edge.x };
        kf_geometry_add_edge( _geometry, index, next );
        kf_geometry_add_edge( _geometry, index + count, next + count );
        kf_geometry_add_edge( _geometry, index, index + count );
        if( (int64_t)outward.x * ( centerX - low.x ) + (int64_t)outward.z * ( centerZ - low.z ) > 0 )
        {
            outward = kf_vec3_neg( outward );
        }
        /* The extrusion direction survives zero height and rounded vertex
           collapse. Transform the local feature directions at Q30 precision,
           rather than recovering them from a possibly zero world edge. */
        outward = kf_geometry_direction( _frame, kf_geometry_direction_q30( outward.x, outward.y, outward.z ) );
        kf_geometry_add_axis( _geometry, outward );
        kf_geometry_add_direction( _geometry, kf_geometry_direction( _frame, kf_geometry_direction_q30( edge.x, edge.y, edge.z ) ) );
        kf_geometry_add_face( _geometry, kf_geometry_normalize( outward ), side, 4 );
        _geometry->faces[_geometry->faceCount - 1u].planeNormal = outward;
        bottom[index] = index;
        top[index] = index + count;
    }
    kf_geometry_add_axis( _geometry, up );
    kf_geometry_add_direction( _geometry, up );
    kf_geometry_add_face( _geometry, kf_vec3_neg( kf_geometry_normalize( up ) ), bottom, count );
    _geometry->faces[_geometry->faceCount - 1u].planeNormal = kf_vec3_neg( up );
    kf_geometry_add_face( _geometry, kf_geometry_normalize( up ), top, count );
    _geometry->faces[_geometry->faceCount - 1u].planeNormal = up;
}

/* World shapes were validated when added; public entry points validate theirs.
   Without features only vertices and radius are needed. Extent calculation and
   containment also build intermediate coordinates outside the world bounds. */
static kf_status_t kf_geometry_build( const kf_shape_t * _shape, kf_pose_t _pose, kf_bool_t _validate, kf_bool_t _features, kf_bool_t _bounded, kf_geometry_t * _geometry )
{
    kf_geometry_frame_t frame;
    uint32_t index;
    int64_t centerX = 0;
    int64_t centerY = 0;
    int64_t centerZ = 0;
    if( _validate == KF_TRUE )
    {
        kf_status_t status = kf_shape_validate( _shape );
        if( status != KF_OK )
        {
            return status;
        }
    }
    if( _bounded == KF_TRUE && kf_geometry_vector_valid( _pose.position, KF_COORDINATE_LIMIT ) == KF_FALSE )
    {
        return KF_OUT_OF_RANGE;
    }
    if( kf_rotation_valid( _pose.rotation ) == KF_FALSE )
    {
        return KF_INVALID_ARGUMENT;
    }
    frame = kf_geometry_frame( _pose );
    _geometry->rounded = KF_FALSE;
    _geometry->surface = KF_FALSE;
    _geometry->box = KF_FALSE;
    _geometry->radius = 0;
    _geometry->vertexCount = 0;
    _geometry->edgeCount = 0;
    _geometry->axisCount = 0;
    _geometry->directionCount = 0;
    _geometry->faceCount = 0;
    _geometry->indexCount = 0;
    switch( _shape->type )
    {
    case KF_SHAPE_SPHERE:
        _geometry->rounded = KF_TRUE;
        _geometry->radius = _shape->data.sphere.radius;
        _geometry->vertices[0] = kf_geometry_point( &frame, _shape->data.sphere.position );
        _geometry->vertices[1] = _geometry->vertices[0];
        _geometry->vertexCount = 2;
        break;
    case KF_SHAPE_CAPSULE:
    {
        kf_vec3_t low = _shape->data.capsule.position;
        kf_vec3_t high = low;
        low.y += _shape->data.capsule.radius;
        high.y += _shape->data.capsule.height - _shape->data.capsule.radius;
        _geometry->rounded = KF_TRUE;
        _geometry->radius = _shape->data.capsule.radius;
        _geometry->vertices[0] = kf_geometry_point( &frame, low );
        _geometry->vertices[1] = kf_geometry_point( &frame, high );
        _geometry->vertexCount = 2;
    }
    break;
    case KF_SHAPE_AABB:
        kf_geometry_box( _geometry, _shape->data.aabb.minimum, _shape->data.aabb.maximum, 0, (kf_vec3_t){ 0, 0, 0 }, &frame, _features );
        break;
    case KF_SHAPE_BOX:
        kf_geometry_box( _geometry, kf_vec3_neg( _shape->data.box.half_extents ), _shape->data.box.half_extents, _shape->data.box.yaw, _shape->data.box.center, &frame, _features );
        break;
    case KF_SHAPE_PRISM:
        kf_geometry_prism( _geometry, &_shape->data.prism, &frame, _features );
        break;
    case KF_SHAPE_TRIANGLE:
    {
        static const uint32_t front[3] = { 0, 1, 2 };
        static const uint32_t back[3] = { 0, 2, 1 };
        kf_vec3_t normal;
        _geometry->vertexCount = 3;
        _geometry->surface = KF_TRUE;
        for( index = 0; index != 3; ++index )
        {
            _geometry->vertices[index] = kf_geometry_point( &frame, _shape->data.triangle.vertices[index] );
        }
        if( _features == KF_FALSE )
        {
            break;
        }
        for( index = 0; index != 3; ++index )
        {
            kf_geometry_add_edge( _geometry, index, ( index + 1u ) % 3u );
            kf_geometry_add_direction( _geometry, kf_vec3_sub( _geometry->vertices[( index + 1u ) % 3u], _geometry->vertices[index] ) );
        }
        normal = kf_geometry_normalize( kf_geometry_cross_normal( kf_vec3_sub( _geometry->vertices[1], _geometry->vertices[0] ), kf_vec3_sub( _geometry->vertices[2], _geometry->vertices[0] ) ) );
        kf_geometry_add_axis( _geometry, kf_geometry_cross_plane( kf_vec3_sub( _geometry->vertices[1], _geometry->vertices[0] ), kf_vec3_sub( _geometry->vertices[2], _geometry->vertices[0] ) ) );
        kf_geometry_add_face( _geometry, normal, front, 3 );
        kf_geometry_add_face( _geometry, kf_vec3_neg( normal ), back, 3 );
    }
    break;
    default:
        return KF_INVALID_ARGUMENT;
    }
    for( index = 0; index != _geometry->vertexCount; ++index )
    {
        kf_vec3_t vertex = _geometry->vertices[index];
        if( _bounded == KF_TRUE && ( vertex.x < -KF_COORDINATE_LIMIT + _geometry->radius || vertex.x > KF_COORDINATE_LIMIT - _geometry->radius ) )
        {
            return KF_OUT_OF_RANGE;
        }
        if( _bounded == KF_TRUE && ( vertex.y < -KF_COORDINATE_LIMIT + _geometry->radius || vertex.y > KF_COORDINATE_LIMIT - _geometry->radius ) )
        {
            return KF_OUT_OF_RANGE;
        }
        if( _bounded == KF_TRUE && ( vertex.z < -KF_COORDINATE_LIMIT + _geometry->radius || vertex.z > KF_COORDINATE_LIMIT - _geometry->radius ) )
        {
            return KF_OUT_OF_RANGE;
        }
        centerX += vertex.x;
        centerY += vertex.y;
        centerZ += vertex.z;
    }
    _geometry->center = (kf_vec3_t){ (kf_fixed_t)( centerX / _geometry->vertexCount ), (kf_fixed_t)( centerY / _geometry->vertexCount ), (kf_fixed_t)( centerZ / _geometry->vertexCount ) };
    return KF_OK;
}

static kf_status_t kf_geometry_bounds( const kf_shape_t * _shape, kf_pose_t _pose, kf_bool_t _validate, kf_bool_t _bounded, kf_aabb_t * _bounds )
{
    kf_geometry_t geometry;
    kf_status_t status;
    kf_aabb_t result;
    uint32_t index;
    if( _bounds == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    status = kf_geometry_build( _shape, _pose, _validate, KF_FALSE, _bounded, &geometry );
    if( status != KF_OK )
    {
        return status;
    }
    result.minimum = (kf_vec3_t){ INT32_MAX, INT32_MAX, INT32_MAX };
    result.maximum = (kf_vec3_t){ INT32_MIN, INT32_MIN, INT32_MIN };
    for( index = 0; index != geometry.vertexCount; ++index )
    {
        kf_vec3_t point = geometry.vertices[index];
        if( point.x < result.minimum.x )
        {
            result.minimum.x = point.x;
        }
        if( point.y < result.minimum.y )
        {
            result.minimum.y = point.y;
        }
        if( point.z < result.minimum.z )
        {
            result.minimum.z = point.z;
        }
        if( point.x > result.maximum.x )
        {
            result.maximum.x = point.x;
        }
        if( point.y > result.maximum.y )
        {
            result.maximum.y = point.y;
        }
        if( point.z > result.maximum.z )
        {
            result.maximum.z = point.z;
        }
    }
    result.minimum.x -= geometry.radius;
    result.minimum.y -= geometry.radius;
    result.minimum.z -= geometry.radius;
    result.maximum.x += geometry.radius;
    result.maximum.y += geometry.radius;
    result.maximum.z += geometry.radius;
    *_bounds = result;
    return KF_OK;
}

kf_status_t kf_shape_bounds( const kf_shape_t * _shape, kf_pose_t _pose, kf_aabb_t * _bounds )
{
    return kf_geometry_bounds( _shape, _pose, KF_TRUE, KF_TRUE, _bounds );
}

kf_status_t kf_internal_shape_bounds( const kf_shape_t * _shape, kf_pose_t _pose, kf_aabb_t * _bounds )
{
    return kf_geometry_bounds( _shape, _pose, KF_FALSE, KF_TRUE, _bounds );
}

kf_status_t kf_internal_shape_unbounded_bounds( const kf_shape_t * _shape, kf_pose_t _pose, kf_aabb_t * _bounds )
{
    return kf_geometry_bounds( _shape, _pose, KF_FALSE, KF_FALSE, _bounds );
}

static void kf_geometry_project( const kf_geometry_t * _geometry, kf_vec3_t _axis, kf_fixed_t _axisLength, int64_t * _minimum, int64_t * _maximum )
{
    int64_t minimum = kf_geometry_dot( _geometry->vertices[0], _axis );
    int64_t maximum = minimum;
    uint32_t index;
    for( index = 1; index != _geometry->vertexCount; ++index )
    {
        int64_t projection = kf_geometry_dot( _geometry->vertices[index], _axis );
        if( projection < minimum )
        {
            minimum = projection;
        }
        if( projection > maximum )
        {
            maximum = projection;
        }
    }
    *_minimum = minimum - (int64_t)_geometry->radius * _axisLength;
    *_maximum = maximum + (int64_t)_geometry->radius * _axisLength;
}

static kf_bool_t kf_geometry_test_axis( const kf_geometry_t * _left, const kf_geometry_t * _right, kf_vec3_t _axis, uint32_t _feature, kf_geometry_separation_t * _separation )
{
    int64_t leftMinimum;
    int64_t leftMaximum;
    int64_t rightMinimum;
    int64_t rightMaximum;
    int64_t positive;
    int64_t negative;
    int64_t distance;
    kf_fixed_t narrowed;
    kf_fixed_t axisLength;
    if( _axis.x == 0 && _axis.y == 0 && _axis.z == 0 )
    {
        return KF_FALSE;
    }
    _axis = kf_geometry_direction_q30( _axis.x, _axis.y, _axis.z );
    if( _axis.x < 0 || ( _axis.x == 0 && _axis.y < 0 ) || ( _axis.x == 0 && _axis.y == 0 && _axis.z < 0 ) )
    {
        _axis = kf_vec3_neg( _axis );
    }
    axisLength = kf_geometry_length( _axis );
    kf_geometry_project( _left, _axis, axisLength, &leftMinimum, &leftMaximum );
    kf_geometry_project( _right, _axis, axisLength, &rightMinimum, &rightMaximum );
    positive = leftMinimum - rightMaximum;
    negative = rightMinimum - leftMaximum;
    if( positive >= negative )
    {
        distance = positive;
    }
    else
    {
        distance = negative;
        _axis = kf_vec3_neg( _axis );
    }
    narrowed = (kf_fixed_t)( distance / axisLength );
    if( _separation->initialized == KF_TRUE && narrowed <= _separation->distance )
    {
        return KF_FALSE;
    }
    _separation->initialized = KF_TRUE;
    _separation->distance = narrowed;
    _separation->normal = (kf_vec3_t){ (kf_fixed_t)( (int64_t)_axis.x * KF_FIXED_SCALE / axisLength ), (kf_fixed_t)( (int64_t)_axis.y * KF_FIXED_SCALE / axisLength ), (kf_fixed_t)( (int64_t)_axis.z * KF_FIXED_SCALE / axisLength ) };
    _separation->feature = _feature;
    _separation->witness = KF_FALSE;
    return KF_TRUE;
}

/* Keep the parameter in Q32 until the point is reconstructed: a Q16 parameter
   can move a point on a long core by more than a thin shape's whole radius. */
static uint64_t kf_geometry_segment_fraction( kf_vec3_t _first, kf_vec3_t _second, kf_vec3_t _point )
{
    kf_vec3_t edge = kf_vec3_sub( _second, _first );
    int64_t square = kf_geometry_dot( edge, edge );
    int64_t projection;
    if( square == 0 )
    {
        return 0;
    }
    projection = kf_geometry_dot( kf_vec3_sub( _point, _first ), edge );
    if( projection <= 0 )
    {
        return 0;
    }
    if( projection >= square )
    {
        return UINT64_C(1) << 32;
    }
    return kf_geometry_ratio( (uint64_t)projection, (uint64_t)square, 32 );
}

static kf_vec3_t kf_geometry_segment_at( kf_vec3_t _first, kf_vec3_t _second, uint64_t _fraction )
{
    kf_vec3_t edge = kf_vec3_sub( _second, _first );
    return (kf_vec3_t){ _first.x + (kf_fixed_t)kf_internal_mul_shift( edge.x, (int64_t)_fraction, 32 ), _first.y + (kf_fixed_t)kf_internal_mul_shift( edge.y, (int64_t)_fraction, 32 ), _first.z + (kf_fixed_t)kf_internal_mul_shift( edge.z, (int64_t)_fraction, 32 ) };
}

static kf_vec3_t kf_geometry_segment_point( kf_vec3_t _first, kf_vec3_t _second, kf_vec3_t _point )
{
    kf_vec3_t edge = kf_vec3_sub( _second, _first );
    uint64_t fraction;
    /* Axis-aligned segments clamp the coordinate exactly. */
    if( edge.x == 0 && edge.z == 0 )
    {
        return (kf_vec3_t){ _first.x, kf_fixed_clamp( _point.y, kf_fixed_min( _first.y, _second.y ), kf_fixed_max( _first.y, _second.y ) ), _first.z };
    }
    if( edge.y == 0 && edge.z == 0 )
    {
        return (kf_vec3_t){ kf_fixed_clamp( _point.x, kf_fixed_min( _first.x, _second.x ), kf_fixed_max( _first.x, _second.x ) ), _first.y, _first.z };
    }
    if( edge.x == 0 && edge.y == 0 )
    {
        return (kf_vec3_t){ _first.x, _first.y, kf_fixed_clamp( _point.z, kf_fixed_min( _first.z, _second.z ), kf_fixed_max( _first.z, _second.z ) ) };
    }
    fraction = kf_geometry_segment_fraction( _first, _second, _point );
    if( fraction == 0 )
    {
        return _first;
    }
    if( fraction == ( UINT64_C(1) << 32 ) )
    {
        return _second;
    }
    return kf_geometry_segment_at( _first, _second, fraction );
}

/* Exact signed product difference, represented by a sign and magnitude. */
static kf_wide_t kf_geometry_product_difference( int64_t _a, int64_t _b, int64_t _c, int64_t _d, kf_bool_t * _negative )
{
    kf_wide_t difference = kf_internal_wide_multiply( kf_internal_magnitude( _a ), kf_internal_magnitude( _b ) );
    *_negative = (kf_bool_t)( ( _a < 0 ) != ( _b < 0 ) );
    /* a * b - c * d adds c * d with the opposite sign. */
    kf_internal_wide_accumulate( &difference, _negative, kf_internal_wide_multiply( kf_internal_magnitude( _c ), kf_internal_magnitude( _d ) ), (kf_bool_t)( ( _c < 0 ) == ( _d < 0 ) ) );
    return difference;
}

/* Q32 ratio in [0,1]. Geometry products have at most 122 bits, leaving room
   to double each remainder without discarding any near-parallel precision. */
static uint64_t kf_geometry_wide_fraction( kf_wide_t _numerator, kf_wide_t _denominator )
{
    uint64_t result = 0;
    uint64_t high;
    uint64_t low;
    uint32_t bit;
    if( kf_internal_wide_compare( _numerator, _denominator ) >= 0 )
    {
        return UINT64_C(1) << 32;
    }
    /* One exact division when numerator << 32 fits in 128 bits; the doubling
       loop below gives the same floor for wider operands. */
    kf_internal_wide_parts( _numerator, &high, &low );
    if( ( high >> 32 ) == 0 )
    {
        return kf_internal_wide_quotient( kf_internal_wide_make( ( high << 32 ) | ( low >> 32 ), low << 32 ), _denominator );
    }
    for( bit = 0; bit != 32u; ++bit )
    {
        _numerator = kf_internal_wide_add( _numerator, _numerator );
        result <<= 1;
        if( kf_internal_wide_compare( _numerator, _denominator ) < 0 )
        {
            continue;
        }
        _numerator = kf_internal_wide_subtract( _numerator, _denominator );
        result |= UINT64_C(1);
    }
    return result;
}

/* A bounded minimum is either an interior stationary pair or a segment
   endpoint projected onto the other segment. Retain the exact normal equations
   until division: a shared scale erases short cores and small crossing angles. */
static void kf_geometry_segment_pair( kf_vec3_t _firstStart, kf_vec3_t _firstEnd, kf_vec3_t _secondStart, kf_vec3_t _secondEnd, kf_vec3_t * _first, kf_vec3_t * _second )
{
    kf_vec3_t firstEdge = kf_vec3_sub( _firstEnd, _firstStart );
    kf_vec3_t secondEdge = kf_vec3_sub( _secondEnd, _secondStart );
    kf_vec3_t offset = kf_vec3_sub( _firstStart, _secondStart );
    int64_t a = kf_geometry_dot( firstEdge, firstEdge );
    int64_t b = kf_geometry_dot( firstEdge, secondEdge );
    int64_t c = kf_geometry_dot( firstEdge, offset );
    int64_t e = kf_geometry_dot( secondEdge, secondEdge );
    int64_t f = kf_geometry_dot( secondEdge, offset );
    kf_bool_t negative;
    kf_wide_t zero = kf_internal_wide_multiply( 0, 0 );
    kf_wide_t denominator;
    int32_t side = -1; /* stationary first parameter below 0, or parallel cores: -1; above 1: 1 */
    if( a == 0 )
    {
        *_first = _firstStart;
        *_second = kf_geometry_segment_point( _secondStart, _secondEnd, _firstStart );
        return;
    }
    if( e == 0 )
    {
        *_second = _secondStart;
        *_first = kf_geometry_segment_point( _firstStart, _firstEnd, _secondStart );
        return;
    }
    denominator = kf_geometry_product_difference( a, e, b, b, &negative );
    if( kf_internal_wide_compare( denominator, zero ) != 0 )
    {
        kf_bool_t firstNegative;
        kf_bool_t secondNegative;
        kf_wide_t firstNumerator = kf_geometry_product_difference( b, f, c, e, &firstNegative );
        kf_wide_t secondNumerator = kf_geometry_product_difference( a, f, b, c, &secondNegative );
        kf_bool_t firstBelow = (kf_bool_t)( firstNegative == KF_TRUE && kf_internal_wide_compare( firstNumerator, zero ) != 0 );
        kf_bool_t secondBelow = (kf_bool_t)( secondNegative == KF_TRUE && kf_internal_wide_compare( secondNumerator, zero ) != 0 );
        kf_bool_t firstAbove = (kf_bool_t)( firstBelow == KF_FALSE && kf_internal_wide_compare( firstNumerator, denominator ) > 0 );
        kf_bool_t secondAbove = (kf_bool_t)( secondBelow == KF_FALSE && kf_internal_wide_compare( secondNumerator, denominator ) > 0 );
        if( firstBelow == KF_FALSE && firstAbove == KF_FALSE )
        {
            if( secondBelow == KF_FALSE && secondAbove == KF_FALSE )
            {
                uint64_t firstFraction = kf_geometry_wide_fraction( firstNumerator, denominator );
                uint64_t secondFraction = kf_geometry_wide_fraction( secondNumerator, denominator );
                *_first = kf_geometry_segment_at( _firstStart, _firstEnd, firstFraction );
                *_second = kf_geometry_segment_at( _secondStart, _secondEnd, secondFraction );
                return;
            }
            /* The stationary first point is inside: the minimum lies on the
               second segment's clamped end. */
            *_second = secondBelow == KF_TRUE ? _secondStart : _secondEnd;
            *_first = kf_geometry_segment_point( _firstStart, _firstEnd, *_second );
            return;
        }
        side = firstBelow == KF_TRUE ? -1 : 1;
    }
    /* Clamp the first parameter, project onto the second segment, and clamp
       that parameter in turn (Ericson's closest points of two segments). The
       exact signs above select the edge of the parameter square. */
    {
        int64_t projection = side < 0 ? f : kf_internal_add_saturated( b, f );
        kf_vec3_t first = side < 0 ? _firstStart : _firstEnd;
        if( projection >= 0 && projection <= e )
        {
            *_first = first;
            *_second = kf_geometry_segment_point( _secondStart, _secondEnd, first );
            return;
        }
        *_second = projection < 0 ? _secondStart : _secondEnd;
        *_first = kf_geometry_segment_point( _firstStart, _firstEnd, *_second );
    }
}

static void kf_geometry_round_witness( const kf_geometry_t * _round, const kf_geometry_t * _left, kf_vec3_t _core, kf_vec3_t _poly, kf_geometry_separation_t * _separation )
{
    _separation->witness = KF_TRUE;
    _separation->pointLeft = _round == _left ? _core : _poly;
    _separation->pointRight = _round == _left ? _poly : _core;
}

/* A rounded nearest point is a witness, not an exact supporting direction.
   Construct interior-feature axes from their lines before rounding witnesses;
   a one-raw tangential residual otherwise grows with a long edge's length. */
static kf_vec3_t kf_geometry_feature_axis( kf_vec3_t _firstStart, kf_vec3_t _firstEnd, kf_vec3_t _secondStart, kf_vec3_t _secondEnd, kf_vec3_t _first, kf_vec3_t _second )
{
    kf_vec3_t firstEdge = kf_vec3_sub( _firstEnd, _firstStart );
    kf_vec3_t secondEdge = kf_vec3_sub( _secondEnd, _secondStart );
    kf_bool_t firstInterior = (kf_bool_t)( kf_vec3_equal( _first, _firstStart ) == KF_FALSE && kf_vec3_equal( _first, _firstEnd ) == KF_FALSE );
    kf_bool_t secondInterior = (kf_bool_t)( kf_vec3_equal( _second, _secondStart ) == KF_FALSE && kf_vec3_equal( _second, _secondEnd ) == KF_FALSE );
    if( firstInterior == KF_TRUE && secondInterior == KF_TRUE )
    {
        kf_vec3_t axis = kf_geometry_cross_plane( firstEdge, secondEdge );
        if( axis.x != 0 || axis.y != 0 || axis.z != 0 )
        {
            return axis;
        }
    }
    if( firstInterior == KF_TRUE )
    {
        return kf_geometry_cross_plane( firstEdge, kf_geometry_cross_plane( kf_vec3_sub( _second, _firstStart ), firstEdge ) );
    }
    if( secondInterior == KF_TRUE )
    {
        return kf_geometry_cross_plane( secondEdge, kf_geometry_cross_plane( kf_vec3_sub( _first, _secondStart ), secondEdge ) );
    }
    return kf_vec3_sub( _first, _second );
}

static void kf_geometry_round_axes( const kf_geometry_t * _round, const kf_geometry_t * _poly, const kf_geometry_t * _left, const kf_geometry_t * _right, kf_geometry_separation_t * _separation )
{
    uint32_t index;
    for( index = 0; index != _poly->vertexCount; ++index )
    {
        kf_vec3_t vertex = _poly->vertices[index];
        kf_vec3_t nearest = kf_geometry_segment_point( _round->vertices[0], _round->vertices[1], vertex );
        if( kf_geometry_test_axis( _left, _right, kf_geometry_feature_axis( _round->vertices[0], _round->vertices[1], vertex, vertex, nearest, vertex ), KF_FEATURE_OTHER, _separation ) == KF_FALSE )
        {
            continue;
        }
        kf_geometry_round_witness( _round, _left, nearest, vertex, _separation );
    }
    for( index = 0; index != _poly->edgeCount; ++index )
    {
        kf_vec3_t first = _poly->vertices[_poly->edges[index].first];
        kf_vec3_t second = _poly->vertices[_poly->edges[index].second];
        kf_vec3_t roundPoint;
        kf_vec3_t polyPoint;
        kf_geometry_segment_pair( _round->vertices[0], _round->vertices[1], first, second, &roundPoint, &polyPoint );
        if( kf_geometry_test_axis( _left, _right, kf_geometry_feature_axis( _round->vertices[0], _round->vertices[1], first, second, roundPoint, polyPoint ), KF_FEATURE_OTHER, _separation ) == KF_FALSE )
        {
            continue;
        }
        kf_geometry_round_witness( _round, _left, roundPoint, polyPoint, _separation );
    }
}

/* Exact separation of a sphere, or a capsule parallel to a box axis, from a box,
   computed in the box frame as the distance from a point to the box extended by
   the core segment. Other capsule orientations use the general axes. */
static kf_bool_t kf_geometry_round_box( const kf_geometry_t * _round, const kf_geometry_t * _box, kf_bool_t _roundIsLeft, kf_geometry_separation_t * _separation )
{
    int64_t point[3];
    int64_t delta[3];
    int64_t minimum[3] = { _box->boxMinimum.x, _box->boxMinimum.y, _box->boxMinimum.z };
    int64_t maximum[3] = { _box->boxMaximum.x, _box->boxMaximum.y, _box->boxMaximum.z };
    int64_t difference[3];
    uint32_t moving = 3;
    uint32_t outside = 0;
    uint32_t axis;
    kf_vec3_t local;
    kf_vec3_t normal;
    kf_vec3_t first = kf_internal_matrix_apply_transpose( &_box->boxRotation, kf_vec3_sub( _round->vertices[0], _box->boxPosition ) );
    kf_vec3_t second = kf_internal_matrix_apply_transpose( &_box->boxRotation, kf_vec3_sub( _round->vertices[1], _box->boxPosition ) );
    kf_fixed_t coordinates[3] = { first.x, first.y, first.z };
    kf_fixed_t endpoints[3] = { second.x, second.y, second.z };
    for( axis = 0; axis != 3u; ++axis )
    {
        point[axis] = coordinates[axis];
        delta[axis] = (int64_t)endpoints[axis] - coordinates[axis];
        if( delta[axis] == 0 )
        {
            continue;
        }
        if( moving != 3u )
        {
            return KF_FALSE;
        }
        moving = axis;
    }
    if( moving != 3u )
    {
        /* Points p with p + t d in the box for some t in [0, 1]. */
        if( delta[moving] > 0 )
        {
            minimum[moving] -= delta[moving];
        }
        else
        {
            maximum[moving] -= delta[moving];
        }
    }
    for( axis = 0; axis != 3u; ++axis )
    {
        int64_t nearest = point[axis] < minimum[axis] ? minimum[axis] : ( point[axis] > maximum[axis] ? maximum[axis] : point[axis] );
        difference[axis] = point[axis] - nearest;
        outside += difference[axis] != 0 ? 1u : 0u;
    }
    if( outside != 0 )
    {
        local = (kf_vec3_t){ (kf_fixed_t)difference[0], (kf_fixed_t)difference[1], (kf_fixed_t)difference[2] };
        _separation->distance = kf_geometry_length( local ) - _round->radius;
        local = kf_geometry_normalize( local );
    }
    else
    {
        int64_t best = point[0] - minimum[0];
        local = (kf_vec3_t){ -KF_FIXED_SCALE, 0, 0 };
        for( axis = 0; axis != 3u; ++axis )
        {
            if( point[axis] - minimum[axis] < best )
            {
                best = point[axis] - minimum[axis];
                local = (kf_vec3_t){ axis == 0 ? -KF_FIXED_SCALE : 0, axis == 1 ? -KF_FIXED_SCALE : 0, axis == 2 ? -KF_FIXED_SCALE : 0 };
            }
            if( maximum[axis] - point[axis] < best )
            {
                best = maximum[axis] - point[axis];
                local = (kf_vec3_t){ axis == 0 ? KF_FIXED_SCALE : 0, axis == 1 ? KF_FIXED_SCALE : 0, axis == 2 ? KF_FIXED_SCALE : 0 };
            }
        }
        _separation->distance = (kf_fixed_t)( -best ) - _round->radius;
        outside = 1;
    }
    normal = kf_internal_matrix_apply( &_box->boxRotation, local );
    _separation->normal = _roundIsLeft == KF_TRUE ? normal : kf_vec3_neg( normal );
    _separation->initialized = KF_TRUE;
    _separation->feature = KF_FEATURE_OTHER;
    if( outside == 1u )
    {
        _separation->feature = _roundIsLeft == KF_TRUE ? KF_FEATURE_RIGHT_FACE : KF_FEATURE_LEFT_FACE;
    }
    return KF_TRUE;
}

static kf_geometry_separation_t kf_geometry_prefer( kf_geometry_separation_t _face, kf_geometry_separation_t _other )
{
    if( _other.initialized == KF_FALSE )
    {
        return _face;
    }
    if( _face.initialized == KF_FALSE )
    {
        return _other;
    }
    /* A preferred face cannot hide a positive separating gap, including a
       gap inside the contact tolerance but outside a caller's zero margin. */
    if( _other.distance > 0 )
    {
        if( _other.distance > _face.distance )
        {
            return _other;
        }
    }
    if( (int64_t)_other.distance > (int64_t)_face.distance + KF_GEOMETRY_FEATURE_TOLERANCE )
    {
        return _other;
    }
    return _face;
}

static kf_geometry_separation_t kf_geometry_separate( const kf_geometry_t * _left, const kf_geometry_t * _right )
{
    kf_geometry_separation_t face = { 0 };
    kf_geometry_separation_t other = { 0 };
    uint32_t index;
    uint32_t rightIndex;
    if( _left->rounded == KF_TRUE && _right->rounded == KF_TRUE )
    {
        kf_vec3_t pointLeft;
        kf_vec3_t pointRight;
        kf_vec3_t difference;
        kf_fixed_t coreDistance;
        kf_geometry_segment_pair( _left->vertices[0], _left->vertices[1], _right->vertices[0], _right->vertices[1], &pointLeft, &pointRight );
        difference = kf_vec3_sub( pointLeft, pointRight );
        coreDistance = kf_geometry_length( difference );
        other.normal = kf_geometry_normalize( difference );
        /* At a core crossing, integer point reconstruction can leave a tiny
           residual along either core. It is not a valid escape direction. */
        if( coreDistance <= KF_GEOMETRY_TOLERANCE )
        {
            kf_vec3_t cross = kf_geometry_cross_normal( kf_vec3_sub( _left->vertices[1], _left->vertices[0] ), kf_vec3_sub( _right->vertices[1], _right->vertices[0] ) );
            if( cross.x != 0 || cross.y != 0 || cross.z != 0 )
            {
                int64_t projection = kf_geometry_dot( difference, cross ) / KF_FIXED_SCALE;
                other.normal = projection < 0 ? kf_vec3_neg( cross ) : cross;
                coreDistance = (kf_fixed_t)kf_internal_magnitude( projection );
            }
        }
        if( other.normal.x == 0 && other.normal.y == 0 && other.normal.z == 0 )
        {
            kf_vec3_t leftEdge = kf_vec3_sub( _left->vertices[1], _left->vertices[0] );
            kf_vec3_t rightEdge = kf_vec3_sub( _right->vertices[1], _right->vertices[0] );
            other.normal = kf_geometry_cross_normal( leftEdge, rightEdge );
            if( other.normal.x == 0 && other.normal.y == 0 && other.normal.z == 0 )
            {
                kf_vec3_t edge = kf_geometry_dot( leftEdge, leftEdge ) != 0 ? leftEdge : rightEdge;
                kf_vec3_t axis = { KF_FIXED_SCALE, 0, 0 };
                if( kf_internal_magnitude( edge.y ) < kf_internal_magnitude( edge.x ) )
                {
                    axis = (kf_vec3_t){ 0, KF_FIXED_SCALE, 0 };
                }
                if( kf_internal_magnitude( edge.z ) < kf_internal_magnitude( edge.x ) && kf_internal_magnitude( edge.z ) < kf_internal_magnitude( edge.y ) )
                {
                    axis = (kf_vec3_t){ 0, 0, KF_FIXED_SCALE };
                }
                other.normal = kf_geometry_cross_normal( edge, axis );
                if( other.normal.x == 0 && other.normal.y == 0 && other.normal.z == 0 )
                {
                    other.normal.x = KF_FIXED_SCALE;
                }
            }
        }
        other.distance = coreDistance - _left->radius - _right->radius;
        other.initialized = KF_TRUE;
        return other;
    }
    if( _left->rounded == KF_TRUE && _right->box == KF_TRUE && kf_geometry_round_box( _left, _right, KF_TRUE, &other ) == KF_TRUE )
    {
        return other;
    }
    if( _right->rounded == KF_TRUE && _left->box == KF_TRUE && kf_geometry_round_box( _right, _left, KF_FALSE, &other ) == KF_TRUE )
    {
        return other;
    }
    for( index = 0; index != _left->axisCount; ++index )
    {
        kf_geometry_test_axis( _left, _right, _left->axes[index], KF_FEATURE_LEFT_FACE, &face );
    }
    for( index = 0; index != _right->axisCount; ++index )
    {
        kf_geometry_test_axis( _left, _right, _right->axes[index], KF_FEATURE_RIGHT_FACE, &face );
    }
    if( _left->rounded == KF_TRUE )
    {
        kf_geometry_round_axes( _left, _right, _left, _right, &other );
    }
    else if( _right->rounded == KF_TRUE )
    {
        kf_geometry_round_axes( _right, _left, _left, _right, &other );
    }
    else
    {
        for( index = 0; index != _left->directionCount; ++index )
        {
            for( rightIndex = 0; rightIndex != _right->directionCount; ++rightIndex )
            {
                kf_geometry_test_axis( _left, _right, kf_geometry_cross_plane( _left->directions[index], _right->directions[rightIndex] ), KF_FEATURE_OTHER, &other );
            }
        }
    }
    /* A zero-thickness surface also separates along its in-plane edge normals. */
    if( _left->surface == KF_TRUE )
    {
        for( index = 0; index != _left->directionCount; ++index )
        {
            kf_geometry_test_axis( _left, _right, kf_geometry_cross_plane( _left->axes[0], _left->directions[index] ), KF_FEATURE_OTHER, &other );
        }
    }
    if( _right->surface == KF_TRUE )
    {
        for( index = 0; index != _right->directionCount; ++index )
        {
            kf_geometry_test_axis( _left, _right, kf_geometry_cross_plane( _right->axes[0], _right->directions[index] ), KF_FEATURE_OTHER, &other );
        }
    }
    return kf_geometry_prefer( face, other );
}

/* Average of vertices within one raw Q32 unit of the support plane. */
static kf_vec3_t kf_geometry_support( const kf_geometry_t * _geometry, kf_vec3_t _direction )
{
    int64_t selectedProjection = kf_geometry_dot( _geometry->vertices[0], _direction );
    int64_t x = 0;
    int64_t y = 0;
    int64_t z = 0;
    uint32_t count = 0;
    uint32_t index;
    for( index = 1; index != _geometry->vertexCount; ++index )
    {
        int64_t projection = kf_geometry_dot( _geometry->vertices[index], _direction );
        if( projection > selectedProjection )
        {
            selectedProjection = projection;
        }
    }
    for( index = 0; index != _geometry->vertexCount; ++index )
    {
        int64_t projection = kf_geometry_dot( _geometry->vertices[index], _direction );
        if( selectedProjection - projection > KF_FIXED_SCALE )
        {
            continue;
        }
        x += _geometry->vertices[index].x;
        y += _geometry->vertices[index].y;
        z += _geometry->vertices[index].z;
        ++count;
    }
    return (kf_vec3_t){ (kf_fixed_t)( x / count ), (kf_fixed_t)( y / count ), (kf_fixed_t)( z / count ) };
}

/* Polytope edge whose endpoints reach farthest along a direction; ties keep the first. */
static void kf_geometry_support_edge( const kf_geometry_t * _geometry, kf_vec3_t _direction, kf_vec3_t * _first, kf_vec3_t * _second )
{
    int64_t best = 0;
    uint32_t index;
    *_first = kf_geometry_support( _geometry, _direction );
    *_second = *_first;
    for( index = 0; index != _geometry->edgeCount; ++index )
    {
        kf_vec3_t first = _geometry->vertices[_geometry->edges[index].first];
        kf_vec3_t second = _geometry->vertices[_geometry->edges[index].second];
        int64_t projection = kf_geometry_dot( first, _direction ) + kf_geometry_dot( second, _direction );
        if( index != 0 && projection <= best )
        {
            continue;
        }
        best = projection;
        *_first = first;
        *_second = second;
    }
}

static void kf_geometry_add_point( kf_manifold_t * _manifold, kf_vec3_t _left, kf_vec3_t _right, kf_fixed_t _separation )
{
    kf_manifold_point_t * point;
    if( _manifold->count == KF_MANIFOLD_MAX_POINTS )
    {
        return;
    }
    point = _manifold->points + _manifold->count++;
    point->point_left = _left;
    point->point_right = _right;
    point->separation = _separation;
}

/* Core endpoint reaching farthest along a direction; a core perpendicular to it
   uses its point nearest to the other shape's support. */
static kf_vec3_t kf_geometry_core_support( const kf_geometry_t * _round, kf_vec3_t _direction, kf_vec3_t _target )
{
    int64_t first = kf_geometry_dot( _round->vertices[0], _direction );
    int64_t second = kf_geometry_dot( _round->vertices[1], _direction );
    if( first - second > (int64_t)KF_GEOMETRY_TOLERANCE * KF_FIXED_SCALE )
    {
        return _round->vertices[0];
    }
    if( second - first > (int64_t)KF_GEOMETRY_TOLERANCE * KF_FIXED_SCALE )
    {
        return _round->vertices[1];
    }
    return kf_geometry_segment_point( _round->vertices[0], _round->vertices[1], _target );
}

static kf_vec3_t kf_geometry_face_point( const kf_geometry_t * _poly, kf_vec3_t _direction, kf_vec3_t _point );

/* Edge and vertex separation retain the actual closest feature witnesses. */
static void kf_geometry_single_point( const kf_geometry_t * _left, const kf_geometry_t * _right, const kf_geometry_separation_t * _separation, kf_manifold_t * _manifold )
{
    kf_vec3_t normal = _separation->normal;
    kf_vec3_t pointLeft;
    kf_vec3_t pointRight;
    if( _separation->witness == KF_TRUE )
    {
        pointLeft = kf_geometry_offset( _separation->pointLeft, normal, -(int64_t)_left->radius );
        pointRight = kf_geometry_offset( _separation->pointRight, normal, _right->radius );
        kf_geometry_add_point( _manifold, pointLeft, pointRight, _separation->distance );
        return;
    }
    if( _left->rounded == KF_FALSE && _right->rounded == KF_FALSE )
    {
        kf_vec3_t leftFirst;
        kf_vec3_t leftSecond;
        kf_vec3_t rightFirst;
        kf_vec3_t rightSecond;
        kf_geometry_support_edge( _left, kf_vec3_neg( normal ), &leftFirst, &leftSecond );
        kf_geometry_support_edge( _right, normal, &rightFirst, &rightSecond );
        kf_geometry_segment_pair( leftFirst, leftSecond, rightFirst, rightSecond, &pointLeft, &pointRight );
        kf_geometry_add_point( _manifold, pointLeft, pointRight, (kf_fixed_t)( kf_geometry_dot( kf_vec3_sub( pointLeft, pointRight ), normal ) / KF_FIXED_SCALE ) );
        return;
    }
    if( _left->rounded == KF_TRUE )
    {
        kf_vec3_t core = kf_geometry_core_support( _left, kf_vec3_neg( normal ), kf_geometry_support( _right, normal ) );
        pointLeft = kf_geometry_offset( core, normal, -(int64_t)_left->radius );
        pointRight = kf_geometry_face_point( _right, normal, kf_geometry_offset( pointLeft, normal, -(int64_t)_separation->distance ) );
    }
    else
    {
        kf_vec3_t core = kf_geometry_core_support( _right, normal, kf_geometry_support( _left, kf_vec3_neg( normal ) ) );
        pointRight = kf_geometry_offset( core, normal, _right->radius );
        pointLeft = kf_geometry_face_point( _left, kf_vec3_neg( normal ), kf_geometry_offset( pointRight, normal, _separation->distance ) );
    }
    kf_geometry_add_point( _manifold, pointLeft, pointRight, _separation->distance );
}

/* An endpoint's closest direction to the other core need not be perpendicular
   to its own core. Reproject the witness onto its own rounded surface. */
static kf_vec3_t kf_geometry_round_surface( const kf_geometry_t * _round, kf_vec3_t _point )
{
    kf_vec3_t core = kf_geometry_segment_point( _round->vertices[0], _round->vertices[1], _point );
    kf_vec3_t direction = kf_vec3_sub( _point, core );
    kf_fixed_t length = kf_geometry_length( direction );
    if( length == 0 )
    {
        return core;
    }
    return (kf_vec3_t){
        core.x + (kf_fixed_t)( (int64_t)direction.x * _round->radius / length ),
        core.y + (kf_fixed_t)( (int64_t)direction.y * _round->radius / length ),
        core.z + (kf_fixed_t)( (int64_t)direction.z * _round->radius / length ) };
}

static void kf_geometry_rounded_pair( const kf_geometry_t * _left, const kf_geometry_t * _right, const kf_geometry_separation_t * _separation, kf_fixed_t _margin, kf_manifold_t * _manifold )
{
    kf_vec3_t normal = _separation->normal;
    kf_vec3_t leftEdge = kf_vec3_sub( _left->vertices[1], _left->vertices[0] );
    kf_vec3_t rightEdge = kf_vec3_sub( _right->vertices[1], _right->vertices[0] );
    int64_t leftSquare = kf_geometry_dot( leftEdge, leftEdge );
    int64_t rightSquare = kf_geometry_dot( rightEdge, rightEdge );
    if( leftSquare != 0 && rightSquare != 0 )
    {
        kf_vec3_t cross = kf_geometry_cross_normal( leftEdge, rightEdge );
        int64_t alignment = kf_geometry_dot( kf_geometry_normalize( leftEdge ), kf_geometry_normalize( rightEdge ) );
        int64_t parallelAlignment = INT64_C(4294311361); /* cos(1 degree) in Q32 */
        /* Nearly parallel resting cores can keep both ends of their overlap,
           provided their closest directions agree with the shared normal. */
        if( ( cross.x == 0 && cross.y == 0 && cross.z == 0 ) || alignment > parallelAlignment || alignment < -parallelAlignment )
        {
            uint64_t start = kf_geometry_segment_fraction( _left->vertices[0], _left->vertices[1], _right->vertices[0] );
            uint64_t end = kf_geometry_segment_fraction( _left->vertices[0], _left->vertices[1], _right->vertices[1] );
            uint64_t low = start < end ? start : end;
            uint64_t high = start > end ? start : end;
            if( high - low > ( (uint64_t)KF_GEOMETRY_TOLERANCE << 16 ) )
            {
                uint64_t fractions[2] = { low, high };
                kf_vec3_t pointsLeft[2];
                kf_vec3_t pointsRight[2];
                kf_fixed_t separations[2];
                uint32_t index;
                for( index = 0; index != 2u; ++index )
                {
                    kf_vec3_t core = kf_geometry_segment_at( _left->vertices[0], _left->vertices[1], fractions[index] );
                    kf_vec3_t other = kf_geometry_segment_point( _right->vertices[0], _right->vertices[1], core );
                    kf_vec3_t difference = kf_vec3_sub( core, other );
                    kf_fixed_t distance = kf_geometry_length( difference );
                    kf_vec3_t direction = distance == 0 ? normal : kf_geometry_normalize( difference );
                    if( kf_geometry_dot( direction, normal ) < parallelAlignment )
                    {
                        break;
                    }
                    pointsLeft[index] = kf_geometry_offset( core, direction, -(int64_t)_left->radius );
                    pointsRight[index] = kf_geometry_offset( other, direction, _right->radius );
                    pointsLeft[index] = kf_geometry_round_surface( _left, pointsLeft[index] );
                    pointsRight[index] = kf_geometry_round_surface( _right, pointsRight[index] );
                    difference = kf_vec3_sub( pointsLeft[index], pointsRight[index] );
                    separations[index] = (kf_fixed_t)( kf_geometry_dot( difference, normal ) / KF_FIXED_SCALE );
                    if( separations[index] > _margin )
                    {
                        break;
                    }
                }
                if( index == 2u )
                {
                    kf_geometry_add_point( _manifold, pointsLeft[0], pointsRight[0], separations[0] );
                    kf_geometry_add_point( _manifold, pointsLeft[1], pointsRight[1], separations[1] );
                    return;
                }
            }
        }
    }
    {
        kf_vec3_t pointLeft;
        kf_vec3_t pointRight;
        kf_geometry_segment_pair( _left->vertices[0], _left->vertices[1], _right->vertices[0], _right->vertices[1], &pointLeft, &pointRight );
        kf_geometry_add_point( _manifold, kf_geometry_offset( pointLeft, normal, -(int64_t)_left->radius ), kf_geometry_offset( pointRight, normal, _right->radius ), _separation->distance );
    }
}

/* Reference face whose outward normal is most aligned with a direction. */
static const kf_geometry_face_t * kf_geometry_face( const kf_geometry_t * _geometry, kf_vec3_t _direction, kf_bool_t _aligned )
{
    const kf_geometry_face_t * selected = NULL;
    int64_t best = 0;
    uint32_t index;
    for( index = 0; index != _geometry->faceCount; ++index )
    {
        const kf_geometry_face_t * face = _geometry->faces + index;
        int64_t projection = kf_geometry_dot( face->normal, _direction );
        if( _aligned == KF_FALSE )
        {
            projection = -projection;
        }
        if( selected != NULL && projection <= best )
        {
            continue;
        }
        selected = face;
        best = projection;
    }
    return selected;
}

/* Outward side plane through an edge of a face polygon. */
static kf_vec3_t kf_geometry_side_normal( const kf_geometry_t * _geometry, const kf_geometry_face_t * _face, uint32_t _index, kf_vec3_t _centroid )
{
    kf_vec3_t first = _geometry->vertices[_geometry->indices[_face->first + _index]];
    kf_vec3_t second = _geometry->vertices[_geometry->indices[_face->first + ( _index + 1u ) % _face->count]];
    kf_vec3_t side = kf_geometry_cross_plane( kf_vec3_sub( second, first ), _face->planeNormal );
    if( kf_geometry_dot( kf_vec3_sub( _centroid, first ), side ) > 0 )
    {
        side = kf_vec3_neg( side );
    }
    return side;
}

static kf_vec3_t kf_geometry_face_centroid( const kf_geometry_t * _geometry, const kf_geometry_face_t * _face )
{
    int64_t x = 0;
    int64_t y = 0;
    int64_t z = 0;
    uint32_t index;
    for( index = 0; index != _face->count; ++index )
    {
        kf_vec3_t vertex = _geometry->vertices[_geometry->indices[_face->first + index]];
        x += vertex.x;
        y += vertex.y;
        z += vertex.z;
    }
    return (kf_vec3_t){ (kf_fixed_t)( x / _face->count ), (kf_fixed_t)( y / _face->count ), (kf_fixed_t)( z / _face->count ) };
}

/* Signed distance to the retained face plane along the public unit normal.
   The rational projection keeps Q30 plane coefficients until the final scalar. */
static kf_fixed_t kf_geometry_face_height( const kf_geometry_t * _poly, const kf_geometry_face_t * _face, kf_vec3_t _point )
{
    kf_vec3_t origin = _poly->vertices[_poly->indices[_face->first]];
    int64_t numerator = kf_geometry_dot( kf_vec3_sub( _point, origin ), _face->planeNormal );
    int64_t denominator = kf_geometry_dot( _face->normal, _face->planeNormal );
    return (kf_fixed_t)kf_internal_mul_div( numerator, KF_FIXED_SCALE, denominator, 0 );
}

/* A box's surface is measured in the retained rigid frame, including witness
   projection. A Q16 output normal is not a sufficiently accurate frame at large
   extents, even when normalized. */
static kf_vec3_t kf_geometry_box_face_point( const kf_geometry_t * _box, kf_vec3_t _direction, kf_vec3_t _point, kf_fixed_t * _height )
{
    kf_vec3_t local = kf_internal_matrix_apply_transpose( &_box->boxRotation, kf_vec3_sub( _point, _box->boxPosition ) );
    kf_vec3_t direction = kf_internal_matrix_apply_transpose( &_box->boxRotation, _direction );
    kf_fixed_t coordinates[3] = { local.x, local.y, local.z };
    kf_fixed_t components[3] = { direction.x, direction.y, direction.z };
    kf_fixed_t minimum[3] = { _box->boxMinimum.x, _box->boxMinimum.y, _box->boxMinimum.z };
    kf_fixed_t maximum[3] = { _box->boxMaximum.x, _box->boxMaximum.y, _box->boxMaximum.z };
    uint32_t axis = 0;
    uint32_t index;
    for( index = 1; index != 3u; ++index )
    {
        if( kf_internal_magnitude( components[index] ) > kf_internal_magnitude( components[axis] ) )
        {
            axis = index;
        }
    }
    *_height = components[axis] < 0 ? minimum[axis] - coordinates[axis] : coordinates[axis] - maximum[axis];
    for( index = 0; index != 3u; ++index )
    {
        if( coordinates[index] < minimum[index] )
        {
            coordinates[index] = minimum[index];
        }
        if( coordinates[index] > maximum[index] )
        {
            coordinates[index] = maximum[index];
        }
    }
    coordinates[axis] = components[axis] < 0 ? minimum[axis] : maximum[axis];
    local = kf_internal_matrix_apply( &_box->boxRotation, (kf_vec3_t){ coordinates[0], coordinates[1], coordinates[2] } );
    return kf_vec3_add( _box->boxPosition, local );
}

/* A preferred face normal can win over a nearby edge within the feature
   tolerance. Its plane witness must still be clamped to the actual polygon. */
static kf_vec3_t kf_geometry_face_point( const kf_geometry_t * _poly, kf_vec3_t _direction, kf_vec3_t _point )
{
    const kf_geometry_face_t * face;
    kf_vec3_t centroid;
    kf_vec3_t origin;
    kf_vec3_t nearest;
    kf_vec3_t point;
    int64_t best = INT64_MAX;
    int64_t height;
    kf_bool_t inside = KF_TRUE;
    uint32_t index;
    if( _poly->box == KF_TRUE )
    {
        kf_fixed_t boxHeight;
        return kf_geometry_box_face_point( _poly, _direction, _point, &boxHeight );
    }
    face = kf_geometry_face( _poly, _direction, KF_TRUE );
    if( face == NULL )
    {
        return kf_geometry_support( _poly, _direction );
    }
    origin = _poly->vertices[_poly->indices[face->first]];
    height = kf_geometry_face_height( _poly, face, _point );
    point = kf_geometry_offset( _point, face->normal, -height );
    centroid = kf_geometry_face_centroid( _poly, face );
    nearest = origin;
    for( index = 0; index != face->count; ++index )
    {
        kf_vec3_t first = _poly->vertices[_poly->indices[face->first + index]];
        kf_vec3_t side = kf_geometry_side_normal( _poly, face, index, centroid );
        if( kf_geometry_dot( kf_vec3_sub( point, first ), side ) > 0 )
        {
            inside = KF_FALSE;
            break;
        }
    }
    if( inside == KF_TRUE )
    {
        return point;
    }
    for( index = 0; index != face->count; ++index )
    {
        kf_vec3_t first = _poly->vertices[_poly->indices[face->first + index]];
        kf_vec3_t second = _poly->vertices[_poly->indices[face->first + ( index + 1u ) % face->count]];
        kf_vec3_t candidate = kf_geometry_segment_point( first, second, point );
        kf_vec3_t difference = kf_vec3_sub( candidate, point );
        int64_t square = kf_geometry_dot( difference, difference );
        if( square < best )
        {
            best = square;
            nearest = candidate;
        }
    }
    return nearest;
}

static kf_vec3_t kf_geometry_clip_point( kf_vec3_t _first, kf_vec3_t _second, int64_t _firstDistance, int64_t _secondDistance )
{
    uint64_t fraction = kf_geometry_ratio( kf_internal_magnitude( _firstDistance ), kf_internal_magnitude( _firstDistance - _secondDistance ), 32 );
    return kf_geometry_segment_at( _first, _second, fraction );
}

/* Sutherland-Hodgman clip keeping dot(point - origin, normal) <= 0. */
static uint32_t kf_geometry_clip( const kf_vec3_t * _input, uint32_t _count, kf_vec3_t _origin, kf_vec3_t _normal, kf_vec3_t * _output )
{
    uint32_t output = 0;
    uint32_t index;
    for( index = 0; index != _count; ++index )
    {
        kf_vec3_t first = _input[index];
        kf_vec3_t second = _input[( index + 1u ) % _count];
        int64_t firstDistance = kf_geometry_dot( kf_vec3_sub( first, _origin ), _normal );
        int64_t secondDistance = kf_geometry_dot( kf_vec3_sub( second, _origin ), _normal );
        if( firstDistance <= 0 && output != KF_GEOMETRY_MAX_CLIP )
        {
            _output[output++] = first;
        }
        if( ( firstDistance < 0 && secondDistance > 0 ) || ( firstDistance > 0 && secondDistance < 0 ) )
        {
            if( output != KF_GEOMETRY_MAX_CLIP )
            {
                _output[output++] = kf_geometry_clip_point( first, second, firstDistance, secondDistance );
            }
        }
    }
    return output;
}

/* Keeps the deepest point, the farthest point from it, and the two points
   spanning the largest areas on opposite sides of their diagonal. */
static uint32_t kf_geometry_reduce( kf_vec3_t * _points, kf_fixed_t * _separations, uint32_t _count, kf_vec3_t _normal )
{
    uint32_t selected[KF_MANIFOLD_MAX_POINTS];
    kf_vec3_t points[KF_MANIFOLD_MAX_POINTS];
    kf_fixed_t separations[KF_MANIFOLD_MAX_POINTS];
    int64_t best;
    int64_t sideArea = 0;
    uint32_t axis = 0;
    uint32_t index;
    if( _count <= KF_MANIFOLD_MAX_POINTS )
    {
        return _count;
    }
    selected[0] = 0;
    for( index = 1; index != _count; ++index )
    {
        if( _separations[index] < _separations[selected[0]] )
        {
            selected[0] = index;
        }
    }
    selected[1] = selected[0];
    best = -1;
    for( index = 0; index != _count; ++index )
    {
        kf_vec3_t difference = kf_vec3_sub( _points[index], _points[selected[0]] );
        int64_t square = kf_geometry_dot( difference, difference );
        if( square > best )
        {
            best = square;
            selected[1] = index;
        }
    }
    if( kf_internal_magnitude( _normal.y ) > kf_internal_magnitude( _normal.x ) )
    {
        axis = 1;
    }
    if( kf_internal_magnitude( _normal.z ) > ( axis == 0 ? kf_internal_magnitude( _normal.x ) : kf_internal_magnitude( _normal.y ) ) )
    {
        axis = 2;
    }
    selected[2] = selected[0];
    selected[3] = selected[0];
    for( index = 0; index != _count; ++index )
    {
        kf_vec3_t diagonal = kf_vec3_sub( _points[selected[1]], _points[selected[0]] );
        kf_vec3_t offset = kf_vec3_sub( _points[index], _points[selected[0]] );
        /* A dominant-axis projection preserves the area ordering of a planar
           polygon. Raw Q32 determinants fit int64 throughout the shape range;
           converting their cross product to a Q16 vector would lose its area. */
        int64_t area = axis == 0 ? (int64_t)diagonal.y * offset.z - (int64_t)diagonal.z * offset.y
            : ( axis == 1 ? (int64_t)diagonal.z * offset.x - (int64_t)diagonal.x * offset.z : (int64_t)diagonal.x * offset.y - (int64_t)diagonal.y * offset.x );
        if( area > sideArea )
        {
            sideArea = area;
            selected[2] = index;
        }
        if( area < 0 && ( selected[3] == selected[0] || area < best ) )
        {
            best = area;
            selected[3] = index;
        }
    }
    for( index = 0; index != KF_MANIFOLD_MAX_POINTS; ++index )
    {
        points[index] = _points[selected[index]];
        separations[index] = _separations[selected[index]];
    }
    _count = 0;
    for( index = 0; index != KF_MANIFOLD_MAX_POINTS; ++index )
    {
        uint32_t previous;
        kf_bool_t duplicate = KF_FALSE;
        for( previous = 0; previous != index; ++previous )
        {
            if( selected[previous] == selected[index] )
            {
                duplicate = KF_TRUE;
            }
        }
        if( duplicate == KF_TRUE )
        {
            continue;
        }
        _points[_count] = points[index];
        _separations[_count] = separations[index];
        ++_count;
    }
    return _count;
}

/* Clips the incident face against a reference face. The reference normal points
   out of the reference shape toward the incident shape. */
static void kf_geometry_face_pair( const kf_geometry_t * _reference, const kf_geometry_t * _incident, kf_bool_t _referenceIsRight, kf_vec3_t _referenceDirection, kf_fixed_t _margin, kf_manifold_t * _manifold )
{
    const kf_geometry_face_t * reference = kf_geometry_face( _reference, _referenceDirection, KF_TRUE );
    const kf_geometry_face_t * incident;
    kf_vec3_t buffers[2][KF_GEOMETRY_MAX_CLIP];
    kf_fixed_t separations[KF_GEOMETRY_MAX_CLIP];
    kf_vec3_t centroid;
    uint32_t count;
    uint32_t current = 0;
    uint32_t kept = 0;
    uint32_t index;
    if( reference == NULL )
    {
        return;
    }
    incident = kf_geometry_face( _incident, reference->normal, KF_FALSE );
    if( incident == NULL )
    {
        return;
    }
    centroid = kf_geometry_face_centroid( _reference, reference );
    count = incident->count;
    for( index = 0; index != count; ++index )
    {
        buffers[0][index] = _incident->vertices[_incident->indices[incident->first + index]];
    }
    for( index = 0; index != reference->count && count != 0; ++index )
    {
        kf_vec3_t planePoint = _reference->vertices[_reference->indices[reference->first + index]];
        kf_vec3_t side = kf_geometry_side_normal( _reference, reference, index, centroid );
        count = kf_geometry_clip( buffers[current], count, planePoint, side, buffers[current ^ 1u] );
        current ^= 1u;
    }
    for( index = 0; index != count; ++index )
    {
        kf_vec3_t point = buffers[current][index];
        kf_fixed_t separation = kf_geometry_face_height( _reference, reference, point );
        if( _reference->box == KF_TRUE )
        {
            (void)kf_geometry_box_face_point( _reference, reference->normal, point, &separation );
        }
        if( separation > _margin )
        {
            continue;
        }
        buffers[current][kept] = point;
        separations[kept] = separation;
        ++kept;
    }
    kept = kf_geometry_reduce( buffers[current], separations, kept, incident->planeNormal );
    for( index = 0; index != kept; ++index )
    {
        kf_vec3_t point = buffers[current][index];
        kf_vec3_t projected = kf_geometry_offset( point, reference->normal, -(int64_t)separations[index] );
        if( _reference->box == KF_TRUE )
        {
            kf_fixed_t height;
            projected = kf_geometry_box_face_point( _reference, reference->normal, point, &height );
        }
        if( _referenceIsRight == KF_TRUE )
        {
            kf_geometry_add_point( _manifold, point, projected, separations[index] );
        }
        else
        {
            kf_geometry_add_point( _manifold, projected, point, separations[index] );
        }
    }
}

/* Clips a capsule core against a polytope face; returns KF_FALSE when the core
   is not parallel to that face. */
static kf_bool_t kf_geometry_segment_face( const kf_geometry_t * _round, const kf_geometry_t * _poly, kf_bool_t _polyIsRight, kf_vec3_t _polyDirection, kf_fixed_t _margin, kf_manifold_t * _manifold )
{
    const kf_geometry_face_t * face = kf_geometry_face( _poly, _polyDirection, KF_TRUE );
    kf_vec3_t first = _round->vertices[0];
    kf_vec3_t second = _round->vertices[1];
    kf_vec3_t edge = kf_vec3_sub( second, first );
    kf_vec3_t centroid;
    uint32_t index;
    int64_t along;
    int64_t square = kf_geometry_dot( edge, edge );
    if( face == NULL || square == 0 )
    {
        return KF_FALSE;
    }
    along = kf_geometry_dot( edge, face->normal ) / KF_FIXED_SCALE;
    /* About 3 degrees: |edge . n| <= |edge| / 16. */
    if( along * along * 256 > square )
    {
        return KF_FALSE;
    }
    centroid = kf_geometry_face_centroid( _poly, face );
    for( index = 0; index != face->count; ++index )
    {
        kf_vec3_t planePoint = _poly->vertices[_poly->indices[face->first + index]];
        kf_vec3_t side = kf_geometry_side_normal( _poly, face, index, centroid );
        int64_t firstDistance = kf_geometry_dot( kf_vec3_sub( first, planePoint ), side );
        int64_t secondDistance = kf_geometry_dot( kf_vec3_sub( second, planePoint ), side );
        if( firstDistance > 0 && secondDistance > 0 )
        {
            return KF_FALSE;
        }
        if( firstDistance > 0 )
        {
            first = kf_geometry_clip_point( first, second, firstDistance, secondDistance );
        }
        else if( secondDistance > 0 )
        {
            second = kf_geometry_clip_point( second, first, secondDistance, firstDistance );
        }
    }
    {
        kf_vec3_t cores[2] = { first, second };
        uint32_t count = kf_vec3_equal( first, second ) == KF_TRUE ? 1u : 2u;
        for( index = 0; index != count; ++index )
        {
            kf_fixed_t height = kf_geometry_face_height( _poly, face, cores[index] );
            kf_fixed_t separation;
            kf_vec3_t roundPoint = kf_geometry_offset( cores[index], face->normal, -(int64_t)_round->radius );
            kf_vec3_t polyPoint = kf_geometry_offset( cores[index], face->normal, -(int64_t)height );
            if( _poly->box == KF_TRUE )
            {
                polyPoint = kf_geometry_box_face_point( _poly, face->normal, cores[index], &height );
            }
            separation = height - _round->radius;
            if( separation > _margin )
            {
                continue;
            }
            if( _polyIsRight == KF_TRUE )
            {
                kf_geometry_add_point( _manifold, roundPoint, polyPoint, separation );
            }
            else
            {
                kf_geometry_add_point( _manifold, polyPoint, roundPoint, separation );
            }
        }
    }
    return (kf_bool_t)( _manifold->count != 0 );
}

static void kf_geometry_manifold( const kf_geometry_t * _left, const kf_geometry_t * _right, const kf_geometry_separation_t * _separation, kf_fixed_t _margin, kf_manifold_t * _manifold )
{
    memset( _manifold, 0, sizeof( *_manifold ) );
    _manifold->normal = _separation->normal;
    if( _separation->distance > _margin )
    {
        return;
    }
    if( _left->rounded == KF_TRUE && _right->rounded == KF_TRUE )
    {
        kf_geometry_rounded_pair( _left, _right, _separation, _margin, _manifold );
        return;
    }
    if( _left->rounded == KF_TRUE && _separation->feature == KF_FEATURE_RIGHT_FACE )
    {
        if( kf_geometry_segment_face( _left, _right, KF_TRUE, _separation->normal, _margin, _manifold ) == KF_TRUE )
        {
            return;
        }
    }
    else if( _right->rounded == KF_TRUE && _separation->feature == KF_FEATURE_LEFT_FACE )
    {
        if( kf_geometry_segment_face( _right, _left, KF_FALSE, kf_vec3_neg( _separation->normal ), _margin, _manifold ) == KF_TRUE )
        {
            return;
        }
    }
    else if( _left->rounded == KF_FALSE && _right->rounded == KF_FALSE )
    {
        if( _separation->feature == KF_FEATURE_RIGHT_FACE )
        {
            kf_geometry_face_pair( _right, _left, KF_TRUE, _separation->normal, _margin, _manifold );
        }
        else if( _separation->feature == KF_FEATURE_LEFT_FACE )
        {
            kf_geometry_face_pair( _left, _right, KF_FALSE, kf_vec3_neg( _separation->normal ), _margin, _manifold );
        }
        if( _manifold->count != 0 )
        {
            return;
        }
    }
    _manifold->count = 0;
    kf_geometry_single_point( _left, _right, _separation, _manifold );
}

/* A rotating compound's individual shapes can leave the world box between
   contained substeps. A distant pair can then exceed Q16 vector subtraction.
   An AABB plane is already a valid separating axis; use its exact 64-bit gap
   until the pair is close enough for the full feature arithmetic. The 16384
   span also bounds sums of three squared differences below INT64_MAX. */
static kf_bool_t kf_geometry_distant_axis( const kf_geometry_t * _left, const kf_geometry_t * _right, kf_geometry_separation_t * _separation )
{
    int64_t minimum[2][3];
    int64_t maximum[2][3];
    const kf_geometry_t * shapes[2] = { _left, _right };
    uint32_t shape;
    uint32_t axis;
    for( shape = 0; shape != 2u; ++shape )
    {
        const kf_geometry_t * geometry = shapes[shape];
        uint32_t index;
        for( axis = 0; axis != 3u; ++axis )
        {
            minimum[shape][axis] = INT64_MAX;
            maximum[shape][axis] = INT64_MIN;
        }
        for( index = 0; index != geometry->vertexCount; ++index )
        {
            int64_t coordinates[3] = { geometry->vertices[index].x, geometry->vertices[index].y, geometry->vertices[index].z };
            for( axis = 0; axis != 3u; ++axis )
            {
                if( coordinates[axis] - geometry->radius < minimum[shape][axis] )
                {
                    minimum[shape][axis] = coordinates[axis] - geometry->radius;
                }
                if( coordinates[axis] + geometry->radius > maximum[shape][axis] )
                {
                    maximum[shape][axis] = coordinates[axis] + geometry->radius;
                }
            }
        }
    }
    for( axis = 0; axis != 3u; ++axis )
    {
        int64_t low = minimum[0][axis] < minimum[1][axis] ? minimum[0][axis] : minimum[1][axis];
        int64_t high = maximum[0][axis] > maximum[1][axis] ? maximum[0][axis] : maximum[1][axis];
        int64_t positive = minimum[0][axis] - maximum[1][axis];
        int64_t negative = minimum[1][axis] - maximum[0][axis];
        int32_t * normal[3] = { &_separation->normal.x, &_separation->normal.y, &_separation->normal.z };
        if( high - low <= (int64_t)KF_COORDINATE_LIMIT * 4 )
        {
            continue;
        }
        if( positive <= 0 && negative <= 0 )
        {
            continue;
        }
        memset( _separation, 0, sizeof( *_separation ) );
        *normal[axis] = positive > negative ? KF_FIXED_SCALE : -KF_FIXED_SCALE;
        _separation->distance = kf_geometry_saturate( positive > negative ? positive : negative );
        _separation->initialized = KF_TRUE;
        return KF_TRUE;
    }
    return KF_FALSE;
}

static kf_status_t kf_geometry_pair( const kf_shape_t * _left, kf_pose_t _leftPose, const kf_shape_t * _right, kf_pose_t _rightPose, kf_bool_t _validate, kf_bool_t _bounded, kf_geometry_t * _leftGeometry, kf_geometry_t * _rightGeometry, kf_geometry_separation_t * _separation )
{
    kf_status_t status = kf_geometry_build( _left, _leftPose, _validate, KF_TRUE, _bounded, _leftGeometry );
    if( status != KF_OK )
    {
        return status;
    }
    status = kf_geometry_build( _right, _rightPose, _validate, KF_TRUE, _bounded, _rightGeometry );
    if( status != KF_OK )
    {
        return status;
    }
    if( _bounded == KF_FALSE && kf_geometry_distant_axis( _leftGeometry, _rightGeometry, _separation ) == KF_TRUE )
    {
        return KF_OK;
    }
    *_separation = kf_geometry_separate( _leftGeometry, _rightGeometry );
    return KF_OK;
}

/* Deepest manifold point; equal depths keep the first point. */
static const kf_manifold_point_t * kf_geometry_deepest( const kf_manifold_t * _manifold )
{
    const kf_manifold_point_t * deepest = _manifold->points;
    uint32_t index;
    for( index = 1; index < _manifold->count; ++index )
    {
        if( _manifold->points[index].separation < deepest->separation )
        {
            deepest = _manifold->points + index;
        }
    }
    return deepest;
}

static kf_status_t kf_geometry_shape_manifold( const kf_shape_t * _left, kf_pose_t _leftPose, const kf_shape_t * _right, kf_pose_t _rightPose, kf_fixed_t _margin, kf_bool_t _validate, kf_bool_t _bounded, kf_manifold_t * _manifold )
{
    kf_geometry_t left;
    kf_geometry_t right;
    kf_geometry_separation_t separation;
    kf_status_t status;
    if( _manifold == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( _margin < 0 || _margin > KF_SIZE_LIMIT )
    {
        return KF_OUT_OF_RANGE;
    }
    status = kf_geometry_pair( _left, _leftPose, _right, _rightPose, _validate, _bounded, &left, &right, &separation );
    if( status != KF_OK )
    {
        return status;
    }
    kf_geometry_manifold( &left, &right, &separation, _margin, _manifold );
    return _manifold->count == 0 ? KF_NO_HIT : KF_OK;
}

kf_status_t kf_shape_manifold( const kf_shape_t * _left, kf_pose_t _leftPose, const kf_shape_t * _right, kf_pose_t _rightPose, kf_fixed_t _margin, kf_manifold_t * _manifold )
{
    return kf_geometry_shape_manifold( _left, _leftPose, _right, _rightPose, _margin, KF_TRUE, KF_TRUE, _manifold );
}

kf_status_t kf_internal_shape_manifold( const kf_shape_t * _left, kf_pose_t _leftPose, const kf_shape_t * _right, kf_pose_t _rightPose, kf_fixed_t _margin, kf_manifold_t * _manifold )
{
    return kf_geometry_shape_manifold( _left, _leftPose, _right, _rightPose, _margin, KF_FALSE, KF_TRUE, _manifold );
}

kf_status_t kf_internal_shape_unbounded_manifold( const kf_shape_t * _left, kf_pose_t _leftPose, const kf_shape_t * _right, kf_pose_t _rightPose, kf_fixed_t _margin, kf_manifold_t * _manifold )
{
    return kf_geometry_shape_manifold( _left, _leftPose, _right, _rightPose, _margin, KF_FALSE, KF_FALSE, _manifold );
}

kf_status_t kf_internal_shape_overlap( const kf_shape_t * _left, kf_pose_t _leftPose, const kf_shape_t * _right, kf_pose_t _rightPose )
{
    kf_geometry_t left;
    kf_geometry_t right;
    kf_geometry_separation_t separation;
    kf_status_t status = kf_geometry_pair( _left, _leftPose, _right, _rightPose, KF_FALSE, KF_TRUE, &left, &right, &separation );
    if( status != KF_OK )
    {
        return status;
    }
    return separation.distance > KF_GEOMETRY_TOLERANCE ? KF_NO_HIT : KF_OK;
}

kf_status_t kf_shape_contact( const kf_shape_t * _left, kf_pose_t _leftPose, const kf_shape_t * _right, kf_pose_t _rightPose, kf_contact_t * _contact )
{
    kf_geometry_t left;
    kf_geometry_t right;
    kf_geometry_separation_t separation;
    kf_manifold_t manifold;
    const kf_manifold_point_t * deepest;
    kf_status_t status;
    if( _contact == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    status = kf_geometry_pair( _left, _leftPose, _right, _rightPose, KF_TRUE, KF_TRUE, &left, &right, &separation );
    if( status != KF_OK )
    {
        return status;
    }
    if( separation.distance > KF_GEOMETRY_TOLERANCE )
    {
        return KF_NO_HIT;
    }
    kf_geometry_manifold( &left, &right, &separation, KF_GEOMETRY_TOLERANCE, &manifold );
    deepest = kf_geometry_deepest( &manifold );
    memset( _contact, 0, sizeof( *_contact ) );
    _contact->normal = separation.normal;
    _contact->depth = separation.distance < 0 ? -separation.distance : 0;
    _contact->point_left = deepest->point_left;
    _contact->point_right = deepest->point_right;
    return KF_OK;
}

static kf_vec3_t kf_geometry_moved( kf_vec3_t _position, kf_vec3_t _displacement, uint64_t _fraction )
{
    return (kf_vec3_t){ _position.x + (kf_fixed_t)( (int64_t)_displacement.x * (int64_t)_fraction / ( INT64_C(1) << 32 ) ), _position.y + (kf_fixed_t)( (int64_t)_displacement.y * (int64_t)_fraction / ( INT64_C(1) << 32 ) ), _position.z + (kf_fixed_t)( (int64_t)_displacement.z * (int64_t)_fraction / ( INT64_C(1) << 32 ) ) };
}

kf_status_t kf_internal_shape_sweep_fraction( const kf_shape_t * _moving, kf_pose_t _pose, kf_vec3_t _displacement, const kf_shape_t * _obstacle, kf_pose_t _obstaclePose, kf_sweep_hit_t * _hit, uint64_t * _fraction )
{
    kf_geometry_t moving;
    kf_geometry_t obstacle;
    kf_geometry_t original;
    kf_status_t status;
    uint64_t fraction = 0;
    kf_vec3_t previousNormal = { 0, 0, 0 };
    uint32_t iteration;
    kf_bool_t startedInside = KF_FALSE;
    if( _hit == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( kf_geometry_vector_valid( _displacement, KF_COORDINATE_LIMIT * 4 ) == KF_FALSE )
    {
        return KF_OUT_OF_RANGE;
    }
    status = kf_geometry_build( _moving, _pose, KF_TRUE, KF_TRUE, KF_TRUE, &original );
    if( status != KF_OK )
    {
        return status;
    }
    status = kf_geometry_build( _obstacle, _obstaclePose, KF_TRUE, KF_TRUE, KF_TRUE, &obstacle );
    if( status != KF_OK )
    {
        return status;
    }
    for( iteration = 0; iteration != KF_GEOMETRY_SWEEP_ITERATIONS; ++iteration )
    {
        kf_geometry_separation_t separation;
        int64_t approach;
        uint64_t advance;
        uint32_t vertex;
        moving = original;
        for( vertex = 0; vertex != moving.vertexCount; ++vertex )
        {
            moving.vertices[vertex] = kf_geometry_moved( original.vertices[vertex], _displacement, fraction );
        }
        moving.center = kf_geometry_moved( original.center, _displacement, fraction );
        if( original.box == KF_TRUE )
        {
            moving.boxPosition = kf_geometry_moved( original.boxPosition, _displacement, fraction );
        }
        separation = kf_geometry_separate( &moving, &obstacle );
        approach = -kf_geometry_dot( _displacement, separation.normal ) / KF_FIXED_SCALE;
        if( separation.distance <= KF_GEOMETRY_TOLERANCE )
        {
            kf_manifold_t manifold;
            /* At a zero-thickness plane the exact impact has two equal escape
               directions. Keep the side approached on the preceding advance. */
            if( fraction != 0 && separation.distance == 0 && approach < 0 )
            {
                kf_vec3_t inverseNormal = kf_vec3_neg( previousNormal );
                if( kf_vec3_equal( separation.normal, inverseNormal ) == KF_TRUE )
                {
                    separation.normal = previousNormal;
                }
            }
            if( fraction == 0 && approach <= 0 )
            {
                return KF_NO_HIT;
            }
            if( fraction == 0 && separation.distance < 0 )
            {
                startedInside = KF_TRUE;
            }
            kf_geometry_manifold( &moving, &obstacle, &separation, KF_GEOMETRY_TOLERANCE, &manifold );
            memset( _hit, 0, sizeof( *_hit ) );
            _hit->fraction = (kf_fixed_t)( fraction >> 16 );
            _hit->fraction_q32 = fraction;
            _hit->position = kf_geometry_moved( _pose.position, _displacement, fraction );
            _hit->normal = separation.normal;
            _hit->contact_point = kf_geometry_deepest( &manifold )->point_right;
            _hit->distance = (kf_fixed_t)( (int64_t)kf_geometry_length( _displacement ) * (int64_t)fraction / ( INT64_C(1) << 32 ) );
            _hit->started_inside = startedInside;
            if( _fraction != NULL )
            {
                *_fraction = fraction;
            }
            return KF_OK;
        }
        if( approach <= 0 )
        {
            return KF_NO_HIT;
        }
        if( separation.distance > approach )
        {
            return KF_NO_HIT;
        }
        advance = kf_geometry_ratio( (uint64_t)separation.distance, (uint64_t)approach, 32 );
        if( advance == 0 )
        {
            advance = 1;
        }
        if( advance > ( UINT64_C(1) << 32 ) - fraction )
        {
            return KF_NO_HIT;
        }
        previousNormal = separation.normal;
        fraction += advance;
    }
    return KF_ITERATION_LIMIT;
}

kf_status_t kf_shape_sweep( const kf_shape_t * _moving, kf_pose_t _pose, kf_vec3_t _displacement, const kf_shape_t * _obstacle, kf_pose_t _obstaclePose, kf_sweep_hit_t * _hit )
{
    kf_pose_t endPose = _pose;
    kf_aabb_t endBounds;
    kf_status_t status;
    if( _hit == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( kf_geometry_vector_valid( _pose.position, KF_COORDINATE_LIMIT ) == KF_FALSE )
    {
        return KF_OUT_OF_RANGE;
    }
    if( kf_geometry_vector_valid( _displacement, KF_COORDINATE_LIMIT * 2 ) == KF_FALSE )
    {
        return KF_OUT_OF_RANGE;
    }
    endPose.position = kf_geometry_moved( _pose.position, _displacement, UINT64_C(1) << 32 );
    status = kf_shape_bounds( _moving, endPose, &endBounds );
    if( status != KF_OK )
    {
        return status;
    }
    return kf_internal_shape_sweep_fraction( _moving, _pose, _displacement, _obstacle, _obstaclePose, _hit, NULL );
}

kf_status_t kf_shape_raycast( const kf_ray_t * _ray, const kf_shape_t * _shape, kf_pose_t _pose, kf_sweep_hit_t * _hit )
{
    kf_shape_t point;
    kf_pose_t pointPose;
    kf_fixed_t length;
    kf_fixed_t maximum;
    kf_vec3_t direction;
    kf_vec3_t displacement;
    if( _ray == NULL )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( _ray->maximum_distance <= 0 )
    {
        return KF_INVALID_ARGUMENT;
    }
    if( _ray->maximum_distance > KF_COORDINATE_LIMIT * 2 )
    {
        return KF_OUT_OF_RANGE;
    }
    if( kf_geometry_vector_valid( _ray->direction, KF_COORDINATE_LIMIT ) == KF_FALSE )
    {
        return KF_OUT_OF_RANGE;
    }
    maximum = kf_fixed_max( kf_fixed_abs( _ray->direction.x ), kf_fixed_max( kf_fixed_abs( _ray->direction.y ), kf_fixed_abs( _ray->direction.z ) ) );
    if( maximum == 0 ) return KF_INVALID_ARGUMENT;
    direction = kf_vec3_mul_div( _ray->direction, KF_FIXED_SCALE, maximum );
    length = kf_geometry_length( direction );
    displacement.x = (kf_fixed_t)((int64_t)direction.x * _ray->maximum_distance / length);
    displacement.y = (kf_fixed_t)((int64_t)direction.y * _ray->maximum_distance / length);
    displacement.z = (kf_fixed_t)((int64_t)direction.z * _ray->maximum_distance / length);
    memset( &point, 0, sizeof( point ) );
    point.type = KF_SHAPE_SPHERE;
    memset( &pointPose, 0, sizeof( pointPose ) );
    pointPose.position = _ray->origin;
    return kf_shape_sweep( &point, pointPose, displacement, _shape, _pose, _hit );
}

static int64_t kf_geometry_path( int32_t _start, int32_t _end, int32_t _curve, uint64_t _fraction, uint64_t _square )
{
    return _start + kf_internal_mul_shift( (int64_t)_end - _start - _curve, (int64_t)_fraction, 32 ) + kf_internal_mul_shift( _curve, (int64_t)_square, 32 );
}

kf_vec3_t kf_internal_motion_center( const kf_motion_t * _motion, uint64_t _fraction )
{
    uint64_t square;
    if( _motion->translation_segments != NULL && _motion->translation_count != 0 )
    {
        uint64_t time;
        uint64_t end;
        uint64_t local;
        uint32_t index = 0;
        const kf_rotation_segment_t * segment;
        kf_vec3_t next;
        if( _fraction == 0 )
        {
            return _motion->center_start;
        }
        if( _fraction >= ( UINT64_C(1) << 32 ) )
        {
            return _motion->center_end;
        }
        time = _motion->rotation_fraction_start + (uint64_t)kf_internal_mul_shift( (int64_t)( ( UINT64_C(1) << 32 ) - _motion->rotation_fraction_start ), (int64_t)_fraction, 32 );
        while( index + 1u < _motion->translation_count && _motion->translation_segments[index + 1u].time <= time )
        {
            ++index;
        }
        segment = _motion->translation_segments + index;
        end = index + 1u < _motion->translation_count ? _motion->translation_segments[index + 1u].time : ( UINT64_C(1) << 32 );
        next = index + 1u < _motion->translation_count ? _motion->translation_segments[index + 1u].center : _motion->center_end;
        local = time <= segment->time ? 0 : (uint64_t)kf_internal_div_shift( (int64_t)(time - segment->time), (int64_t)(end - segment->time), 32 );
        return (kf_vec3_t){
            kf_geometry_saturate( segment->center.x + kf_internal_mul_shift( (int64_t)next.x - segment->center.x, (int64_t)local, 32 ) ),
            kf_geometry_saturate( segment->center.y + kf_internal_mul_shift( (int64_t)next.y - segment->center.y, (int64_t)local, 32 ) ),
            kf_geometry_saturate( segment->center.z + kf_internal_mul_shift( (int64_t)next.z - segment->center.z, (int64_t)local, 32 ) ) };
    }
    square = (uint64_t)kf_internal_mul_shift( (int64_t)_fraction, (int64_t)_fraction, 32 );
    return (kf_vec3_t){
        kf_geometry_saturate( kf_geometry_path( _motion->center_start.x, _motion->center_end.x, _motion->center_curve.x, _fraction, square ) ),
        kf_geometry_saturate( kf_geometry_path( _motion->center_start.y, _motion->center_end.y, _motion->center_curve.y, _fraction, square ) ),
        kf_geometry_saturate( kf_geometry_path( _motion->center_start.z, _motion->center_end.z, _motion->center_curve.z, _fraction, square ) ) };
}

kf_quat_t kf_internal_motion_rotation( const kf_motion_t * _motion, uint64_t _fraction )
{
    uint64_t fraction;
    uint64_t segmentFraction;
    const kf_rotation_segment_t * segment;
    uint32_t index = 0;
    uint64_t end;
    int64_t vector[3];
    uint32_t axis;
    if( _fraction == 0 )
    {
        return _motion->rotation_start;
    }
    if( _fraction >= ( UINT64_C(1) << 32 ) )
    {
        return _motion->rotation_end;
    }
    if( _motion->rotation_segments == NULL )
    {
        return kf_internal_quat_nlerp( _motion->rotation_start, _motion->rotation_end, _fraction );
    }
    if( _motion->rotation_count == 0 )
    {
        return kf_internal_quat_nlerp( _motion->rotation_start, _motion->rotation_end, _fraction );
    }
    fraction = _motion->rotation_fraction_start + (uint64_t)kf_internal_mul_shift( (int64_t)( ( UINT64_C(1) << 32 ) - _motion->rotation_fraction_start ), (int64_t)_fraction, 32 );
    if( fraction >= ( UINT64_C(1) << 32 ) )
    {
        return _motion->rotation_end;
    }
    while( index + 1u < _motion->rotation_count && _motion->rotation_segments[index + 1u].time <= fraction )
    {
        ++index;
    }
    segment = _motion->rotation_segments + index;
    end = index + 1u < _motion->rotation_count ? _motion->rotation_segments[index + 1u].time : ( UINT64_C(1) << 32 );
    segmentFraction = fraction <= segment->time ? 0 : (uint64_t)kf_internal_div_shift( (int64_t)(fraction - segment->time), (int64_t)(end - segment->time), 32 );
    for( axis = 0; axis != 3u; ++axis )
    {
        vector[axis] = kf_internal_mul_shift( segment->vector[axis], (int64_t)segmentFraction, 32 );
    }
    return kf_internal_quat_rotate( segment->rotation, vector );
}

kf_pose_t kf_internal_motion_pose( const kf_motion_t * _motion, uint64_t _fraction )
{
    kf_pose_t pose;
    kf_quat_t quat = kf_internal_motion_rotation( _motion, _fraction );
    kf_matrix_t matrix = kf_internal_matrix( quat );
    kf_vec3_t offset = kf_internal_matrix_apply( &matrix, _motion->local_center );
    kf_vec3_t center = kf_internal_motion_center( _motion, _fraction );
    pose.position = (kf_vec3_t){ kf_geometry_saturate( (int64_t)center.x - offset.x ), kf_geometry_saturate( (int64_t)center.y - offset.y ), kf_geometry_saturate( (int64_t)center.z - offset.z ) };
    pose.rotation = kf_internal_rotation_unit( quat );
    return pose;
}

/* Upper bound of the speed of any shape point relative to its center, per unit
   path fraction. Recorded substeps retain turns beyond the shortest arc. */
static int64_t kf_geometry_rotation_bound( const kf_motion_t * _motion )
{
    int64_t chord;
    if( _motion->rotation_segments != NULL && _motion->rotation_count != 0 )
    {
        int64_t maximum = 0;
        uint32_t index;
        for( index = 0; index != _motion->rotation_count; ++index )
        {
            uint64_t end = index + 1u < _motion->rotation_count ? _motion->rotation_segments[index + 1u].time : ( UINT64_C(1) << 32 );
            const int64_t * vector = _motion->rotation_segments[index].vector;
            int64_t speed = (int64_t)kf_internal_magnitude( vector[0] ) + (int64_t)kf_internal_magnitude( vector[1] ) + (int64_t)kf_internal_magnitude( vector[2] );
            if( end <= _motion->rotation_fraction_start )
            {
                continue;
            }
            speed = kf_internal_div_shift( speed, (int64_t)(end - _motion->rotation_segments[index].time), 32 );
            if( speed > maximum )
            {
                maximum = speed;
            }
        }
        return kf_internal_mul3_shift( maximum, (int64_t)( ( UINT64_C(1) << 32 ) - _motion->rotation_fraction_start ), _motion->extent, 62 ) + 1;
    }
    chord = kf_internal_quat_chord( _motion->rotation_start, _motion->rotation_end );
    return kf_internal_mul3_shift( chord, _motion->extent, KF_GEOMETRY_CHORD_SPEED, 46 ) + ( chord != 0 ? 1 : 0 );
}

/* Maximum closing center speed along one separation axis. Substep paths can
   reverse direction, so the endpoint chord alone is not a safe speed bound. */
static int64_t kf_geometry_translation_bound( const kf_motion_t * _motion, kf_vec3_t _normal, int32_t _sign )
{
    int64_t maximum = INT64_MIN;
    uint32_t index;
    if( _motion->translation_segments == NULL || _motion->translation_count == 0 )
    {
        kf_vec3_t delta = {
            kf_geometry_saturate( (int64_t)_motion->center_end.x - _motion->center_start.x - _motion->center_curve.x ),
            kf_geometry_saturate( (int64_t)_motion->center_end.y - _motion->center_start.y - _motion->center_curve.y ),
            kf_geometry_saturate( (int64_t)_motion->center_end.z - _motion->center_start.z - _motion->center_curve.z ) };
        return _sign * kf_geometry_dot( delta, _normal ) / KF_FIXED_SCALE + 2 * ((int64_t)kf_internal_magnitude( _motion->center_curve.x ) + (int64_t)kf_internal_magnitude( _motion->center_curve.y ) + (int64_t)kf_internal_magnitude( _motion->center_curve.z ));
    }
    for( index = 0; index != _motion->translation_count; ++index )
    {
        const kf_rotation_segment_t * segment = _motion->translation_segments + index;
        uint64_t end = index + 1u < _motion->translation_count ? _motion->translation_segments[index + 1u].time : ( UINT64_C(1) << 32 );
        kf_vec3_t next = index + 1u < _motion->translation_count ? _motion->translation_segments[index + 1u].center : _motion->center_end;
        int64_t projection;
        int64_t speed;
        if( end <= _motion->rotation_fraction_start )
        {
            continue;
        }
        projection = ((int64_t)next.x - segment->center.x) * _normal.x + ((int64_t)next.y - segment->center.y) * _normal.y + ((int64_t)next.z - segment->center.z) * _normal.z;
        speed = kf_internal_div_shift( _sign * projection, (int64_t)(end - segment->time), 16 );
        if( speed > maximum )
        {
            maximum = speed;
        }
    }
    if( maximum == INT64_MIN )
    {
        return 0;
    }
    return kf_internal_mul_shift( maximum, (int64_t)(( UINT64_C(1) << 32 ) - _motion->rotation_fraction_start), 32 ) + 1;
}

static int64_t kf_geometry_segment_speed( const kf_motion_t * _motion, uint32_t _index, kf_vec3_t _normal )
{
    const kf_rotation_segment_t * segment = _motion->translation_segments + _index;
    uint64_t end = _index + 1u < _motion->translation_count ? _motion->translation_segments[_index + 1u].time : ( UINT64_C(1) << 32 );
    kf_vec3_t next = _index + 1u < _motion->translation_count ? _motion->translation_segments[_index + 1u].center : _motion->center_end;
    int64_t projection = ((int64_t)next.x - segment->center.x) * _normal.x + ((int64_t)next.y - segment->center.y) * _normal.y + ((int64_t)next.z - segment->center.z) * _normal.z;
    return kf_internal_div_shift( projection, (int64_t)(end - segment->time), 16 );
}

/* Match simultaneous intervals before bounding relative speed. Independent
   maxima would invent closing motion for bodies following the same acceleration. */
static int64_t kf_geometry_relative_translation_bound( const kf_motion_t * _motion, const kf_motion_t * _obstacleMotion, kf_vec3_t _normal )
{
    uint32_t left = 0;
    uint32_t right = 0;
    uint64_t time = _motion->rotation_fraction_start;
    int64_t maximum = INT64_MIN;
    if( _motion->translation_count == 0 || _obstacleMotion->translation_count == 0 || _motion->rotation_fraction_start != _obstacleMotion->rotation_fraction_start )
    {
        return kf_geometry_translation_bound( _motion, _normal, -1 ) + kf_geometry_translation_bound( _obstacleMotion, _normal, 1 );
    }
    while( time < ( UINT64_C(1) << 32 ) )
    {
        uint64_t leftEnd;
        uint64_t rightEnd;
        int64_t speed;
        while( left + 1u < _motion->translation_count && _motion->translation_segments[left + 1u].time <= time )
        {
            ++left;
        }
        while( right + 1u < _obstacleMotion->translation_count && _obstacleMotion->translation_segments[right + 1u].time <= time )
        {
            ++right;
        }
        leftEnd = left + 1u < _motion->translation_count ? _motion->translation_segments[left + 1u].time : ( UINT64_C(1) << 32 );
        rightEnd = right + 1u < _obstacleMotion->translation_count ? _obstacleMotion->translation_segments[right + 1u].time : ( UINT64_C(1) << 32 );
        speed = kf_geometry_segment_speed( _obstacleMotion, right, _normal ) - kf_geometry_segment_speed( _motion, left, _normal );
        if( speed > maximum )
        {
            maximum = speed;
        }
        time = leftEnd < rightEnd ? leftEnd : rightEnd;
    }
    if( maximum == INT64_MIN )
    {
        return 0;
    }
    return kf_internal_mul_shift( maximum, (int64_t)(( UINT64_C(1) << 32 ) - _motion->rotation_fraction_start), 32 ) + 2;
}

kf_status_t kf_internal_shape_toi( const kf_shape_t * _moving, const kf_motion_t * _motion, const kf_shape_t * _obstacle, const kf_motion_t * _obstacleMotion, kf_fixed_t _target, uint64_t * _fraction, kf_fixed_t * _initialSeparation )
{
    int64_t rotation = kf_geometry_rotation_bound( _motion ) + kf_geometry_rotation_bound( _obstacleMotion );
    uint64_t fraction = 0;
    uint32_t iteration;
    for( iteration = 0; iteration != KF_GEOMETRY_SWEEP_ITERATIONS; ++iteration )
    {
        kf_geometry_t moving;
        kf_geometry_t obstacle;
        kf_geometry_separation_t separation;
        int64_t approach;
        int64_t gap;
        uint64_t advance;
        /* Containment guarantees committed poses. A recorded rotation between
           them can briefly cross a world plane and still needs a valid TOI. */
        kf_status_t status = kf_geometry_pair( _moving, kf_internal_motion_pose( _motion, fraction ), _obstacle, kf_internal_motion_pose( _obstacleMotion, fraction ), KF_FALSE, KF_FALSE, &moving, &obstacle, &separation );
        if( status != KF_OK )
        {
            return status;
        }
        if( iteration == 0 )
        {
            *_initialSeparation = separation.distance;
        }
        gap = (int64_t)separation.distance - _target;
        if( gap <= KF_GEOMETRY_TOLERANCE )
        {
            *_fraction = fraction;
            return KF_OK;
        }
        /* Separation along the current axis shrinks at most this fast. */
        approach = kf_geometry_relative_translation_bound( _motion, _obstacleMotion, separation.normal ) + rotation;
        if( approach <= 0 || gap > approach )
        {
            return KF_NO_HIT;
        }
        advance = kf_geometry_ratio( (uint64_t)gap, (uint64_t)approach, 32 );
        if( advance == 0 )
        {
            advance = 1;
        }
        if( advance > ( UINT64_C(1) << 32 ) - fraction )
        {
            return KF_NO_HIT;
        }
        fraction += advance;
    }
    /* Every advanced fraction is proven separated; stop at the last one. */
    *_fraction = fraction;
    return KF_OK;
}

static int64_t kf_geometry_square( int64_t _value )
{
    return kf_internal_mul_shift( _value, _value, 8 );
}

kf_status_t kf_internal_shape_mass( const kf_shape_t * _shape, int64_t * _volume, kf_vec3_t * _centroid, kf_tensor_t * _inertia, kf_fixed_t * _thickness )
{
    memset( _inertia, 0, sizeof( *_inertia ) );
    switch( _shape->type )
    {
    case KF_SHAPE_SPHERE:
    {
        int64_t radius = _shape->data.sphere.radius;
        int64_t square = kf_geometry_square( radius );
        /* Volumes are Q16.16 cubic units. */
        *_volume = kf_internal_mul3_shift( radius * radius, radius, INT64_C(274517), 48 );
        *_centroid = _shape->data.sphere.position;
        _inertia->xx = square * 2 / 5;
        _inertia->yy = _inertia->xx;
        _inertia->zz = _inertia->xx;
        *_thickness = _shape->data.sphere.radius;
        return KF_OK;
    }
    case KF_SHAPE_CAPSULE:
    {
        int64_t radius = _shape->data.capsule.radius;
        int64_t cylinder = (int64_t)_shape->data.capsule.height - 2 * radius;
        int64_t cap = radius * 4 / 3;
        int64_t square = kf_geometry_square( radius );
        int64_t length = kf_geometry_square( cylinder );
        int64_t product = kf_internal_mul_shift( cylinder, radius, 8 );
        int64_t cylinderShare = cylinder + cap == 0 ? 0 : kf_internal_div_shift( cylinder, cylinder + cap, 30 );
        int64_t capShare = ( INT64_C(1) << 30 ) - cylinderShare;
        *_volume = kf_internal_mul3_shift( radius * radius, INT64_C(205887), cylinder + cap, 48 );
        *_centroid = _shape->data.capsule.position;
        _centroid->y += _shape->data.capsule.height / 2;
        _inertia->yy = kf_internal_mul_shift( cylinderShare, square / 2, 30 ) + kf_internal_mul_shift( capShare, square * 2 / 5, 30 );
        _inertia->xx = kf_internal_mul_shift( cylinderShare, square / 4 + length / 12, 30 ) + kf_internal_mul_shift( capShare, square * 2 / 5 + length / 4 + product * 3 / 8, 30 );
        _inertia->zz = _inertia->xx;
        *_thickness = _shape->data.capsule.radius;
        return KF_OK;
    }
    case KF_SHAPE_AABB:
    case KF_SHAPE_BOX:
    {
        int64_t half[3];
        kf_angle16_t yaw = 0;
        int64_t xx;
        int64_t yy;
        int64_t zz;
        kf_matrix_t rotation;
        uint32_t axis;
        if( _shape->type == KF_SHAPE_AABB )
        {
            half[0] = ( (int64_t)_shape->data.aabb.maximum.x - _shape->data.aabb.minimum.x ) / 2;
            half[1] = ( (int64_t)_shape->data.aabb.maximum.y - _shape->data.aabb.minimum.y ) / 2;
            half[2] = ( (int64_t)_shape->data.aabb.maximum.z - _shape->data.aabb.minimum.z ) / 2;
            _centroid->x = (kf_fixed_t)( ( (int64_t)_shape->data.aabb.maximum.x + _shape->data.aabb.minimum.x ) / 2 );
            _centroid->y = (kf_fixed_t)( ( (int64_t)_shape->data.aabb.maximum.y + _shape->data.aabb.minimum.y ) / 2 );
            _centroid->z = (kf_fixed_t)( ( (int64_t)_shape->data.aabb.maximum.z + _shape->data.aabb.minimum.z ) / 2 );
        }
        else
        {
            half[0] = _shape->data.box.half_extents.x;
            half[1] = _shape->data.box.half_extents.y;
            half[2] = _shape->data.box.half_extents.z;
            *_centroid = _shape->data.box.center;
            yaw = _shape->data.box.yaw;
        }
        *_volume = kf_internal_mul_shift( half[0] * half[1], half[2], 32 ) * 8;
        xx = ( kf_geometry_square( half[1] ) + kf_geometry_square( half[2] ) ) / 3;
        yy = ( kf_geometry_square( half[0] ) + kf_geometry_square( half[2] ) ) / 3;
        zz = ( kf_geometry_square( half[0] ) + kf_geometry_square( half[1] ) ) / 3;
        rotation = kf_internal_matrix( kf_internal_quat( kf_rotation_from_yaw( yaw ) ) );
        _inertia->xx = xx;
        _inertia->yy = yy;
        _inertia->zz = zz;
        *_inertia = kf_internal_tensor_rotate( &rotation, *_inertia );
        *_thickness = (kf_fixed_t)half[0];
        for( axis = 1; axis != 3u; ++axis )
        {
            if( half[axis] < *_thickness )
            {
                *_thickness = (kf_fixed_t)half[axis];
            }
        }
        return KF_OK;
    }
    case KF_SHAPE_PRISM:
    {
        const kf_prism_t * prism = &_shape->data.prism;
        int64_t height = (int64_t)prism->maximum_y - prism->minimum_y;
        int64_t twiceArea = 0;
        int64_t centerX = 0;
        int64_t centerZ = 0;
        int64_t momentX = 0;
        int64_t momentZ = 0;
        int64_t product = 0;
        int64_t thickness = height / 2;
        uint32_t index;
        for( index = 1; index + 1u < prism->count; ++index )
        {
            kf_vec3_t first = kf_vec3_sub( prism->vertices[index], prism->vertices[0] );
            kf_vec3_t second = kf_vec3_sub( prism->vertices[index + 1u], prism->vertices[0] );
            twiceArea += (int64_t)first.x * second.z - (int64_t)second.x * first.z;
        }
        for( index = 1; index + 1u < prism->count; ++index )
        {
            kf_vec3_t first = kf_vec3_sub( prism->vertices[index], prism->vertices[0] );
            kf_vec3_t second = kf_vec3_sub( prism->vertices[index + 1u], prism->vertices[0] );
            int64_t doubleArea = (int64_t)first.x * second.z - (int64_t)second.x * first.z;
            centerX += kf_internal_mul_div( doubleArea, (int64_t)prism->vertices[0].x + prism->vertices[index].x + prism->vertices[index + 1u].x, twiceArea, 0 );
            centerZ += kf_internal_mul_div( doubleArea, (int64_t)prism->vertices[0].z + prism->vertices[index].z + prism->vertices[index + 1u].z, twiceArea, 0 );
        }
        centerX /= 3;
        centerZ /= 3;
        /* Second moments of triangles (centroid, a, b) per unit area, in Q32. */
        for( index = 0; index != prism->count; ++index )
        {
            const kf_vec3_t * next = prism->vertices + ( index + 1u ) % prism->count;
            int64_t ax = prism->vertices[index].x - centerX;
            int64_t az = prism->vertices[index].z - centerZ;
            int64_t bx = next->x - centerX;
            int64_t bz = next->z - centerZ;
            int64_t weight = kf_internal_div_shift( ax * bz - bx * az, twiceArea, 30 );
            int64_t edgeX = bx - ax;
            int64_t edgeZ = bz - az;
            int64_t edgeLength = (int64_t)kf_internal_sqrt( (uint64_t)( edgeX * edgeX + edgeZ * edgeZ ) );
            momentX += kf_internal_mul_shift( weight, ax * ax + ax * bx + bx * bx, 30 ) / 6;
            momentZ += kf_internal_mul_shift( weight, az * az + az * bz + bz * bz, 30 ) / 6;
            product += kf_internal_mul_shift( weight, 2 * ax * az + ax * bz + bx * az + 2 * bx * bz, 30 ) / 12;
            if( edgeLength != 0 )
            {
                /* Distance from the centroid to the edge line. */
                int64_t distance = kf_internal_magnitude( ax * bz - bx * az ) == 0 ? 0 : (int64_t)( kf_internal_magnitude( ax * bz - bx * az ) / (uint64_t)edgeLength );
                if( distance < thickness )
                {
                    thickness = distance;
                }
            }
        }
        *_volume = kf_internal_mul_shift( twiceArea < 0 ? -twiceArea : twiceArea, height, 33 );
        _centroid->x = (kf_fixed_t)centerX;
        _centroid->y = (kf_fixed_t)( ( (int64_t)prism->minimum_y + prism->maximum_y ) / 2 );
        _centroid->z = (kf_fixed_t)centerZ;
        momentX /= 256;
        momentZ /= 256;
        product /= 256;
        _inertia->xx = kf_geometry_square( height ) / 12 + momentZ;
        _inertia->zz = kf_geometry_square( height ) / 12 + momentX;
        _inertia->yy = momentX + momentZ;
        _inertia->xz = -product;
        *_thickness = (kf_fixed_t)thickness;
        return KF_OK;
    }
    case KF_SHAPE_TRIANGLE:
        *_volume = 0;
        _centroid->x = (kf_fixed_t)( ( (int64_t)_shape->data.triangle.vertices[0].x + _shape->data.triangle.vertices[1].x + _shape->data.triangle.vertices[2].x ) / 3 );
        _centroid->y = (kf_fixed_t)( ( (int64_t)_shape->data.triangle.vertices[0].y + _shape->data.triangle.vertices[1].y + _shape->data.triangle.vertices[2].y ) / 3 );
        _centroid->z = (kf_fixed_t)( ( (int64_t)_shape->data.triangle.vertices[0].z + _shape->data.triangle.vertices[1].z + _shape->data.triangle.vertices[2].z ) / 3 );
        *_thickness = 0;
        return KF_OK;
    default:
        return KF_INVALID_ARGUMENT;
    }
}

kf_status_t kf_internal_shape_extent( const kf_shape_t * _shape, kf_vec3_t _center, kf_fixed_t * _extent )
{
    kf_geometry_t geometry;
    kf_pose_t identity;
    kf_status_t status;
    uint64_t farthest = 0;
    uint32_t index;
    memset( &identity, 0, sizeof( identity ) );
    status = kf_geometry_build( _shape, identity, KF_FALSE, KF_FALSE, KF_FALSE, &geometry );
    if( status != KF_OK )
    {
        return status;
    }
    for( index = 0; index != geometry.vertexCount; ++index )
    {
        int64_t x = (int64_t)geometry.vertices[index].x - _center.x;
        int64_t y = (int64_t)geometry.vertices[index].y - _center.y;
        int64_t z = (int64_t)geometry.vertices[index].z - _center.z;
        uint64_t square = (uint64_t)( x * x ) + (uint64_t)( y * y ) + (uint64_t)( z * z );
        if( square > farthest )
        {
            farthest = square;
        }
    }
    /* Two raw units cover rounding of rotated vertices. */
    *_extent = kf_geometry_saturate( (int64_t)kf_internal_sqrt( farthest ) + geometry.radius + 2 );
    return KF_OK;
}
