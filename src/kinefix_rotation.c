#include "kinefix_internal.h"

#include <limits.h>

#define KF_ROTATION_ONE INT64_C(1073741824)
#define KF_ROTATION_TWO_PI INT64_C(6746518852)
#define KF_ROTATION_HALF_PI INT64_C(1686629713)

/* Index of the highest set bit of a nonzero value. */
static uint32_t kf_rotation_high_bit( uint64_t _value )
{
    uint32_t bits = 0;
    uint32_t step;
    for( step = 32u; step != 0; step >>= 1 )
    {
        if( ( _value >> ( bits + step ) ) != 0 )
        {
            bits += step;
        }
    }
    return bits;
}

uint64_t kf_internal_wide_divide( uint64_t _high, uint64_t _low, uint64_t _divisor )
{
#if defined(KF_INTERNAL_NATIVE_WIDE)
    return (uint64_t)( ( ( (kf_wide_t)_high << 64 ) | _low ) / _divisor );
#elif defined(KF_INTERNAL_MSVC_X64) && _MSC_VER >= 1920
    uint64_t remainder;
    return _udiv128( _high, _low, _divisor, &remainder );
#else
    /* Long division in two 32-bit digits after normalizing the divisor
       (Hacker's Delight, divlu). */
    const uint64_t base = UINT64_C(1) << 32;
    uint32_t shift = 63u - kf_rotation_high_bit( _divisor );
    uint64_t divisor = _divisor << shift;
    uint64_t divisorHigh = divisor >> 32;
    uint64_t divisorLow = divisor & ( base - 1u );
    uint64_t high = shift == 0 ? _high : ( _high << shift ) | ( _low >> ( 64u - shift ) );
    uint64_t low = _low << shift;
    uint64_t lowHigh = low >> 32;
    uint64_t lowLow = low & ( base - 1u );
    uint64_t quotientHigh = high / divisorHigh;
    uint64_t rest = high - quotientHigh * divisorHigh;
    uint64_t middle;
    uint64_t quotientLow;
    while( quotientHigh >= base || quotientHigh * divisorLow > base * rest + lowHigh )
    {
        --quotientHigh;
        rest += divisorHigh;
        if( rest >= base )
        {
            break;
        }
    }
    middle = high * base + lowHigh - quotientHigh * divisor;
    quotientLow = middle / divisorHigh;
    rest = middle - quotientLow * divisorHigh;
    while( quotientLow >= base || quotientLow * divisorLow > base * rest + lowLow )
    {
        --quotientLow;
        rest += divisorHigh;
        if( rest >= base )
        {
            break;
        }
    }
    return quotientHigh * base + quotientLow;
#endif
}

uint64_t kf_internal_wide_quotient( kf_wide_t _numerator, kf_wide_t _denominator )
{
    uint64_t numeratorHigh;
    uint64_t numeratorLow;
    uint64_t divisorHigh;
    uint64_t divisorLow;
#if !defined(KF_INTERNAL_NATIVE_WIDE)
    uint32_t shift;
    uint64_t divisor;
    uint64_t estimate;
    uint64_t productHigh;
    uint64_t productLow;
    uint64_t restHigh;
    uint64_t restLow;
#endif
    kf_internal_wide_parts( _numerator, &numeratorHigh, &numeratorLow );
    kf_internal_wide_parts( _denominator, &divisorHigh, &divisorLow );
    if( divisorHigh == 0 )
    {
        if( divisorLow == 0 || numeratorHigh >= divisorLow )
        {
            return UINT64_MAX;
        }
        return numeratorHigh == 0 ? numeratorLow / divisorLow : kf_internal_wide_divide( numeratorHigh, numeratorLow, divisorLow );
    }
#if defined(KF_INTERNAL_NATIVE_WIDE)
    /* A denominator with a high word makes the quotient fit in one word. */
    return (uint64_t)( _numerator / _denominator );
#else
    /* Normalize the divisor's top word, divide the halved numerator by it and
       correct the estimate once (Hacker's Delight, divDu). */
    shift = 63u - kf_rotation_high_bit( divisorHigh );
    divisor = shift == 0 ? divisorHigh : ( divisorHigh << shift ) | ( divisorLow >> ( 64u - shift ) );
    estimate = kf_internal_wide_divide( numeratorHigh >> 1, ( numeratorLow >> 1 ) | ( numeratorHigh << 63 ), divisor ) >> ( 63u - shift );
    if( estimate != 0 )
    {
        --estimate;
    }
    /* estimate * divisor stays at most the numerator, below 2^128, so the high
       word product cannot wrap and the remainder fits. */
    kf_internal_wide_parts( kf_internal_wide_multiply( estimate, divisorLow ), &productHigh, &productLow );
    productHigh += estimate * divisorHigh;
    restLow = numeratorLow - productLow;
    restHigh = numeratorHigh - productHigh - ( numeratorLow < productLow ? 1u : 0u );
    if( restHigh > divisorHigh || ( restHigh == divisorHigh && restLow >= divisorLow ) )
    {
        ++estimate;
    }
    return estimate;
#endif
}

int64_t kf_internal_mul_div( int64_t _left, int64_t _right, int64_t _denominator, uint32_t _shift )
{
    kf_bool_t negative = (kf_bool_t)( ( ( _left < 0 ) != ( _right < 0 ) ) != ( _denominator < 0 ) );
    uint64_t high;
    uint64_t low;
    kf_internal_wide_parts( kf_internal_wide_multiply( kf_internal_magnitude( _left ), kf_internal_magnitude( _right ) ), &high, &low );
    if( high == 0 && low == 0 )
    {
        return 0;
    }
    if( _shift != 0 )
    {
        if( ( high >> ( 64u - _shift ) ) != 0 )
        {
            return kf_internal_signed( (uint64_t)INT64_MAX, negative );
        }
        high = ( high << _shift ) | ( low >> ( 64u - _shift ) );
        low <<= _shift;
    }
    return kf_internal_signed( kf_internal_wide_quotient( kf_internal_wide_make( high, low ), kf_internal_wide_make( 0, kf_internal_magnitude( _denominator ) ) ), negative );
}

int64_t kf_internal_div_product( int64_t _numerator, int64_t _first, int64_t _second, uint32_t _shift )
{
    kf_bool_t negative = (kf_bool_t)( ( ( _numerator < 0 ) != ( _first < 0 ) ) != ( _second < 0 ) );
    uint64_t magnitude = kf_internal_magnitude( _numerator );
    uint64_t high;
    uint64_t low;
    if( magnitude == 0 )
    {
        return 0;
    }
    high = _shift == 0 ? 0 : ( _shift == 64u ? magnitude : magnitude >> ( 64u - _shift ) );
    low = _shift == 64u ? 0 : magnitude << _shift;
    return kf_internal_signed( kf_internal_wide_quotient( kf_internal_wide_make( high, low ), kf_internal_wide_multiply( kf_internal_magnitude( _first ), kf_internal_magnitude( _second ) ) ), negative );
}

int64_t kf_internal_div_shift( int64_t _numerator, int64_t _denominator, uint32_t _shift )
{
    kf_bool_t negative = (kf_bool_t)( ( _numerator < 0 ) != ( _denominator < 0 ) );
    uint64_t numerator = kf_internal_magnitude( _numerator );
    /* The 128-bit value numerator << shift, for shifts below 64. */
    uint64_t high = _shift == 0 ? 0 : numerator >> ( 64u - _shift );
    uint64_t low = numerator << _shift;
    if( numerator == 0 )
    {
        return 0;
    }
    return kf_internal_signed( kf_internal_wide_quotient( kf_internal_wide_make( high, low ), kf_internal_wide_make( 0, kf_internal_magnitude( _denominator ) ) ), negative );
}

/* Floor square root by integer Newton iteration from a power of two at or
   above the root; the iteration decreases monotonically to the exact floor. */
uint64_t kf_internal_sqrt( uint64_t _value )
{
    uint64_t root;
    if( _value < 2u )
    {
        return _value;
    }
    root = UINT64_C(1) << ( ( kf_rotation_high_bit( _value ) + 2u ) / 2u );
    for( ;; )
    {
        uint64_t next = ( root + _value / root ) / 2u;
        if( next >= root )
        {
            return root;
        }
        root = next;
    }
}

static int64_t kf_rotation_mul( int64_t _left, int64_t _right )
{
    return _left * _right / KF_ROTATION_ONE;
}

static int32_t kf_rotation_clamp_unit( int64_t _value )
{
    if( _value > KF_ROTATION_ONE )
    {
        return (int32_t)KF_ROTATION_ONE;
    }
    if( _value < -KF_ROTATION_ONE )
    {
        return -(int32_t)KF_ROTATION_ONE;
    }
    return (int32_t)_value;
}

void kf_internal_sin_cos( int64_t _radians, int32_t * _sine, int32_t * _cosine )
{
    int64_t angle = _radians % KF_ROTATION_TWO_PI;
    int64_t square;
    int64_t sine;
    int64_t cosine;
    uint32_t halvings = 0;
    uint32_t index;
    if( angle > KF_INTERNAL_PI )
    {
        angle -= KF_ROTATION_TWO_PI;
    }
    if( angle < -KF_INTERNAL_PI )
    {
        angle += KF_ROTATION_TWO_PI;
    }
    /* Taylor terms through x^8 are below one Q30 unit for |x| <= 1/4. */
    while( angle > KF_ROTATION_ONE / 4 || angle < -KF_ROTATION_ONE / 4 )
    {
        angle /= 2;
        ++halvings;
    }
    square = kf_rotation_mul( angle, angle );
    sine = KF_ROTATION_ONE - square / 72;
    sine = KF_ROTATION_ONE - kf_rotation_mul( square, sine ) / 42;
    sine = KF_ROTATION_ONE - kf_rotation_mul( square, sine ) / 20;
    sine = KF_ROTATION_ONE - kf_rotation_mul( square, sine ) / 6;
    sine = kf_rotation_mul( angle, sine );
    cosine = KF_ROTATION_ONE - square / 90;
    cosine = KF_ROTATION_ONE - kf_rotation_mul( square, cosine ) / 56;
    cosine = KF_ROTATION_ONE - kf_rotation_mul( square, cosine ) / 30;
    cosine = KF_ROTATION_ONE - kf_rotation_mul( square, cosine ) / 12;
    cosine = KF_ROTATION_ONE - kf_rotation_mul( square, cosine ) / 2;
    for( index = 0; index != halvings; ++index )
    {
        int64_t doubled = 2 * kf_rotation_mul( sine, cosine );
        cosine = kf_rotation_mul( cosine, cosine ) - kf_rotation_mul( sine, sine );
        sine = doubled;
    }
    *_sine = kf_rotation_clamp_unit( sine );
    *_cosine = kf_rotation_clamp_unit( cosine );
}

kf_quat_t kf_internal_quat( kf_rotation_t _rotation )
{
    uint64_t square = (uint64_t)( (int64_t)_rotation.x * _rotation.x ) + (uint64_t)( (int64_t)_rotation.y * _rotation.y ) + (uint64_t)( (int64_t)_rotation.z * _rotation.z );
    uint64_t one = (uint64_t)KF_ROTATION_ONE * (uint64_t)KF_ROTATION_ONE;
    kf_quat_t quat;
    quat.x = _rotation.x;
    quat.y = _rotation.y;
    quat.z = _rotation.z;
    quat.w = square >= one ? 0 : (int32_t)kf_internal_sqrt( one - square );
    return quat;
}

kf_quat_t kf_internal_quat_normalize( kf_quat_t _quat )
{
    /* Quarter squares keep four near-unit terms within 64 bits. */
    uint64_t square = (uint64_t)( (int64_t)_quat.x * _quat.x ) / 4u + (uint64_t)( (int64_t)_quat.y * _quat.y ) / 4u + (uint64_t)( (int64_t)_quat.z * _quat.z ) / 4u + (uint64_t)( (int64_t)_quat.w * _quat.w ) / 4u;
    int64_t length = (int64_t)kf_internal_sqrt( square );
    kf_quat_t result;
    if( length == 0 )
    {
        result.x = 0;
        result.y = 0;
        result.z = 0;
        result.w = (int32_t)KF_ROTATION_ONE;
        return result;
    }
    result.x = kf_rotation_clamp_unit( (int64_t)_quat.x * ( KF_ROTATION_ONE / 2 ) / length );
    result.y = kf_rotation_clamp_unit( (int64_t)_quat.y * ( KF_ROTATION_ONE / 2 ) / length );
    result.z = kf_rotation_clamp_unit( (int64_t)_quat.z * ( KF_ROTATION_ONE / 2 ) / length );
    result.w = kf_rotation_clamp_unit( (int64_t)_quat.w * ( KF_ROTATION_ONE / 2 ) / length );
    return result;
}

kf_rotation_t kf_internal_rotation( kf_quat_t _quat )
{
    return kf_internal_rotation_unit( kf_internal_quat_normalize( _quat ) );
}

kf_rotation_t kf_internal_rotation_unit( kf_quat_t _quat )
{
    kf_bool_t negate = (kf_bool_t)( _quat.w < 0 );
    kf_rotation_t rotation;
    if( _quat.w == 0 )
    {
        /* Both signs describe a half turn; select the first nonzero positive component. */
        if( _quat.x != 0 )
        {
            negate = (kf_bool_t)( _quat.x < 0 );
        }
        else if( _quat.y != 0 )
        {
            negate = (kf_bool_t)( _quat.y < 0 );
        }
        else
        {
            negate = (kf_bool_t)( _quat.z < 0 );
        }
    }
    rotation.x = negate == KF_TRUE ? -_quat.x : _quat.x;
    rotation.y = negate == KF_TRUE ? -_quat.y : _quat.y;
    rotation.z = negate == KF_TRUE ? -_quat.z : _quat.z;
    /* Rounding near a half turn can leave the vector part a few units longer
       than one; shorten its largest component so the stored rotation is valid. */
    while( kf_rotation_valid( rotation ) == KF_FALSE )
    {
        int32_t * largest = &rotation.x;
        if( kf_internal_magnitude( rotation.y ) > kf_internal_magnitude( *largest ) )
        {
            largest = &rotation.y;
        }
        if( kf_internal_magnitude( rotation.z ) > kf_internal_magnitude( *largest ) )
        {
            largest = &rotation.z;
        }
        *largest += *largest > 0 ? -1 : 1;
    }
    return rotation;
}

kf_quat_t kf_internal_quat_multiply( kf_quat_t _left, kf_quat_t _right )
{
    int64_t w = (int64_t)_left.w * _right.w - (int64_t)_left.x * _right.x - (int64_t)_left.y * _right.y - (int64_t)_left.z * _right.z;
    int64_t x = (int64_t)_left.w * _right.x + (int64_t)_left.x * _right.w + (int64_t)_left.y * _right.z - (int64_t)_left.z * _right.y;
    int64_t y = (int64_t)_left.w * _right.y - (int64_t)_left.x * _right.z + (int64_t)_left.y * _right.w + (int64_t)_left.z * _right.x;
    int64_t z = (int64_t)_left.w * _right.z + (int64_t)_left.x * _right.y - (int64_t)_left.y * _right.x + (int64_t)_left.z * _right.w;
    kf_quat_t result;
    result.x = kf_rotation_clamp_unit( x / KF_ROTATION_ONE );
    result.y = kf_rotation_clamp_unit( y / KF_ROTATION_ONE );
    result.z = kf_rotation_clamp_unit( z / KF_ROTATION_ONE );
    result.w = kf_rotation_clamp_unit( w / KF_ROTATION_ONE );
    return kf_internal_quat_normalize( result );
}

kf_quat_t kf_internal_quat_conjugate( kf_quat_t _quat )
{
    kf_quat_t result;
    result.x = -_quat.x;
    result.y = -_quat.y;
    result.z = -_quat.z;
    result.w = _quat.w;
    return result;
}

static kf_quat_t kf_rotation_axis_angle( const int64_t _vector[3], int64_t _angle )
{
    kf_quat_t delta;
    int32_t sine;
    int32_t cosine;
    if( _angle == 0 )
    {
        delta.x = 0;
        delta.y = 0;
        delta.z = 0;
        delta.w = (int32_t)KF_ROTATION_ONE;
        return delta;
    }
    kf_internal_sin_cos( _angle / 2, &sine, &cosine );
    /* The axis is vector / angle; both share the same Q2.30 scale, so the
       component is vector * sine / angle. */
    delta.x = kf_rotation_clamp_unit( kf_internal_mul_div( _vector[0], sine, _angle, 0 ) );
    delta.y = kf_rotation_clamp_unit( kf_internal_mul_div( _vector[1], sine, _angle, 0 ) );
    delta.z = kf_rotation_clamp_unit( kf_internal_mul_div( _vector[2], sine, _angle, 0 ) );
    delta.w = cosine;
    return kf_internal_quat_normalize( delta );
}

static int64_t kf_rotation_vector_length( const int64_t _vector[3] )
{
    uint64_t maximum = kf_internal_magnitude( _vector[0] );
    uint32_t shift = 0;
    uint64_t square = 0;
    uint32_t axis;
    if( kf_internal_magnitude( _vector[1] ) > maximum )
    {
        maximum = kf_internal_magnitude( _vector[1] );
    }
    if( kf_internal_magnitude( _vector[2] ) > maximum )
    {
        maximum = kf_internal_magnitude( _vector[2] );
    }
    while( ( maximum >> shift ) >= ( UINT64_C(1) << 31 ) )
    {
        ++shift;
    }
    for( axis = 0; axis != 3u; ++axis )
    {
        uint64_t component = kf_internal_magnitude( _vector[axis] ) >> shift;
        square += component * component;
    }
    return (int64_t)( kf_internal_sqrt( square ) << shift );
}

kf_quat_t kf_internal_quat_rotate( kf_quat_t _quat, const int64_t _vector[3] )
{
    int64_t angle = kf_rotation_vector_length( _vector );
    if( angle == 0 )
    {
        return _quat;
    }
    return kf_internal_quat_multiply( kf_rotation_axis_angle( _vector, angle ), _quat );
}

static int64_t kf_rotation_dot( kf_quat_t _left, kf_quat_t _right )
{
    return (int64_t)_left.x * _right.x + (int64_t)_left.y * _right.y + (int64_t)_left.z * _right.z + (int64_t)_left.w * _right.w;
}

static kf_quat_t kf_rotation_align( kf_quat_t _from, kf_quat_t _to )
{
    if( kf_rotation_dot( _from, _to ) < 0 )
    {
        _to.x = -_to.x;
        _to.y = -_to.y;
        _to.z = -_to.z;
        _to.w = -_to.w;
    }
    return _to;
}

kf_quat_t kf_internal_quat_nlerp( kf_quat_t _from, kf_quat_t _to, uint64_t _fraction )
{
    int64_t weight = (int64_t)( _fraction >> 2 );
    kf_quat_t to;
    kf_quat_t result;
    if( _fraction == 0 || ( _from.x == _to.x && _from.y == _to.y && _from.z == _to.z && _from.w == _to.w ) )
    {
        return _from;
    }
    to = kf_rotation_align( _from, _to );
    result.x = (int32_t)( _from.x + ( (int64_t)to.x - _from.x ) * weight / KF_ROTATION_ONE );
    result.y = (int32_t)( _from.y + ( (int64_t)to.y - _from.y ) * weight / KF_ROTATION_ONE );
    result.z = (int32_t)( _from.z + ( (int64_t)to.z - _from.z ) * weight / KF_ROTATION_ONE );
    result.w = (int32_t)( _from.w + ( (int64_t)to.w - _from.w ) * weight / KF_ROTATION_ONE );
    return kf_internal_quat_normalize( result );
}

int64_t kf_internal_quat_chord( kf_quat_t _from, kf_quat_t _to )
{
    kf_quat_t to = kf_rotation_align( _from, _to );
    int64_t x = (int64_t)to.x - _from.x;
    int64_t y = (int64_t)to.y - _from.y;
    int64_t z = (int64_t)to.z - _from.z;
    int64_t w = (int64_t)to.w - _from.w;
    return (int64_t)kf_internal_sqrt( (uint64_t)( x * x ) + (uint64_t)( y * y ) + (uint64_t)( z * z ) + (uint64_t)( w * w ) );
}

void kf_internal_quat_rotation_vector( kf_quat_t _quat, int64_t _vector[3] )
{
    kf_quat_t quat = _quat.w < 0 ? (kf_quat_t){ -_quat.x, -_quat.y, -_quat.z, -_quat.w } : _quat;
    int64_t sine = (int64_t)kf_internal_sqrt( (uint64_t)( (int64_t)quat.x * quat.x ) + (uint64_t)( (int64_t)quat.y * quat.y ) + (uint64_t)( (int64_t)quat.z * quat.z ) );
    int64_t low = 0;
    int64_t high = KF_ROTATION_HALF_PI;
    int64_t angle;
    uint32_t iteration;
    if( sine == 0 )
    {
        _vector[0] = 0;
        _vector[1] = 0;
        _vector[2] = 0;
        return;
    }
    /* Bisection of the half angle atan2(|v|, w) in [0, pi/2]. */
    for( iteration = 0; iteration != 32u; ++iteration )
    {
        int64_t middle = low + ( high - low ) / 2;
        int32_t middleSine;
        int32_t middleCosine;
        kf_internal_sin_cos( middle, &middleSine, &middleCosine );
        if( (int64_t)middleSine * quat.w < (int64_t)middleCosine * sine )
        {
            low = middle;
        }
        else
        {
            high = middle;
        }
    }
    angle = low + high;
    _vector[0] = kf_internal_mul_div( quat.x, angle, sine * ( INT64_C(1) << 14 ), 0 );
    _vector[1] = kf_internal_mul_div( quat.y, angle, sine * ( INT64_C(1) << 14 ), 0 );
    _vector[2] = kf_internal_mul_div( quat.z, angle, sine * ( INT64_C(1) << 14 ), 0 );
}

kf_matrix_t kf_internal_matrix( kf_quat_t _quat )
{
    int64_t xx = (int64_t)_quat.x * _quat.x;
    int64_t yy = (int64_t)_quat.y * _quat.y;
    int64_t zz = (int64_t)_quat.z * _quat.z;
    int64_t xy = (int64_t)_quat.x * _quat.y;
    int64_t xz = (int64_t)_quat.x * _quat.z;
    int64_t yz = (int64_t)_quat.y * _quat.z;
    int64_t wx = (int64_t)_quat.w * _quat.x;
    int64_t wy = (int64_t)_quat.w * _quat.y;
    int64_t wz = (int64_t)_quat.w * _quat.z;
    int64_t half = KF_ROTATION_ONE / 2;
    kf_matrix_t matrix;
    matrix.m[0][0] = kf_rotation_clamp_unit( KF_ROTATION_ONE - ( yy + zz ) / half );
    matrix.m[0][1] = kf_rotation_clamp_unit( ( xy - wz ) / half );
    matrix.m[0][2] = kf_rotation_clamp_unit( ( xz + wy ) / half );
    matrix.m[1][0] = kf_rotation_clamp_unit( ( xy + wz ) / half );
    matrix.m[1][1] = kf_rotation_clamp_unit( KF_ROTATION_ONE - ( xx + zz ) / half );
    matrix.m[1][2] = kf_rotation_clamp_unit( ( yz - wx ) / half );
    matrix.m[2][0] = kf_rotation_clamp_unit( ( xz - wy ) / half );
    matrix.m[2][1] = kf_rotation_clamp_unit( ( yz + wx ) / half );
    matrix.m[2][2] = kf_rotation_clamp_unit( KF_ROTATION_ONE - ( xx + yy ) / half );
    return matrix;
}

static kf_fixed_t kf_rotation_round( int64_t _value )
{
    int64_t half = KF_ROTATION_ONE / 2;
    int64_t rounded = ( _value < 0 ? _value - half : _value + half ) / KF_ROTATION_ONE;
    if( rounded > INT32_MAX )
    {
        return INT32_MAX;
    }
    if( rounded < INT32_MIN )
    {
        return INT32_MIN;
    }
    return (kf_fixed_t)rounded;
}

kf_vec3_t kf_internal_matrix_apply( const kf_matrix_t * _matrix, kf_vec3_t _vector )
{
    kf_vec3_t result;
    result.x = kf_rotation_round( (int64_t)_matrix->m[0][0] * _vector.x + (int64_t)_matrix->m[0][1] * _vector.y + (int64_t)_matrix->m[0][2] * _vector.z );
    result.y = kf_rotation_round( (int64_t)_matrix->m[1][0] * _vector.x + (int64_t)_matrix->m[1][1] * _vector.y + (int64_t)_matrix->m[1][2] * _vector.z );
    result.z = kf_rotation_round( (int64_t)_matrix->m[2][0] * _vector.x + (int64_t)_matrix->m[2][1] * _vector.y + (int64_t)_matrix->m[2][2] * _vector.z );
    return result;
}

kf_vec3_t kf_internal_matrix_apply_transpose( const kf_matrix_t * _matrix, kf_vec3_t _vector )
{
    kf_vec3_t result;
    result.x = kf_rotation_round( (int64_t)_matrix->m[0][0] * _vector.x + (int64_t)_matrix->m[1][0] * _vector.y + (int64_t)_matrix->m[2][0] * _vector.z );
    result.y = kf_rotation_round( (int64_t)_matrix->m[0][1] * _vector.x + (int64_t)_matrix->m[1][1] * _vector.y + (int64_t)_matrix->m[2][1] * _vector.z );
    result.z = kf_rotation_round( (int64_t)_matrix->m[0][2] * _vector.x + (int64_t)_matrix->m[1][2] * _vector.y + (int64_t)_matrix->m[2][2] * _vector.z );
    return result;
}

static void kf_tensor_unpack( kf_tensor_t _tensor, int64_t _matrix[3][3] )
{
    _matrix[0][0] = _tensor.xx;
    _matrix[1][1] = _tensor.yy;
    _matrix[2][2] = _tensor.zz;
    _matrix[0][1] = _tensor.xy;
    _matrix[1][0] = _tensor.xy;
    _matrix[0][2] = _tensor.xz;
    _matrix[2][0] = _tensor.xz;
    _matrix[1][2] = _tensor.yz;
    _matrix[2][1] = _tensor.yz;
}

kf_tensor_t kf_internal_tensor_rotate( const kf_matrix_t * _matrix, kf_tensor_t _tensor )
{
    int64_t source[3][3];
    int64_t product[3][3];
    int64_t result[3][3];
    kf_tensor_t rotated;
    uint32_t row;
    uint32_t column;
    uint32_t inner;
    kf_tensor_unpack( _tensor, source );
    for( row = 0; row != 3u; ++row )
    {
        for( column = 0; column != 3u; ++column )
        {
            int64_t sum = 0;
            for( inner = 0; inner != 3u; ++inner )
            {
                sum += kf_internal_mul_shift( _matrix->m[row][inner], source[inner][column], 30 );
            }
            product[row][column] = sum;
        }
    }
    for( row = 0; row != 3u; ++row )
    {
        for( column = row; column != 3u; ++column )
        {
            int64_t sum = 0;
            for( inner = 0; inner != 3u; ++inner )
            {
                sum += kf_internal_mul_shift( product[row][inner], _matrix->m[column][inner], 30 );
            }
            result[row][column] = sum;
        }
    }
    rotated.xx = result[0][0];
    rotated.yy = result[1][1];
    rotated.zz = result[2][2];
    rotated.xy = result[0][1];
    rotated.xz = result[0][2];
    rotated.yz = result[1][2];
    return rotated;
}

/* Inverts a positive definite Q8.24 tensor into Q32.32. Diagonal scaling
   T = S N S, with S = sqrt(diag(T)), leaves a unit-diagonal Q12.20 matrix N
   whose cofactors and determinant fit 64 bits for any principal ratio. */
kf_tensor_t kf_internal_tensor_inverse( kf_tensor_t _tensor )
{
    int64_t diagonal[3] = { _tensor.xx, _tensor.yy, _tensor.zz };
    int64_t roots[3];
    int64_t xy;
    int64_t xz;
    int64_t yz;
    int64_t one = INT64_C(1) << 20;
    int64_t cofactors[6];
    int64_t determinant;
    kf_tensor_t inverse = { 0, 0, 0, 0, 0, 0 };
    uint32_t axis;
    for( axis = 0; axis != 3u; ++axis )
    {
        if( diagonal[axis] <= 0 )
        {
            return inverse;
        }
        /* Q16.16 square root of a Q8.24 value. */
        roots[axis] = (int64_t)kf_internal_sqrt( (uint64_t)diagonal[axis] << 8 );
        if( roots[axis] == 0 )
        {
            return inverse;
        }
    }
    xy = kf_internal_div_shift( _tensor.xy, roots[0] * roots[1], 28 );
    xz = kf_internal_div_shift( _tensor.xz, roots[0] * roots[2], 28 );
    yz = kf_internal_div_shift( _tensor.yz, roots[1] * roots[2], 28 );
    cofactors[0] = one * one - yz * yz;
    cofactors[1] = one * one - xz * xz;
    cofactors[2] = one * one - xy * xy;
    cofactors[3] = xz * yz - xy * one;
    cofactors[4] = xy * yz - xz * one;
    cofactors[5] = xy * xz - one * yz;
    /* Q24.40, using the unit diagonal of N. */
    determinant = cofactors[0] + kf_internal_mul_shift( xy, cofactors[3], 20 ) + kf_internal_mul_shift( xz, cofactors[4], 20 );
    if( determinant <= 0 )
    {
        inverse.xx = kf_internal_div_shift( INT64_C(1), diagonal[0], 56 );
        inverse.yy = kf_internal_div_shift( INT64_C(1), diagonal[1], 56 );
        inverse.zz = kf_internal_div_shift( INT64_C(1), diagonal[2], 56 );
        return inverse;
    }
    /* T^-1 = S^-1 (C / det) S^-1 in Q32.32, as one division of each cofactor. */
    inverse.xx = kf_internal_div_product( cofactors[0], determinant, roots[0] * roots[0], 64 );
    inverse.yy = kf_internal_div_product( cofactors[1], determinant, roots[1] * roots[1], 64 );
    inverse.zz = kf_internal_div_product( cofactors[2], determinant, roots[2] * roots[2], 64 );
    inverse.xy = kf_internal_div_product( cofactors[3], determinant, roots[0] * roots[1], 64 );
    inverse.xz = kf_internal_div_product( cofactors[4], determinant, roots[0] * roots[2], 64 );
    inverse.yz = kf_internal_div_product( cofactors[5], determinant, roots[1] * roots[2], 64 );
    return inverse;
}

/* Q32.32 tensor times a Q16.16 vector. */
void kf_internal_tensor_apply( const kf_tensor_t * _tensor, const int64_t _vector[3], int64_t _result[3] )
{
    _result[0] = kf_internal_mul_shift_term( _tensor->xx, _vector[0], 32 ) + kf_internal_mul_shift_term( _tensor->xy, _vector[1], 32 ) + kf_internal_mul_shift_term( _tensor->xz, _vector[2], 32 );
    _result[1] = kf_internal_mul_shift_term( _tensor->xy, _vector[0], 32 ) + kf_internal_mul_shift_term( _tensor->yy, _vector[1], 32 ) + kf_internal_mul_shift_term( _tensor->yz, _vector[2], 32 );
    _result[2] = kf_internal_mul_shift_term( _tensor->xz, _vector[0], 32 ) + kf_internal_mul_shift_term( _tensor->yz, _vector[1], 32 ) + kf_internal_mul_shift_term( _tensor->zz, _vector[2], 32 );
}

kf_bool_t kf_rotation_valid( kf_rotation_t _rotation )
{
    uint64_t square = (uint64_t)( (int64_t)_rotation.x * _rotation.x ) + (uint64_t)( (int64_t)_rotation.y * _rotation.y ) + (uint64_t)( (int64_t)_rotation.z * _rotation.z );
    return (kf_bool_t)( square <= (uint64_t)KF_ROTATION_ONE * (uint64_t)KF_ROTATION_ONE );
}

kf_rotation_t kf_rotation_from_yaw( kf_angle16_t _yaw )
{
    kf_quat_t quat;
    int32_t sine;
    int32_t cosine;
    kf_internal_sin_cos( (int64_t)_yaw * KF_INTERNAL_PI / (int64_t)KF_ANGLE16_TURN, &sine, &cosine );
    quat.x = 0;
    quat.y = sine;
    quat.z = 0;
    quat.w = cosine;
    return kf_internal_rotation( quat );
}

kf_rotation_t kf_rotation_from_axis_angle( kf_vec3_t _axis, kf_fixed_t _radians )
{
    int64_t vector[3] = { _axis.x, _axis.y, _axis.z };
    int64_t angle = (int64_t)_radians * ( KF_ROTATION_ONE / KF_FIXED_SCALE );
    kf_quat_t quat;
    kf_rotation_t identity = { 0, 0, 0 };
    uint64_t maximum = kf_internal_magnitude( vector[0] );
    uint32_t highBit;
    int64_t length;
    int32_t sine;
    int32_t cosine;
    uint32_t component;
    for( component = 1; component != 3u; ++component )
    {
        uint64_t magnitude = kf_internal_magnitude( vector[component] );
        if( magnitude > maximum )
        {
            maximum = magnitude;
        }
    }
    if( maximum == 0 )
    {
        return identity;
    }
    if( angle == 0 )
    {
        return identity;
    }
    /* Normalize direction independently of angle. A power-of-two scale puts
       even a one-raw-unit axis in Q30 precision before taking its length.
       Three components below 2^31 keep the squared length within uint64_t. */
    highBit = kf_rotation_high_bit( maximum );
    for( component = 0; component != 3u; ++component )
    {
        if( highBit <= 30u )
        {
            vector[component] *= INT64_C(1) << ( 30u - highBit );
        }
        else
        {
            vector[component] /= INT64_C(1) << ( highBit - 30u );
        }
    }
    length = kf_rotation_vector_length( vector );
    kf_internal_sin_cos( angle / 2, &sine, &cosine );
    quat.x = kf_rotation_clamp_unit( kf_internal_mul_div( vector[0], sine, length, 0 ) );
    quat.y = kf_rotation_clamp_unit( kf_internal_mul_div( vector[1], sine, length, 0 ) );
    quat.z = kf_rotation_clamp_unit( kf_internal_mul_div( vector[2], sine, length, 0 ) );
    quat.w = cosine;
    return kf_internal_rotation( quat );
}

kf_rotation_t kf_rotation_multiply( kf_rotation_t _left, kf_rotation_t _right )
{
    return kf_internal_rotation( kf_internal_quat_multiply( kf_internal_quat( _left ), kf_internal_quat( _right ) ) );
}

kf_rotation_t kf_rotation_inverse( kf_rotation_t _rotation )
{
    return kf_internal_rotation( kf_internal_quat_conjugate( kf_internal_quat( _rotation ) ) );
}

kf_vec3_t kf_rotation_apply( kf_rotation_t _rotation, kf_vec3_t _vector )
{
    kf_matrix_t matrix = kf_internal_matrix( kf_internal_quat( _rotation ) );
    return kf_internal_matrix_apply( &matrix, _vector );
}

kf_vec3_t kf_rotation_apply_inverse( kf_rotation_t _rotation, kf_vec3_t _vector )
{
    kf_matrix_t matrix = kf_internal_matrix( kf_internal_quat( _rotation ) );
    return kf_internal_matrix_apply_transpose( &matrix, _vector );
}
