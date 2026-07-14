#include "kinefix/kinefix.h"

#include <assert.h>
#include <ctype.h>
#include <limits.h>
#include <string.h>

#if defined(_MSC_VER)
#   define KF_THREAD_LOCAL __declspec(thread)
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
#   define KF_THREAD_LOCAL _Thread_local
#else
#   define KF_THREAD_LOCAL __thread
#endif

static KF_THREAD_LOCAL kf_fault_t * g_kf_fault = NULL;

static void kf_raise_bound( kf_fault_code_t code, const char * operation )
{
    if( g_kf_fault != NULL )
    {
        kf_fault_raise( g_kf_fault, code, operation );
    }
#ifndef NDEBUG
    assert( 0 && "kinefix deterministic arithmetic fault" );
#endif
}

static uint64_t kf_magnitude( int64_t value )
{
    return value < 0 ? (uint64_t)(-(value + 1)) + UINT64_C(1) : (uint64_t)value;
}

static int64_t kf_signed_magnitude( uint64_t value, int negative, const char * operation )
{
    const uint64_t limit = negative != 0 ? (UINT64_C(1) << 63) : (uint64_t)INT64_MAX;
    if( value > limit )
    {
        kf_raise_bound( KF_FAULT_OVERFLOW, operation );
        return 0;
    }
    if( negative == 0 )
    {
        return (int64_t)value;
    }
    if( value == (UINT64_C(1) << 63) )
    {
        return INT64_MIN;
    }
    return -(int64_t)value;
}

#if !defined(__SIZEOF_INT128__)
typedef struct kf_uint128_t
{
    uint64_t high;
    uint64_t low;
} kf_uint128_t;

static kf_uint128_t kf_multiply_u64( uint64_t left, uint64_t right )
{
    const uint64_t l0 = (uint32_t)left;
    const uint64_t l1 = left >> 32;
    const uint64_t r0 = (uint32_t)right;
    const uint64_t r1 = right >> 32;
    const uint64_t p00 = l0 * r0;
    const uint64_t p01 = l0 * r1;
    const uint64_t p10 = l1 * r0;
    const uint64_t p11 = l1 * r1;
    const uint64_t middle = (p00 >> 32) + (uint32_t)p01 + (uint32_t)p10;
    kf_uint128_t result;
    result.low = (p00 & UINT64_C(0xFFFFFFFF)) | (middle << 32);
    result.high = p11 + (p01 >> 32) + (p10 >> 32) + (middle >> 32);
    return result;
}

static int kf_divide_u128_u64( kf_uint128_t numerator, uint64_t divisor, uint64_t * quotient )
{
    uint64_t result;
    uint64_t remainder;
    int bit;
    if( divisor == 0 || numerator.high >= divisor )
    {
        return 0;
    }
    result = 0;
    remainder = numerator.high;
    for( bit = 63; bit >= 0; --bit )
    {
        const int carry = (remainder >> 63) != 0;
        const uint64_t shifted = (remainder << 1) | ((numerator.low >> bit) & UINT64_C(1));
        if( carry != 0 || shifted >= divisor )
        {
            remainder = shifted - divisor;
            result |= UINT64_C(1) << bit;
        }
        else
        {
            remainder = shifted;
        }
    }
    *quotient = result;
    return 1;
}
#endif

void kf_fault_clear( kf_fault_t * fault )
{
    if( fault == NULL ) return;
    fault->code = KF_FAULT_NONE;
    fault->operation = NULL;
}

void kf_fault_raise( kf_fault_t * fault, kf_fault_code_t code, const char * operation )
{
    if( fault != NULL && fault->code == KF_FAULT_NONE )
    {
        fault->code = code;
        fault->operation = operation;
    }
}

int kf_fault_failed( const kf_fault_t * fault )
{
    return fault != NULL && fault->code != KF_FAULT_NONE;
}

kf_fault_t * kf_fault_bind( kf_fault_t * fault )
{
    kf_fault_t * previous = g_kf_fault;
    g_kf_fault = fault;
    return previous;
}

void kf_fault_restore( kf_fault_t * previous )
{
    g_kf_fault = previous;
}

kf_fixed_t kf_fixed_from_int( int64_t value )
{
    if( value < INT32_MIN || value > INT32_MAX )
    {
        kf_raise_bound( KF_FAULT_OVERFLOW, "from_int" );
        return 0;
    }
    return value * KF_FIXED_SCALE;
}

kf_fixed_t kf_fixed_from_decimal( const char * value )
{
    return value == NULL ? kf_fixed_from_decimal_n( value, 0 ) : kf_fixed_from_decimal_n( value, strlen( value ) );
}

kf_fixed_t kf_fixed_from_decimal_n( const char * value, size_t size )
{
    size_t cursor;
    int negative;
    int has_digit;
    uint64_t integer;
    uint64_t fraction;
    uint64_t divisor;
    uint64_t raw;

    if( value == NULL || size == 0 )
    {
        kf_raise_bound( KF_FAULT_INVALID_CONFIGURATION, "from_decimal" );
        return 0;
    }
    cursor = 0;
    negative = 0;
    if( value[cursor] == '+' || value[cursor] == '-' )
    {
        negative = value[cursor] == '-';
        ++cursor;
    }
    integer = 0;
    has_digit = 0;
    while( cursor < size && isdigit( (unsigned char)value[cursor] ) != 0 )
    {
        const uint32_t digit = (uint32_t)(value[cursor] - '0');
        has_digit = 1;
        if( integer > (UINT64_MAX - digit) / UINT64_C(10) )
        {
            kf_raise_bound( KF_FAULT_OVERFLOW, "from_decimal" );
            return 0;
        }
        integer = integer * UINT64_C(10) + digit;
        ++cursor;
    }
    fraction = 0;
    divisor = 1;
    if( cursor < size && value[cursor] == '.' )
    {
        ++cursor;
        while( cursor < size && isdigit( (unsigned char)value[cursor] ) != 0 )
        {
            has_digit = 1;
            if( divisor <= UINT64_MAX / UINT64_C(10) )
            {
                fraction = fraction * UINT64_C(10) + (uint32_t)(value[cursor] - '0');
                divisor *= UINT64_C(10);
            }
            ++cursor;
        }
    }
    if( has_digit == 0 || cursor != size || integer > (uint64_t)INT32_MAX + (negative != 0 ? 1u : 0u) )
    {
        kf_raise_bound( KF_FAULT_INVALID_CONFIGURATION, "from_decimal" );
        return 0;
    }
    raw = integer << KF_FIXED_FRACTION_BITS;
    if( divisor > 1 )
    {
#if defined(__SIZEOF_INT128__)
        raw += (uint64_t)(((unsigned __int128)fraction << KF_FIXED_FRACTION_BITS) / divisor);
#else
        kf_uint128_t numerator;
        uint64_t fraction_raw = 0;
        numerator.high = fraction >> 32;
        numerator.low = fraction << 32;
        if( kf_divide_u128_u64( numerator, divisor, &fraction_raw ) == 0 )
        {
            kf_raise_bound( KF_FAULT_OVERFLOW, "from_decimal" );
            return 0;
        }
        raw += fraction_raw;
#endif
    }
    return kf_signed_magnitude( raw, negative, "from_decimal" );
}

double kf_fixed_to_double( kf_fixed_t value )
{
    return (double)value / (double)KF_FIXED_SCALE;
}

kf_fixed_t kf_fixed_neg( kf_fixed_t value )
{
    if( value == INT64_MIN )
    {
        kf_raise_bound( KF_FAULT_OVERFLOW, "negate" );
        return 0;
    }
    return -value;
}

kf_fixed_t kf_fixed_add( kf_fixed_t left, kf_fixed_t right )
{
    if( (right > 0 && left > INT64_MAX - right) || (right < 0 && left < INT64_MIN - right) )
    {
        kf_raise_bound( KF_FAULT_OVERFLOW, "add" );
        return 0;
    }
    return left + right;
}

kf_fixed_t kf_fixed_sub( kf_fixed_t left, kf_fixed_t right )
{
    if( (right < 0 && left > INT64_MAX + right) || (right > 0 && left < INT64_MIN + right) )
    {
        kf_raise_bound( KF_FAULT_OVERFLOW, "subtract" );
        return 0;
    }
    return left - right;
}

kf_fixed_t kf_fixed_mul( kf_fixed_t left, kf_fixed_t right )
{
    const int negative = (left < 0) != (right < 0);
    const uint64_t l = kf_magnitude( left );
    const uint64_t r = kf_magnitude( right );
    uint64_t shifted;
#if defined(__SIZEOF_INT128__)
    const unsigned __int128 product = (unsigned __int128)l * (unsigned __int128)r;
    const unsigned __int128 scaled = product >> KF_FIXED_FRACTION_BITS;
    if( scaled > ((unsigned __int128)UINT64_C(1) << 63) )
    {
        kf_raise_bound( KF_FAULT_OVERFLOW, "multiply" );
        return 0;
    }
    shifted = (uint64_t)scaled;
#else
    const kf_uint128_t product = kf_multiply_u64( l, r );
    if( (product.high >> 32) != 0 )
    {
        kf_raise_bound( KF_FAULT_OVERFLOW, "multiply" );
        return 0;
    }
    shifted = (product.high << 32) | (product.low >> 32);
#endif
    return kf_signed_magnitude( shifted, negative, "multiply" );
}

kf_fixed_t kf_fixed_div( kf_fixed_t left, kf_fixed_t right )
{
    const int negative = (left < 0) != (right < 0);
    const uint64_t l = kf_magnitude( left );
    const uint64_t r = kf_magnitude( right );
    uint64_t quotient;
    if( right == 0 )
    {
        kf_raise_bound( KF_FAULT_DIVISION_BY_ZERO, "divide" );
        return 0;
    }
#if defined(__SIZEOF_INT128__)
    {
        const unsigned __int128 numerator = (unsigned __int128)l << KF_FIXED_FRACTION_BITS;
        const unsigned __int128 divided = numerator / r;
        if( divided > ((unsigned __int128)UINT64_C(1) << 63) )
        {
            kf_raise_bound( KF_FAULT_OVERFLOW, "divide" );
            return 0;
        }
        quotient = (uint64_t)divided;
    }
#else
    {
        kf_uint128_t numerator;
        numerator.high = l >> 32;
        numerator.low = l << 32;
        if( kf_divide_u128_u64( numerator, r, &quotient ) == 0 )
        {
            kf_raise_bound( KF_FAULT_OVERFLOW, "divide" );
            return 0;
        }
    }
#endif
    return kf_signed_magnitude( quotient, negative, "divide" );
}

kf_fixed_t kf_fixed_abs( kf_fixed_t value ) { return value < 0 ? kf_fixed_neg( value ) : value; }
kf_fixed_t kf_fixed_min( kf_fixed_t left, kf_fixed_t right ) { return left < right ? left : right; }
kf_fixed_t kf_fixed_max( kf_fixed_t left, kf_fixed_t right ) { return left > right ? left : right; }
kf_fixed_t kf_fixed_clamp( kf_fixed_t value, kf_fixed_t minimum, kf_fixed_t maximum ) { return kf_fixed_max( minimum, kf_fixed_min( maximum, value ) ); }

kf_fixed_t kf_fixed_sqrt( kf_fixed_t value )
{
    kf_fixed_t estimate;
    uint32_t iteration;
    if( value < 0 )
    {
        kf_raise_bound( KF_FAULT_INVALID_SQUARE_ROOT, "sqrt" );
        return 0;
    }
    if( value == 0 ) return 0;
    estimate = value > KF_FIXED_SCALE ? value : KF_FIXED_SCALE;
    for( iteration = 0; iteration != 48; ++iteration )
    {
        estimate = kf_fixed_mul( kf_fixed_add( estimate, kf_fixed_div( value, estimate ) ), KF_FIXED_SCALE / 2 );
    }
    return estimate;
}

kf_fixed_t kf_fixed_wrap_angle( kf_fixed_t angle )
{
    while( angle > KF_FIXED_PI ) angle = kf_fixed_sub( angle, KF_FIXED_TWO_PI );
    while( angle < -KF_FIXED_PI ) angle = kf_fixed_add( angle, KF_FIXED_TWO_PI );
    return angle;
}

void kf_fixed_sin_cos( kf_fixed_t angle, kf_fixed_t * sine, kf_fixed_t * cosine )
{
    kf_fixed_t x = kf_fixed_wrap_angle( angle );
    int cosine_negative = 0;
    kf_fixed_t x2;
    kf_fixed_t sin_polynomial;
    kf_fixed_t cos_polynomial;
    const kf_fixed_t sin_c1 = kf_fixed_from_decimal( "-0.166666666666666666" );
    const kf_fixed_t sin_c2 = kf_fixed_from_decimal( "0.008333333333333333" );
    const kf_fixed_t sin_c3 = kf_fixed_from_decimal( "-0.000198412698412698" );
    const kf_fixed_t sin_c4 = kf_fixed_from_decimal( "0.000002755731922398" );
    const kf_fixed_t cos_c1 = kf_fixed_from_decimal( "-0.5" );
    const kf_fixed_t cos_c2 = kf_fixed_from_decimal( "0.041666666666666666" );
    const kf_fixed_t cos_c3 = kf_fixed_from_decimal( "-0.001388888888888888" );
    const kf_fixed_t cos_c4 = kf_fixed_from_decimal( "0.000024801587301587" );
    if( x > KF_FIXED_HALF_PI )
    {
        x = kf_fixed_sub( KF_FIXED_PI, x );
        cosine_negative = 1;
    }
    else if( x < -KF_FIXED_HALF_PI )
    {
        x = kf_fixed_sub( -KF_FIXED_PI, x );
        cosine_negative = 1;
    }
    x2 = kf_fixed_mul( x, x );
    sin_polynomial = kf_fixed_add( KF_FIXED_SCALE, kf_fixed_mul( x2,
        kf_fixed_add( sin_c1, kf_fixed_mul( x2, kf_fixed_add( sin_c2,
        kf_fixed_mul( x2, kf_fixed_add( sin_c3, kf_fixed_mul( x2, sin_c4 ) ) ) ) ) ) ) );
    cos_polynomial = kf_fixed_add( KF_FIXED_SCALE, kf_fixed_mul( x2,
        kf_fixed_add( cos_c1, kf_fixed_mul( x2, kf_fixed_add( cos_c2,
        kf_fixed_mul( x2, kf_fixed_add( cos_c3, kf_fixed_mul( x2, cos_c4 ) ) ) ) ) ) ) );
    if( sine != NULL ) *sine = kf_fixed_mul( x, sin_polynomial );
    if( cosine != NULL ) *cosine = cosine_negative != 0 ? kf_fixed_neg( cos_polynomial ) : cos_polynomial;
}

kf_vec3_t kf_vec3_add( kf_vec3_t left, kf_vec3_t right )
{
    kf_vec3_t result = {kf_fixed_add( left.x, right.x ), kf_fixed_add( left.y, right.y ), kf_fixed_add( left.z, right.z )};
    return result;
}

kf_vec3_t kf_vec3_sub( kf_vec3_t left, kf_vec3_t right )
{
    kf_vec3_t result = {kf_fixed_sub( left.x, right.x ), kf_fixed_sub( left.y, right.y ), kf_fixed_sub( left.z, right.z )};
    return result;
}

kf_vec3_t kf_vec3_mul( kf_vec3_t value, kf_fixed_t scalar )
{
    kf_vec3_t result = {kf_fixed_mul( value.x, scalar ), kf_fixed_mul( value.y, scalar ), kf_fixed_mul( value.z, scalar )};
    return result;
}

kf_vec3_t kf_vec3_div( kf_vec3_t value, kf_fixed_t scalar )
{
    kf_vec3_t result = {kf_fixed_div( value.x, scalar ), kf_fixed_div( value.y, scalar ), kf_fixed_div( value.z, scalar )};
    return result;
}

kf_fixed_t kf_vec3_dot( kf_vec3_t left, kf_vec3_t right )
{
    return kf_fixed_add( kf_fixed_add( kf_fixed_mul( left.x, right.x ), kf_fixed_mul( left.y, right.y ) ), kf_fixed_mul( left.z, right.z ) );
}

kf_vec3_t kf_vec3_cross( kf_vec3_t left, kf_vec3_t right )
{
    kf_vec3_t result = {
        kf_fixed_sub( kf_fixed_mul( left.y, right.z ), kf_fixed_mul( left.z, right.y ) ),
        kf_fixed_sub( kf_fixed_mul( left.z, right.x ), kf_fixed_mul( left.x, right.z ) ),
        kf_fixed_sub( kf_fixed_mul( left.x, right.y ), kf_fixed_mul( left.y, right.x ) )};
    return result;
}

kf_fixed_t kf_vec3_length_squared( kf_vec3_t value ) { return kf_vec3_dot( value, value ); }
kf_fixed_t kf_vec3_length( kf_vec3_t value ) { return kf_fixed_sqrt( kf_vec3_length_squared( value ) ); }

kf_vec3_t kf_vec3_normalize( kf_vec3_t value )
{
    const kf_fixed_t length = kf_vec3_length( value );
    const kf_vec3_t zero = {0, 0, 0};
    return length == 0 ? zero : kf_vec3_div( value, length );
}

kf_fixed_t kf_segment_point_distance_squared( kf_vec3_t start, kf_vec3_t end, kf_vec3_t point )
{
    const kf_vec3_t segment = kf_vec3_sub( end, start );
    const kf_fixed_t length_squared = kf_vec3_length_squared( segment );
    kf_fixed_t t;
    kf_vec3_t closest;
    if( length_squared == 0 ) return kf_vec3_length_squared( kf_vec3_sub( point, start ) );
    t = kf_fixed_clamp( kf_fixed_div( kf_vec3_dot( kf_vec3_sub( point, start ), segment ), length_squared ), 0, KF_FIXED_SCALE );
    closest = kf_vec3_add( start, kf_vec3_mul( segment, t ) );
    return kf_vec3_length_squared( kf_vec3_sub( point, closest ) );
}

uint64_t kf_splitmix64( uint64_t value )
{
    value += UINT64_C(0x9E3779B97F4A7C15);
    value = (value ^ (value >> 30)) * UINT64_C(0xBF58476D1CE4E5B9);
    value = (value ^ (value >> 27)) * UINT64_C(0x94D049BB133111EB);
    return value ^ (value >> 31);
}

void kf_pcg32_seed( kf_pcg32_t * random, uint64_t seed, uint64_t stream )
{
    if( random == NULL ) return;
    random->state = 0;
    random->increment = (stream << 1u) | UINT64_C(1);
    (void)kf_pcg32_next( random );
    random->state += seed;
    (void)kf_pcg32_next( random );
}

uint32_t kf_pcg32_next( kf_pcg32_t * random )
{
    const uint64_t old = random->state;
    const uint32_t shifted = (uint32_t)(((old >> 18u) ^ old) >> 27u);
    const uint32_t rotation = (uint32_t)(old >> 59u);
    random->state = old * UINT64_C(6364136223846793005) + random->increment;
    return (shifted >> rotation) | (shifted << ((0u - rotation) & 31u));
}

uint32_t kf_pcg32_bounded( kf_pcg32_t * random, uint32_t bound )
{
    uint32_t threshold;
    if( bound == 0 ) return 0;
    threshold = (0u - bound) % bound;
    for( ;; )
    {
        const uint32_t value = kf_pcg32_next( random );
        if( value >= threshold ) return value % bound;
    }
}

void kf_pcg32_restore( kf_pcg32_t * random, uint64_t state, uint64_t stream )
{
    if( random == NULL ) return;
    random->state = state;
    random->increment = stream | UINT64_C(1);
}

static kf_vec3_t kf_normal_axis( size_t axis, int sign )
{
    kf_vec3_t normal = {0, 0, 0};
    const kf_fixed_t value = sign < 0 ? -KF_FIXED_SCALE : KF_FIXED_SCALE;
    if( axis == 0 ) normal.x = value;
    else if( axis == 1 ) normal.y = value;
    else normal.z = value;
    return normal;
}

static int kf_sweep_point_bounds( kf_vec3_t origin, kf_vec3_t displacement,
    const kf_fixed_t minimum[3], const kf_fixed_t maximum[3], uint32_t object_id, kf_hit_t * hit )
{
    const kf_fixed_t origins[3] = {origin.x, origin.y, origin.z};
    const kf_fixed_t deltas[3] = {displacement.x, displacement.y, displacement.z};
    kf_fixed_t entry = 0;
    kf_fixed_t exit = KF_FIXED_SCALE;
    size_t entry_axis = 3;
    int entry_sign = 0;
    int started_inside = 1;
    size_t axis;

    for( axis = 0; axis != 3; ++axis )
    {
        const kf_fixed_t end = kf_fixed_add( origins[axis], deltas[axis] );
        kf_fixed_t axis_entry = 0;
        kf_fixed_t axis_exit = KF_FIXED_SCALE;
        int axis_sign = 0;

        if( origins[axis] <= minimum[axis] || origins[axis] >= maximum[axis] ) started_inside = 0;

        if( deltas[axis] > 0 )
        {
            if( origins[axis] > maximum[axis] || end < minimum[axis] ) return 0;
            if( origins[axis] < minimum[axis] )
            {
                axis_entry = kf_fixed_div( kf_fixed_sub( minimum[axis], origins[axis] ), deltas[axis] );
                axis_sign = -1;
            }
            else if( origins[axis] == minimum[axis] )
            {
                axis_sign = -1;
            }
            if( end > maximum[axis] ) axis_exit = kf_fixed_div( kf_fixed_sub( maximum[axis], origins[axis] ), deltas[axis] );
        }
        else if( deltas[axis] < 0 )
        {
            if( origins[axis] < minimum[axis] || end > maximum[axis] ) return 0;
            if( origins[axis] > maximum[axis] )
            {
                axis_entry = kf_fixed_div( kf_fixed_sub( maximum[axis], origins[axis] ), deltas[axis] );
                axis_sign = 1;
            }
            else if( origins[axis] == maximum[axis] )
            {
                axis_sign = 1;
            }
            if( end < minimum[axis] ) axis_exit = kf_fixed_div( kf_fixed_sub( minimum[axis], origins[axis] ), deltas[axis] );
        }
        else if( origins[axis] <= minimum[axis] || origins[axis] >= maximum[axis] )
        {
            return 0;
        }

        if( axis_entry > entry || (axis_entry == entry && entry_axis == 3 && axis_sign != 0) )
        {
            entry = axis_entry;
            entry_axis = axis;
            entry_sign = axis_sign;
        }
        exit = kf_fixed_min( exit, axis_exit );
        if( entry > exit ) return 0;
    }

    if( exit <= 0 || entry > KF_FIXED_SCALE ) return 0;
    if( hit != NULL )
    {
        memset( hit, 0, sizeof( *hit ) );
        hit->object_id = object_id;
        hit->fraction = kf_fixed_clamp( entry, 0, KF_FIXED_SCALE );
        hit->position = kf_vec3_add( origin, kf_vec3_mul( displacement, hit->fraction ) );
        hit->normal = entry_axis == 3 ? (kf_vec3_t){0, 0, 0} : kf_normal_axis( entry_axis, entry_sign );
        hit->started_inside = (uint8_t)started_inside;
    }
    return 1;
}

static int kf_hit_precedes( const kf_hit_t * candidate, const kf_hit_t * selected, int has_selected )
{
    return has_selected == 0 || candidate->fraction < selected->fraction ||
        (candidate->fraction == selected->fraction && candidate->object_id < selected->object_id);
}

static int kf_capsule_valid( const kf_capsule_t * capsule )
{
    if( capsule != NULL && capsule->radius >= 0 && capsule->height >= kf_fixed_mul( capsule->radius, kf_fixed_from_int( 2 ) ) ) return 1;
    kf_raise_bound( KF_FAULT_INVALID_CONFIGURATION, "capsule" );
    return 0;
}

int kf_overlap_aabb_aabb( const kf_aabb_t * left, const kf_aabb_t * right )
{
    if( left == NULL || right == NULL ) return 0;
    return left->maximum[0] > right->minimum[0] && left->minimum[0] < right->maximum[0] &&
        left->maximum[1] > right->minimum[1] && left->minimum[1] < right->maximum[1] &&
        left->maximum[2] > right->minimum[2] && left->minimum[2] < right->maximum[2];
}

int kf_overlap_sphere_aabb( const kf_sphere_t * sphere, const kf_aabb_t * box )
{
    kf_fixed_t dx;
    kf_fixed_t dy;
    kf_fixed_t dz;
    kf_fixed_t distance_squared;
    if( sphere == NULL || box == NULL || sphere->radius < 0 ) return 0;
    dx = sphere->position.x < box->minimum[0] ? kf_fixed_sub( box->minimum[0], sphere->position.x ) :
        (sphere->position.x > box->maximum[0] ? kf_fixed_sub( sphere->position.x, box->maximum[0] ) : 0);
    dy = sphere->position.y < box->minimum[1] ? kf_fixed_sub( box->minimum[1], sphere->position.y ) :
        (sphere->position.y > box->maximum[1] ? kf_fixed_sub( sphere->position.y, box->maximum[1] ) : 0);
    dz = sphere->position.z < box->minimum[2] ? kf_fixed_sub( box->minimum[2], sphere->position.z ) :
        (sphere->position.z > box->maximum[2] ? kf_fixed_sub( sphere->position.z, box->maximum[2] ) : 0);
    distance_squared = kf_fixed_add( kf_fixed_add( kf_fixed_mul( dx, dx ), kf_fixed_mul( dy, dy ) ), kf_fixed_mul( dz, dz ) );
    return distance_squared < kf_fixed_mul( sphere->radius, sphere->radius );
}

int kf_overlap_capsule_aabb( const kf_capsule_t * capsule, const kf_aabb_t * box )
{
    kf_fixed_t segment_minimum_y;
    kf_fixed_t segment_maximum_y;
    kf_fixed_t dx;
    kf_fixed_t dy;
    kf_fixed_t dz;
    kf_fixed_t distance_squared;
    if( box == NULL || kf_capsule_valid( capsule ) == 0 ) return 0;
    segment_minimum_y = kf_fixed_add( capsule->position.y, capsule->radius );
    segment_maximum_y = kf_fixed_sub( kf_fixed_add( capsule->position.y, capsule->height ), capsule->radius );
    dx = capsule->position.x < box->minimum[0] ? kf_fixed_sub( box->minimum[0], capsule->position.x ) :
        (capsule->position.x > box->maximum[0] ? kf_fixed_sub( capsule->position.x, box->maximum[0] ) : 0);
    dy = segment_maximum_y < box->minimum[1] ? kf_fixed_sub( box->minimum[1], segment_maximum_y ) :
        (segment_minimum_y > box->maximum[1] ? kf_fixed_sub( segment_minimum_y, box->maximum[1] ) : 0);
    dz = capsule->position.z < box->minimum[2] ? kf_fixed_sub( box->minimum[2], capsule->position.z ) :
        (capsule->position.z > box->maximum[2] ? kf_fixed_sub( capsule->position.z, box->maximum[2] ) : 0);
    distance_squared = kf_fixed_add( kf_fixed_add( kf_fixed_mul( dx, dx ), kf_fixed_mul( dy, dy ) ), kf_fixed_mul( dz, dz ) );
    return distance_squared < kf_fixed_mul( capsule->radius, capsule->radius );
}

int kf_overlap_sphere_capsule( const kf_sphere_t * sphere, const kf_capsule_t * capsule )
{
    kf_fixed_t segment_minimum_y;
    kf_fixed_t segment_maximum_y;
    kf_fixed_t closest_y;
    kf_fixed_t combined_radius;
    kf_vec3_t closest;
    if( sphere == NULL || sphere->radius < 0 || kf_capsule_valid( capsule ) == 0 ) return 0;
    segment_minimum_y = kf_fixed_add( capsule->position.y, capsule->radius );
    segment_maximum_y = kf_fixed_sub( kf_fixed_add( capsule->position.y, capsule->height ), capsule->radius );
    closest_y = kf_fixed_clamp( sphere->position.y, segment_minimum_y, segment_maximum_y );
    closest = (kf_vec3_t){capsule->position.x, closest_y, capsule->position.z};
    combined_radius = kf_fixed_add( sphere->radius, capsule->radius );
    return kf_vec3_length_squared( kf_vec3_sub( sphere->position, closest ) ) < kf_fixed_mul( combined_radius, combined_radius );
}

int kf_world_overlap_capsule( const kf_aabb_t * boxes, size_t count, const kf_capsule_t * capsule, uint32_t * brush_id )
{
    size_t index;
    int found = 0;
    uint32_t selected_id = UINT32_MAX;
    for( index = 0; index != count; ++index )
    {
        if( kf_overlap_capsule_aabb( capsule, boxes + index ) == 0 ) continue;
        if( found == 0 || boxes[index].id < selected_id )
        {
            found = 1;
            selected_id = boxes[index].id;
        }
    }
    if( found != 0 && brush_id != NULL ) *brush_id = selected_id;
    return found;
}

int kf_raycast_aabb( const kf_ray_t * ray, const kf_aabb_t * box, kf_hit_t * hit )
{
    kf_vec3_t direction;
    kf_vec3_t displacement;
    int result;
    if( ray == NULL || box == NULL || ray->maximum_distance <= 0 ) return 0;
    direction = kf_vec3_normalize( ray->direction );
    if( kf_vec3_length_squared( direction ) == 0 ) return 0;
    displacement = kf_vec3_mul( direction, ray->maximum_distance );
    result = kf_sweep_point_bounds( ray->origin, displacement, box->minimum, box->maximum, box->id, hit );
    if( result != 0 && hit != NULL ) hit->distance = kf_fixed_mul( ray->maximum_distance, hit->fraction );
    return result;
}

int kf_world_raycast( const kf_aabb_t * boxes, size_t count, const kf_ray_t * ray, kf_hit_t * hit )
{
    size_t index;
    int found = 0;
    kf_hit_t selected;
    for( index = 0; index != count; ++index )
    {
        kf_hit_t candidate;
        if( kf_raycast_aabb( ray, boxes + index, &candidate ) == 0 ) continue;
        if( kf_hit_precedes( &candidate, &selected, found ) != 0 )
        {
            selected = candidate;
            found = 1;
        }
    }
    if( found != 0 && hit != NULL ) *hit = selected;
    return found;
}

int kf_sweep_sphere_aabb_hit( const kf_sphere_t * sphere, kf_vec3_t displacement, const kf_aabb_t * box, kf_hit_t * hit )
{
    kf_fixed_t minimum[3];
    kf_fixed_t maximum[3];
    int result;
    if( sphere == NULL || box == NULL || sphere->radius < 0 ) return 0;
    minimum[0] = kf_fixed_sub( box->minimum[0], sphere->radius );
    minimum[1] = kf_fixed_sub( box->minimum[1], sphere->radius );
    minimum[2] = kf_fixed_sub( box->minimum[2], sphere->radius );
    maximum[0] = kf_fixed_add( box->maximum[0], sphere->radius );
    maximum[1] = kf_fixed_add( box->maximum[1], sphere->radius );
    maximum[2] = kf_fixed_add( box->maximum[2], sphere->radius );
    result = kf_sweep_point_bounds( sphere->position, displacement, minimum, maximum, box->id, hit );
    if( result != 0 && hit != NULL ) hit->distance = kf_fixed_mul( kf_vec3_length( displacement ), hit->fraction );
    return result;
}

int kf_world_sweep_sphere_hit( const kf_aabb_t * boxes, size_t count, const kf_sphere_t * sphere,
    kf_vec3_t displacement, kf_hit_t * hit )
{
    size_t index;
    int found = 0;
    kf_hit_t selected;
    for( index = 0; index != count; ++index )
    {
        kf_hit_t candidate;
        if( kf_sweep_sphere_aabb_hit( sphere, displacement, boxes + index, &candidate ) == 0 ) continue;
        if( kf_hit_precedes( &candidate, &selected, found ) != 0 )
        {
            selected = candidate;
            found = 1;
        }
    }
    if( found != 0 && hit != NULL ) *hit = selected;
    return found;
}

static size_t kf_quadratic_roots( kf_fixed_t a, kf_fixed_t b, kf_fixed_t c, kf_fixed_t roots[2] )
{
    const kf_fixed_t two = KF_FIXED_SCALE * INT64_C(2);
    const kf_fixed_t four = KF_FIXED_SCALE * INT64_C(4);
    kf_fixed_t discriminant;
    kf_fixed_t square_root;
    kf_fixed_t denominator;
    if( a == 0 )
    {
        if( b == 0 ) return 0;
        roots[0] = kf_fixed_div( kf_fixed_neg( c ), b );
        return 1;
    }
    discriminant = kf_fixed_sub( kf_fixed_mul( b, b ), kf_fixed_mul( four, kf_fixed_mul( a, c ) ) );
    if( discriminant < 0 ) return 0;
    square_root = kf_fixed_sqrt( discriminant );
    denominator = kf_fixed_mul( two, a );
    roots[0] = kf_fixed_div( kf_fixed_sub( kf_fixed_neg( b ), square_root ), denominator );
    roots[1] = kf_fixed_div( kf_fixed_add( kf_fixed_neg( b ), square_root ), denominator );
    return 2;
}

static void kf_consider_capsule_root( kf_fixed_t root, kf_fixed_t maximum_distance,
    kf_fixed_t a, kf_fixed_t b, int restrict_y, kf_fixed_t y, kf_fixed_t minimum_y, kf_fixed_t maximum_y,
    int * found, kf_fixed_t * selected )
{
    const kf_fixed_t derivative = kf_fixed_add( kf_fixed_mul( kf_fixed_mul( KF_FIXED_SCALE * INT64_C(2), a ), root ), b );
    if( root < 0 || root > maximum_distance || derivative >= 0 ) return;
    if( restrict_y != 0 && (y < minimum_y || y > maximum_y) ) return;
    if( *found == 0 || root < *selected )
    {
        *found = 1;
        *selected = root;
    }
}

int kf_sweep_sphere_capsule( const kf_sphere_t * sphere, kf_vec3_t displacement,
    const kf_capsule_t * capsule, uint32_t object_id, kf_hit_t * hit )
{
    const kf_fixed_t two = KF_FIXED_SCALE * INT64_C(2);
    kf_vec3_t direction;
    kf_fixed_t length;
    kf_fixed_t combined_radius;
    kf_fixed_t radius_squared;
    kf_fixed_t segment_minimum_y;
    kf_fixed_t segment_maximum_y;
    kf_fixed_t roots[2];
    kf_fixed_t selected = 0;
    int found = 0;
    size_t root_count;
    size_t index;
    kf_vec3_t relative;
    kf_fixed_t a;
    kf_fixed_t b;
    kf_fixed_t c;
    kf_vec3_t centers[2];

    if( sphere == NULL || sphere->radius < 0 || kf_capsule_valid( capsule ) == 0 ) return 0;
    length = kf_vec3_length( displacement );
    if( length == 0 )
    {
        if( kf_overlap_sphere_capsule( sphere, capsule ) == 0 ) return 0;
        if( hit != NULL )
        {
            memset( hit, 0, sizeof( *hit ) );
            hit->object_id = object_id;
            hit->position = sphere->position;
            hit->started_inside = 1;
        }
        return 1;
    }
    direction = kf_vec3_div( displacement, length );
    combined_radius = kf_fixed_add( sphere->radius, capsule->radius );
    radius_squared = kf_fixed_mul( combined_radius, combined_radius );
    segment_minimum_y = kf_fixed_add( capsule->position.y, capsule->radius );
    segment_maximum_y = kf_fixed_sub( kf_fixed_add( capsule->position.y, capsule->height ), capsule->radius );

    if( kf_overlap_sphere_capsule( sphere, capsule ) != 0 )
    {
        selected = 0;
        found = 1;
    }
    else
    {
        relative = kf_vec3_sub( sphere->position, capsule->position );
        a = kf_fixed_add( kf_fixed_mul( direction.x, direction.x ), kf_fixed_mul( direction.z, direction.z ) );
        b = kf_fixed_mul( two, kf_fixed_add( kf_fixed_mul( relative.x, direction.x ), kf_fixed_mul( relative.z, direction.z ) ) );
        c = kf_fixed_sub( kf_fixed_add( kf_fixed_mul( relative.x, relative.x ), kf_fixed_mul( relative.z, relative.z ) ), radius_squared );
        root_count = kf_quadratic_roots( a, b, c, roots );
        for( index = 0; index != root_count; ++index )
        {
            const kf_fixed_t y = kf_fixed_add( sphere->position.y, kf_fixed_mul( direction.y, roots[index] ) );
            kf_consider_capsule_root( roots[index], length, a, b, 1, y, segment_minimum_y, segment_maximum_y, &found, &selected );
        }

        centers[0] = (kf_vec3_t){capsule->position.x, segment_minimum_y, capsule->position.z};
        centers[1] = (kf_vec3_t){capsule->position.x, segment_maximum_y, capsule->position.z};
        for( index = 0; index != 2; ++index )
        {
            relative = kf_vec3_sub( sphere->position, centers[index] );
            a = KF_FIXED_SCALE;
            b = kf_fixed_mul( two, kf_vec3_dot( relative, direction ) );
            c = kf_fixed_sub( kf_vec3_length_squared( relative ), radius_squared );
            root_count = kf_quadratic_roots( a, b, c, roots );
            if( root_count != 0 ) kf_consider_capsule_root( roots[0], length, a, b, 0, 0, 0, 0, &found, &selected );
        }
    }

    if( found == 0 ) return 0;
    if( hit != NULL )
    {
        kf_fixed_t closest_y;
        kf_vec3_t closest;
        memset( hit, 0, sizeof( *hit ) );
        hit->object_id = object_id;
        hit->distance = selected;
        hit->fraction = kf_fixed_div( selected, length );
        hit->position = kf_vec3_add( sphere->position, kf_vec3_mul( direction, selected ) );
        closest_y = kf_fixed_clamp( hit->position.y, segment_minimum_y, segment_maximum_y );
        closest = (kf_vec3_t){capsule->position.x, closest_y, capsule->position.z};
        hit->normal = kf_vec3_normalize( kf_vec3_sub( hit->position, closest ) );
        if( kf_vec3_length_squared( hit->normal ) == 0 ) hit->normal = kf_vec3_mul( direction, -KF_FIXED_SCALE );
        hit->started_inside = (uint8_t)(selected == 0 && kf_overlap_sphere_capsule( sphere, capsule ) != 0);
    }
    return 1;
}

int kf_raycast_capsule( const kf_ray_t * ray, const kf_capsule_t * capsule, uint32_t object_id, kf_hit_t * hit )
{
    kf_vec3_t direction;
    kf_sphere_t point;
    if( ray == NULL || ray->maximum_distance <= 0 || kf_capsule_valid( capsule ) == 0 ) return 0;
    direction = kf_vec3_normalize( ray->direction );
    if( kf_vec3_length_squared( direction ) == 0 ) return 0;
    point.position = ray->origin;
    point.radius = 0;
    return kf_sweep_sphere_capsule( &point, kf_vec3_mul( direction, ray->maximum_distance ), capsule, object_id, hit );
}

int kf_sweep_capsule_aabb( const kf_capsule_t * capsule, kf_vec3_t displacement, const kf_aabb_t * box, kf_hit_t * hit )
{
    kf_fixed_t minimum[3];
    kf_fixed_t maximum[3];
    int result;
    if( box == NULL || kf_capsule_valid( capsule ) == 0 ) return 0;
    minimum[0] = kf_fixed_sub( box->minimum[0], capsule->radius );
    minimum[1] = kf_fixed_sub( box->minimum[1], capsule->height );
    minimum[2] = kf_fixed_sub( box->minimum[2], capsule->radius );
    maximum[0] = kf_fixed_add( box->maximum[0], capsule->radius );
    maximum[1] = box->maximum[1];
    maximum[2] = kf_fixed_add( box->maximum[2], capsule->radius );
    result = kf_sweep_point_bounds( capsule->position, displacement, minimum, maximum, box->id, hit );
    if( result != 0 && hit != NULL ) hit->distance = kf_fixed_mul( kf_vec3_length( displacement ), hit->fraction );
    return result;
}

int kf_world_sweep_capsule( const kf_aabb_t * boxes, size_t count, const kf_capsule_t * capsule,
    kf_vec3_t displacement, kf_hit_t * hit )
{
    size_t index;
    int found = 0;
    kf_hit_t selected;
    for( index = 0; index != count; ++index )
    {
        kf_hit_t candidate;
        if( kf_sweep_capsule_aabb( capsule, displacement, boxes + index, &candidate ) == 0 ) continue;
        if( kf_hit_precedes( &candidate, &selected, found ) != 0 )
        {
            selected = candidate;
            found = 1;
        }
    }
    if( found != 0 && hit != NULL ) *hit = selected;
    return found;
}

int kf_aabb_overlaps_character( const kf_aabb_t * box, kf_vec3_t position, kf_fixed_t half_width, kf_fixed_t height )
{
    const kf_capsule_t capsule = {position, half_width, height};
    return kf_overlap_capsule_aabb( &capsule, box );
}

int kf_world_character_collides( const kf_aabb_t * boxes, size_t count, kf_vec3_t position, kf_fixed_t half_width, kf_fixed_t height )
{
    const kf_capsule_t capsule = {position, half_width, height};
    return kf_world_overlap_capsule( boxes, count, &capsule, NULL );
}

int kf_world_find_step_top( const kf_aabb_t * boxes, size_t count, kf_vec3_t candidate,
    kf_fixed_t half_width, kf_fixed_t height, kf_fixed_t current_y, kf_fixed_t maximum_step, kf_fixed_t * top )
{
    size_t index;
    int found = 0;
    kf_fixed_t selected = current_y;
    for( index = 0; index != count; ++index )
    {
        const kf_aabb_t * box = boxes + index;
        const kf_fixed_t box_top = box->maximum[1];
        if( kf_aabb_overlaps_character( box, candidate, half_width, height ) == 0 ) continue;
        if( box_top <= current_y || kf_fixed_sub( box_top, current_y ) > maximum_step ) continue;
        selected = kf_fixed_max( selected, box_top );
        found = 1;
    }
    if( top != NULL ) *top = selected;
    return found;
}

int kf_sweep_sphere_aabb( kf_vec3_t start, kf_vec3_t end, kf_fixed_t radius, const kf_aabb_t * box, kf_fixed_t * hit_time )
{
    const kf_sphere_t sphere = {start, radius};
    kf_hit_t hit;
    const int result = kf_sweep_sphere_aabb_hit( &sphere, kf_vec3_sub( end, start ), box, &hit );
    if( result != 0 && hit_time != NULL ) *hit_time = hit.fraction;
    return result;
}

int kf_world_sweep_sphere( const kf_aabb_t * boxes, size_t count, kf_vec3_t start, kf_vec3_t end,
    kf_fixed_t radius, kf_fixed_t * hit_time, uint32_t * brush_id )
{
    const kf_sphere_t sphere = {start, radius};
    kf_hit_t hit;
    const int result = kf_world_sweep_sphere_hit( boxes, count, &sphere, kf_vec3_sub( end, start ), &hit );
    if( result != 0 )
    {
        if( hit_time != NULL ) *hit_time = hit.fraction;
        if( brush_id != NULL ) *brush_id = hit.object_id;
    }
    return result;
}

int kf_world_line_blocked( const kf_aabb_t * boxes, size_t count, kf_vec3_t start, kf_vec3_t end,
    kf_fixed_t minimum_time, kf_fixed_t maximum_time )
{
    const kf_vec3_t delta = kf_vec3_sub( end, start );
    const kf_fixed_t length = kf_vec3_length( delta );
    const kf_ray_t ray = {start, delta, length};
    kf_hit_t hit;
    if( length == 0 || kf_world_raycast( boxes, count, &ray, &hit ) == 0 ) return 0;
    return hit.fraction > minimum_time && hit.fraction < maximum_time;
}

static const kf_aabb_t * kf_world_find_box( const kf_aabb_t * boxes, size_t count, uint32_t id )
{
    size_t index;
    for( index = 0; index != count; ++index ) if( boxes[index].id == id ) return boxes + index;
    return NULL;
}

static kf_fixed_t kf_character_safe_fraction( kf_fixed_t fraction, kf_fixed_t distance, kf_fixed_t skin )
{
    kf_fixed_t margin;
    if( fraction <= 0 || distance <= 0 || skin <= 0 ) return kf_fixed_max( fraction, 0 );
    margin = kf_fixed_div( skin, distance );
    return kf_fixed_max( 0, kf_fixed_sub( fraction, margin ) );
}

static int kf_character_try_step( kf_character_body_t * body, const kf_character_config_t * config,
    const kf_aabb_t * boxes, size_t count, kf_vec3_t displacement, const kf_hit_t * blocking_hit )
{
    const kf_aabb_t * step = kf_world_find_box( boxes, count, blocking_hit->object_id );
    kf_capsule_t capsule;
    kf_vec3_t rise;
    kf_hit_t hit;
    kf_fixed_t step_height;
    if( step == NULL || body->grounded == 0 ) return 0;
    step_height = kf_fixed_sub( step->maximum[1], body->position.y );
    if( step_height <= 0 || step_height > config->step_height ) return 0;

    capsule.position = body->position;
    capsule.radius = config->radius;
    capsule.height = config->height;
    rise = (kf_vec3_t){0, step_height, 0};
    if( kf_world_sweep_capsule( boxes, count, &capsule, rise, &hit ) != 0 && hit.object_id != step->id ) return 0;
    capsule.position = kf_vec3_add( capsule.position, rise );
    if( kf_world_overlap_capsule( boxes, count, &capsule, NULL ) != 0 ) return 0;
    if( kf_world_sweep_capsule( boxes, count, &capsule, displacement, &hit ) != 0 ) return 0;
    body->position = kf_vec3_add( capsule.position, displacement );
    return 1;
}

static void kf_character_move_horizontal( kf_character_body_t * body, const kf_character_config_t * config,
    const kf_aabb_t * boxes, size_t count, kf_fixed_t delta, int x_axis, kf_character_result_t * result )
{
    kf_capsule_t capsule;
    kf_vec3_t displacement = {0, 0, 0};
    kf_hit_t hit;
    kf_fixed_t fraction;
    if( delta == 0 ) return;
    if( x_axis != 0 ) displacement.x = delta;
    else displacement.z = delta;
    capsule.position = body->position;
    capsule.radius = config->radius;
    capsule.height = config->height;
    if( kf_world_sweep_capsule( boxes, count, &capsule, displacement, &hit ) == 0 )
    {
        body->position = kf_vec3_add( body->position, displacement );
        return;
    }
    if( kf_character_try_step( body, config, boxes, count, displacement, &hit ) != 0 )
    {
        result->stepped = 1;
        return;
    }
    fraction = kf_character_safe_fraction( hit.fraction, kf_fixed_abs( delta ), config->support_epsilon );
    body->position = kf_vec3_add( body->position, kf_vec3_mul( displacement, fraction ) );
    if( result->blocked == 0 ) result->hit = hit;
    result->blocked = 1;
    if( x_axis != 0 ) body->velocity.x = 0;
    else body->velocity.z = 0;
}

void kf_character_step( kf_character_body_t * body, const kf_character_config_t * config,
    const kf_aabb_t * boxes, size_t count, kf_character_result_t * result )
{
    kf_vec3_t displacement;
    kf_capsule_t capsule;
    kf_vec3_t vertical_displacement;
    kf_hit_t vertical_hit;
    kf_fixed_t boundary;
    memset( result, 0, sizeof( *result ) );
    if( body == NULL || config == NULL || kf_capsule_valid( &(kf_capsule_t){body->position, config->radius, config->height} ) == 0 ) return;
    if( body->grounded == 0 )
    {
        body->velocity.y = kf_fixed_sub( body->velocity.y, kf_fixed_mul( config->gravity, config->tick_seconds ) );
    }
    result->previous_vertical_velocity = body->velocity.y;
    displacement = kf_vec3_mul( body->velocity, config->tick_seconds );
    kf_character_move_horizontal( body, config, boxes, count, displacement.x, 1, result );
    kf_character_move_horizontal( body, config, boxes, count, displacement.z, 0, result );

    capsule.position = body->position;
    capsule.radius = config->radius;
    capsule.height = config->height;
    vertical_displacement = (kf_vec3_t){0, displacement.y, 0};
    if( displacement.y != 0 && kf_world_sweep_capsule( boxes, count, &capsule, vertical_displacement, &vertical_hit ) != 0 )
    {
        body->position = kf_vec3_add( body->position, kf_vec3_mul( vertical_displacement, vertical_hit.fraction ) );
        result->landed = body->grounded == 0 && displacement.y < 0;
        result->hit_ceiling = displacement.y > 0;
        if( result->blocked == 0 ) result->hit = vertical_hit;
        result->blocked = 1;
        body->velocity.y = 0;
        body->grounded = displacement.y < 0;
    }
    else
    {
        kf_capsule_t support;
        body->position = kf_vec3_add( body->position, vertical_displacement );
        support.position = body->position;
        support.position.y = kf_fixed_sub( support.position.y, config->support_epsilon );
        support.radius = config->radius;
        support.height = config->height;
        body->grounded = (uint8_t)kf_world_overlap_capsule( boxes, count, &support, NULL );
    }

    boundary = kf_fixed_sub( config->arena_half_extent, config->radius );
    if( body->position.x < -boundary ) { body->position.x = -boundary; body->velocity.x = 0; }
    if( body->position.x > boundary ) { body->position.x = boundary; body->velocity.x = 0; }
    if( body->position.z < -boundary ) { body->position.z = -boundary; body->velocity.z = 0; }
    if( body->position.z > boundary ) { body->position.z = boundary; body->velocity.z = 0; }
}
