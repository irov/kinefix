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

int kf_aabb_overlaps_character( const kf_aabb_t * box, kf_vec3_t position, kf_fixed_t half_width, kf_fixed_t height )
{
    return kf_fixed_add( position.x, half_width ) > box->minimum[0] && kf_fixed_sub( position.x, half_width ) < box->maximum[0] &&
        kf_fixed_add( position.y, height ) > box->minimum[1] && position.y < box->maximum[1] &&
        kf_fixed_add( position.z, half_width ) > box->minimum[2] && kf_fixed_sub( position.z, half_width ) < box->maximum[2];
}

int kf_world_character_collides( const kf_aabb_t * boxes, size_t count, kf_vec3_t position, kf_fixed_t half_width, kf_fixed_t height )
{
    size_t index;
    for( index = 0; index != count; ++index )
    {
        if( kf_aabb_overlaps_character( boxes + index, position, half_width, height ) != 0 ) return 1;
    }
    return 0;
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
    const kf_vec3_t delta = kf_vec3_sub( end, start );
    const kf_fixed_t origins[3] = {start.x, start.y, start.z};
    const kf_fixed_t deltas[3] = {delta.x, delta.y, delta.z};
    kf_fixed_t entry = 0;
    kf_fixed_t exit = KF_FIXED_SCALE;
    const kf_fixed_t epsilon = kf_fixed_from_decimal( "0.000001" );
    size_t axis;
    for( axis = 0; axis != 3; ++axis )
    {
        const kf_fixed_t minimum = kf_fixed_sub( box->minimum[axis], radius );
        const kf_fixed_t maximum = kf_fixed_add( box->maximum[axis], radius );
        kf_fixed_t first;
        kf_fixed_t second;
        if( kf_fixed_abs( deltas[axis] ) <= epsilon )
        {
            if( origins[axis] < minimum || origins[axis] > maximum ) return 0;
            continue;
        }
        first = kf_fixed_div( kf_fixed_sub( minimum, origins[axis] ), deltas[axis] );
        second = kf_fixed_div( kf_fixed_sub( maximum, origins[axis] ), deltas[axis] );
        if( first > second )
        {
            const kf_fixed_t temporary = first;
            first = second;
            second = temporary;
        }
        entry = kf_fixed_max( entry, first );
        exit = kf_fixed_min( exit, second );
        if( entry > exit ) return 0;
    }
    if( exit <= 0 || entry > KF_FIXED_SCALE ) return 0;
    if( hit_time != NULL ) *hit_time = kf_fixed_clamp( entry, 0, KF_FIXED_SCALE );
    return 1;
}

int kf_world_sweep_sphere( const kf_aabb_t * boxes, size_t count, kf_vec3_t start, kf_vec3_t end,
    kf_fixed_t radius, kf_fixed_t * hit_time, uint32_t * brush_id )
{
    size_t index;
    int hit = 0;
    kf_fixed_t selected_time = KF_FIXED_SCALE;
    uint32_t selected_id = UINT32_MAX;
    for( index = 0; index != count; ++index )
    {
        kf_fixed_t current_time;
        if( kf_sweep_sphere_aabb( start, end, radius, boxes + index, &current_time ) == 0 ) continue;
        if( hit == 0 || current_time < selected_time || (current_time == selected_time && boxes[index].id < selected_id) )
        {
            hit = 1;
            selected_time = current_time;
            selected_id = boxes[index].id;
        }
    }
    if( hit != 0 )
    {
        if( hit_time != NULL ) *hit_time = selected_time;
        if( brush_id != NULL ) *brush_id = selected_id;
    }
    return hit;
}

int kf_world_line_blocked( const kf_aabb_t * boxes, size_t count, kf_vec3_t start, kf_vec3_t end,
    kf_fixed_t minimum_time, kf_fixed_t maximum_time )
{
    size_t index;
    for( index = 0; index != count; ++index )
    {
        kf_fixed_t hit_time;
        if( kf_sweep_sphere_aabb( start, end, 0, boxes + index, &hit_time ) != 0 &&
            hit_time > minimum_time && hit_time < maximum_time ) return 1;
    }
    return 0;
}

static void kf_character_move_horizontal( kf_character_body_t * body, const kf_character_config_t * config,
    const kf_aabb_t * boxes, size_t count, kf_fixed_t delta, int x_axis, kf_character_result_t * result )
{
    kf_vec3_t candidate;
    kf_fixed_t step_top;
    if( delta == 0 ) return;
    candidate = body->position;
    if( x_axis != 0 ) candidate.x = kf_fixed_add( candidate.x, delta );
    else candidate.z = kf_fixed_add( candidate.z, delta );
    if( kf_world_character_collides( boxes, count, candidate, config->half_width, config->height ) == 0 )
    {
        body->position = candidate;
        return;
    }
    if( body->grounded != 0 && kf_world_find_step_top( boxes, count, candidate, config->half_width, config->height,
        body->position.y, config->step_height, &step_top ) != 0 )
    {
        candidate.y = step_top;
        if( kf_world_character_collides( boxes, count, candidate, config->half_width, config->height ) == 0 )
        {
            body->position = candidate;
            result->stepped = 1;
            return;
        }
    }
    if( x_axis != 0 ) body->velocity.x = 0;
    else body->velocity.z = 0;
}

void kf_character_step( kf_character_body_t * body, const kf_character_config_t * config,
    const kf_aabb_t * boxes, size_t count, kf_character_result_t * result )
{
    kf_vec3_t displacement;
    kf_vec3_t vertical_candidate;
    kf_fixed_t resolved_y;
    int vertical_collision = 0;
    size_t index;
    memset( result, 0, sizeof( *result ) );
    if( body->grounded == 0 )
    {
        body->velocity.y = kf_fixed_sub( body->velocity.y, kf_fixed_mul( config->gravity, config->tick_seconds ) );
    }
    result->previous_vertical_velocity = body->velocity.y;
    displacement = kf_vec3_mul( body->velocity, config->tick_seconds );
    kf_character_move_horizontal( body, config, boxes, count, displacement.x, 1, result );
    kf_character_move_horizontal( body, config, boxes, count, displacement.z, 0, result );

    vertical_candidate = body->position;
    vertical_candidate.y = kf_fixed_add( vertical_candidate.y, displacement.y );
    resolved_y = vertical_candidate.y;
    for( index = 0; index != count; ++index )
    {
        if( kf_aabb_overlaps_character( boxes + index, vertical_candidate, config->half_width, config->height ) == 0 ) continue;
        vertical_collision = 1;
        if( displacement.y <= 0 ) resolved_y = kf_fixed_max( resolved_y, boxes[index].maximum[1] );
        else resolved_y = kf_fixed_min( resolved_y, kf_fixed_sub( boxes[index].minimum[1], config->height ) );
    }
    body->position.y = resolved_y;
    if( vertical_collision != 0 )
    {
        result->landed = body->grounded == 0 && displacement.y <= 0;
        result->hit_ceiling = displacement.y > 0;
        body->velocity.y = 0;
        body->grounded = displacement.y <= 0;
    }
    else
    {
        kf_vec3_t support_probe = body->position;
        support_probe.y = kf_fixed_sub( support_probe.y, config->support_epsilon );
        body->grounded = (uint8_t)kf_world_character_collides( boxes, count, support_probe, config->half_width, config->height );
    }
    if( body->position.x < -config->arena_half_extent ) { body->position.x = -config->arena_half_extent; body->velocity.x = 0; }
    if( body->position.x > config->arena_half_extent ) { body->position.x = config->arena_half_extent; body->velocity.x = 0; }
    if( body->position.z < -config->arena_half_extent ) { body->position.z = -config->arena_half_extent; body->velocity.z = 0; }
    if( body->position.z > config->arena_half_extent ) { body->position.z = config->arena_half_extent; body->velocity.z = 0; }
}
