#include "kinefix/kinefix.h"

#include <assert.h>
#include <limits.h>

static const kf_fixed_t g_kf_sine_quarter_q16[1025] =
{
#include "kinefix_sine_quarter_q16.inc"
};

#ifndef NDEBUG
#   define KF_DEBUG_REQUIRE(Condition, Message) assert( (Condition) && (Message) )
#else
#   define KF_DEBUG_REQUIRE(Condition, Message) ((void)0)
#endif

KF_FORCE_INLINE uint64_t __kf_magnitude( int64_t value )
{
    return value < 0 ? (uint64_t)(-(value + 1)) + UINT64_C(1) : (uint64_t)value;
}

typedef int64_t __kf_q32_t;

typedef struct __kf_uint128_t
{
    uint64_t high;
    uint64_t low;
} __kf_uint128_t;

KF_FORCE_INLINE __kf_uint128_t __kf_multiply_u64( uint64_t left, uint64_t right )
{
#if defined(__SIZEOF_INT128__)
    const unsigned __int128 product = (unsigned __int128)left * (unsigned __int128)right;
    const __kf_uint128_t result = {(uint64_t)(product >> 64), (uint64_t)product};
    return result;
#else
    const uint64_t l0 = (uint32_t)left;
    const uint64_t l1 = left >> 32;
    const uint64_t r0 = (uint32_t)right;
    const uint64_t r1 = right >> 32;
    const uint64_t p00 = l0 * r0;
    const uint64_t p01 = l0 * r1;
    const uint64_t p10 = l1 * r0;
    const uint64_t p11 = l1 * r1;
    const uint64_t middle = (p00 >> 32) + (uint32_t)p01 + (uint32_t)p10;
    const __kf_uint128_t result = {p11 + (p01 >> 32) + (p10 >> 32) + (middle >> 32), (p00 & UINT64_C(0xFFFFFFFF)) | (middle << 32)};
    return result;
#endif
}

KF_FORCE_INLINE int32_t __kf_compare_u128( __kf_uint128_t left, __kf_uint128_t right )
{
    if( left.high != right.high ) return left.high < right.high ? -1 : 1;
    if( left.low != right.low ) return left.low < right.low ? -1 : 1;
    return 0;
}

KF_FORCE_INLINE kf_bool_t __kf_add_u128( __kf_uint128_t left, __kf_uint128_t right, __kf_uint128_t * result )
{
    const uint64_t low = left.low + right.low;
    const uint64_t carry = (uint64_t)(low < left.low);
    const uint64_t high_without_carry = left.high + right.high;
    const uint64_t high = high_without_carry + carry;
    const kf_bool_t valid = (kf_bool_t)(high_without_carry >= left.high && high >= high_without_carry);
    KF_DEBUG_REQUIRE( valid == KF_TRUE, "kinefix unsigned 128-bit addition overflow" );
    if( valid == KF_FALSE )
    {
        *result = (__kf_uint128_t){0, 0};
        return KF_FALSE;
    }
    *result = (__kf_uint128_t){high, low};
    return KF_TRUE;
}

KF_FORCE_INLINE __kf_uint128_t __kf_sub_u128( __kf_uint128_t left, __kf_uint128_t right )
{
    __kf_uint128_t result;
    KF_DEBUG_REQUIRE( __kf_compare_u128( left, right ) >= 0, "kinefix unsigned 128-bit subtraction underflow" );
    result.low = left.low - right.low;
    result.high = left.high - right.high - (uint64_t)(left.low < right.low);
    return result;
}

KF_FORCE_INLINE kf_bool_t __kf_shift_left_two_u128( __kf_uint128_t value, __kf_uint128_t * result )
{
    const kf_bool_t valid = (kf_bool_t)((value.high >> 62) == 0);
    KF_DEBUG_REQUIRE( valid == KF_TRUE, "kinefix unsigned 128-bit shift overflow" );
    if( valid == KF_FALSE )
    {
        *result = (__kf_uint128_t){0, 0};
        return KF_FALSE;
    }
    *result = (__kf_uint128_t){(value.high << 2) | (value.low >> 62), value.low << 2};
    return KF_TRUE;
}

static kf_bool_t __kf_divide_u128_u64( __kf_uint128_t numerator, uint64_t divisor, uint64_t * quotient )
{
    uint64_t result;
    uint64_t remainder;
    int32_t bit;
    if( divisor == 0 || quotient == NULL || numerator.high >= divisor ) return KF_FALSE;
    result = 0;
    remainder = numerator.high;
    for( bit = 63; bit >= 0; --bit )
    {
        const kf_bool_t carry = (kf_bool_t)((remainder >> 63) != 0);
        const uint64_t shifted = (remainder << 1) | ((numerator.low >> bit) & UINT64_C(1));
        if( carry == KF_TRUE || shifted >= divisor )
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
    return KF_TRUE;
}

static uint64_t __kf_integer_sqrt_u128( __kf_uint128_t value )
{
    uint64_t result = 0;
    uint64_t bit = UINT64_C(1) << 63;
    while( bit != 0 )
    {
        const uint64_t candidate = result | bit;
        if( __kf_compare_u128( __kf_multiply_u64( candidate, candidate ), value ) <= 0 ) result = candidate;
        bit >>= 1;
    }
    return result;
}

KF_FORCE_INLINE kf_bool_t __kf_signed_sum_magnitude( uint64_t left, kf_bool_t left_negative, uint64_t right, kf_bool_t right_negative, uint64_t * magnitude, kf_bool_t * negative )
{
    if( left_negative == right_negative )
    {
        const uint64_t sum = left + right;
        if( sum < left ) return KF_FALSE;
        *magnitude = sum;
        *negative = left_negative;
        return KF_TRUE;
    }
    if( left >= right )
    {
        *magnitude = left - right;
        *negative = left == right ? KF_FALSE : left_negative;
    }
    else
    {
        *magnitude = right - left;
        *negative = right_negative;
    }
    return KF_TRUE;
}

static kf_bool_t __kf_ratio_magnitude_to_fixed( uint64_t numerator, kf_bool_t numerator_negative, uint64_t denominator, kf_bool_t denominator_negative, kf_fixed_t * result )
{
    __kf_uint128_t scaled;
    uint64_t quotient;
    uint64_t limit;
    kf_bool_t negative;
    if( denominator == 0 || result == NULL ) return KF_FALSE;
    scaled.high = numerator >> 48;
    scaled.low = numerator << KF_FIXED_FRACTION_BITS;
    if( __kf_divide_u128_u64( scaled, denominator, &quotient ) == KF_FALSE ) return KF_FALSE;
    negative = (kf_bool_t)(numerator_negative != denominator_negative && quotient != 0);
    limit = negative == KF_TRUE ? (UINT64_C(1) << 31) : (uint64_t)INT32_MAX;
    if( quotient > limit ) return KF_FALSE;
    if( negative == KF_FALSE ) *result = (kf_fixed_t)quotient;
    else if( quotient == (UINT64_C(1) << 31) ) *result = INT32_MIN;
    else *result = -(kf_fixed_t)quotient;
    return KF_TRUE;
}

static kf_bool_t __kf_ratio_magnitude_to_q32( uint64_t numerator, kf_bool_t numerator_negative, uint64_t denominator, kf_bool_t denominator_negative, __kf_q32_t * result )
{
    __kf_uint128_t scaled;
    uint64_t quotient;
    uint64_t limit;
    kf_bool_t negative;
    if( denominator == 0 || result == NULL ) return KF_FALSE;
    scaled.high = numerator >> 32;
    scaled.low = numerator << 32;
    if( __kf_divide_u128_u64( scaled, denominator, &quotient ) == KF_FALSE ) return KF_FALSE;
    negative = (kf_bool_t)(numerator_negative != denominator_negative && quotient != 0);
    limit = negative == KF_TRUE ? (UINT64_C(1) << 63) : (uint64_t)INT64_MAX;
    if( quotient > limit ) return KF_FALSE;
    if( negative == KF_FALSE ) *result = (int64_t)quotient;
    else if( quotient == (UINT64_C(1) << 63) ) *result = INT64_MIN;
    else *result = -(int64_t)quotient;
    return KF_TRUE;
}

KF_FORCE_INLINE kf_bool_t __kf_ratio_q32_to_fixed( __kf_q32_t numerator, __kf_q32_t denominator, kf_fixed_t * result )
{
    return __kf_ratio_magnitude_to_fixed( __kf_magnitude( numerator ), (kf_bool_t)(numerator < 0), __kf_magnitude( denominator ), (kf_bool_t)(denominator < 0), result );
}

KF_FORCE_INLINE kf_bool_t __kf_ratio_fixed_to_q32( kf_fixed_t numerator, kf_fixed_t denominator, __kf_q32_t * result )
{
    return __kf_ratio_magnitude_to_q32( __kf_magnitude( numerator ), (kf_bool_t)(numerator < 0), __kf_magnitude( denominator ), (kf_bool_t)(denominator < 0), result );
}

KF_FORCE_INLINE kf_fixed_t __kf_signed_magnitude( uint64_t value, kf_bool_t negative )
{
    const uint64_t limit = negative == KF_TRUE ? (UINT64_C(1) << 31) : (uint64_t)INT32_MAX;
    KF_DEBUG_REQUIRE( value <= limit, "kinefix fixed-point overflow" );
    if( value > limit )
    {
        return 0;
    }
    if( negative == KF_FALSE )
    {
        return (kf_fixed_t)value;
    }
    if( value == (UINT64_C(1) << 31) )
    {
        return INT32_MIN;
    }
    return -(kf_fixed_t)value;
}

/* A product of two Q16.16 raw values is a signed Q32.32 value. Keep compound
 * expressions in this representation and narrow only at the public Q16.16
 * result boundary. */
KF_FORCE_INLINE kf_fixed_t __kf_narrow_q32( int64_t value )
{
    const kf_bool_t negative = (kf_bool_t)(value < 0);
    const uint64_t magnitude = __kf_magnitude( value ) >> KF_FIXED_FRACTION_BITS;
    return __kf_signed_magnitude( magnitude, negative );
}

KF_FORCE_INLINE kf_bool_t __kf_wide_add( int64_t left, int64_t right, int64_t * result )
{
    const kf_bool_t valid = (kf_bool_t)((right <= 0 || left <= INT64_MAX - right) && (right >= 0 || left >= INT64_MIN - right));
    KF_DEBUG_REQUIRE( valid == KF_TRUE, "kinefix wide addition overflow" );
    if( valid == KF_FALSE )
    {
        *result = 0;
        return KF_FALSE;
    }
    *result = left + right;
    return KF_TRUE;
}

KF_FORCE_INLINE kf_bool_t __kf_wide_sub( int64_t left, int64_t right, int64_t * result )
{
    const kf_bool_t valid = (kf_bool_t)((right >= 0 || left <= INT64_MAX + right) && (right <= 0 || left >= INT64_MIN + right));
    KF_DEBUG_REQUIRE( valid == KF_TRUE, "kinefix wide subtraction overflow" );
    if( valid == KF_FALSE )
    {
        *result = 0;
        return KF_FALSE;
    }
    *result = left - right;
    return KF_TRUE;
}

KF_FORCE_INLINE kf_fixed_t __kf_fixed_madd( kf_fixed_t base, kf_fixed_t left, kf_fixed_t right )
{
    int64_t wide;
    if( __kf_wide_add( (int64_t)base * (int64_t)KF_FIXED_SCALE, (int64_t)left * (int64_t)right, &wide ) == KF_FALSE ) return 0;
    return __kf_narrow_q32( wide );
}

KF_FORCE_INLINE kf_fixed_t __kf_fixed_msub( kf_fixed_t base, kf_fixed_t left, kf_fixed_t right )
{
    int64_t wide;
    if( __kf_wide_sub( (int64_t)base * (int64_t)KF_FIXED_SCALE, (int64_t)left * (int64_t)right, &wide ) == KF_FALSE ) return 0;
    return __kf_narrow_q32( wide );
}

KF_FORCE_INLINE kf_bool_t __kf_fixed_madd_q32_wide( kf_fixed_t base, kf_fixed_t value, __kf_q32_t scalar, int64_t * result_q48 )
{
    uint64_t magnitude;
    kf_bool_t negative;
    const uint64_t base_magnitude = __kf_magnitude( base ) << 32;
    const uint64_t value_magnitude = __kf_magnitude( value );
    const uint64_t scalar_magnitude = __kf_magnitude( scalar );
    uint64_t product_magnitude;
    uint64_t limit;
    KF_DEBUG_REQUIRE( result_q48 != NULL, "kinefix Q32.32 multiply-add requires an output" );
    if( result_q48 == NULL ) return KF_FALSE;
    KF_DEBUG_REQUIRE( value_magnitude == 0 || scalar_magnitude <= UINT64_MAX / value_magnitude, "kinefix Q32.32 multiply-add product overflow" );
    if( value_magnitude != 0 && scalar_magnitude > UINT64_MAX / value_magnitude ) return KF_FALSE;
    product_magnitude = value_magnitude * scalar_magnitude;
    if( __kf_signed_sum_magnitude( base_magnitude, (kf_bool_t)(base < 0), product_magnitude, (kf_bool_t)((value < 0) != (scalar < 0)), &magnitude, &negative ) == KF_FALSE )
    {
        KF_DEBUG_REQUIRE( 0, "kinefix Q32.32 multiply-add overflow" );
        return KF_FALSE;
    }
    limit = negative == KF_TRUE ? (UINT64_C(1) << 63) : (uint64_t)INT64_MAX;
    KF_DEBUG_REQUIRE( magnitude <= limit, "kinefix Q32.32 multiply-add result overflow" );
    if( magnitude > limit ) return KF_FALSE;
    if( negative == KF_FALSE ) *result_q48 = (int64_t)magnitude;
    else if( magnitude == (UINT64_C(1) << 63) ) *result_q48 = INT64_MIN;
    else *result_q48 = -(int64_t)magnitude;
    return KF_TRUE;
}

KF_FORCE_INLINE kf_fixed_t __kf_fixed_madd_q32( kf_fixed_t base, kf_fixed_t value, __kf_q32_t scalar )
{
    int64_t wide_q48;
    if( __kf_fixed_madd_q32_wide( base, value, scalar, &wide_q48 ) == KF_FALSE ) return 0;
    return __kf_signed_magnitude( __kf_magnitude( wide_q48 ) >> 32, (kf_bool_t)(wide_q48 < 0) );
}

KF_FORCE_INLINE kf_fixed_t __kf_fixed_mul_div( kf_fixed_t value, kf_fixed_t multiplier, kf_fixed_t divisor )
{
    int64_t quotient;
    KF_DEBUG_REQUIRE( divisor != 0, "kinefix multiply-divide by zero" );
    if( divisor == 0 ) return 0;
    quotient = (int64_t)value * (int64_t)multiplier / (int64_t)divisor;
    KF_DEBUG_REQUIRE( quotient >= INT32_MIN && quotient <= INT32_MAX, "kinefix multiply-divide overflow" );
    if( quotient < INT32_MIN || quotient > INT32_MAX ) return 0;
    return (kf_fixed_t)quotient;
}

KF_FORCE_INLINE kf_vec3_t __kf_vec3_madd( kf_vec3_t base, kf_vec3_t value, kf_fixed_t scalar )
{
    const kf_vec3_t result = {__kf_fixed_madd( base.x, value.x, scalar ), __kf_fixed_madd( base.y, value.y, scalar ), __kf_fixed_madd( base.z, value.z, scalar )};
    return result;
}

KF_FORCE_INLINE kf_vec3_t __kf_vec3_madd_q32( kf_vec3_t base, kf_vec3_t value, __kf_q32_t scalar )
{
    const kf_vec3_t result = {__kf_fixed_madd_q32( base.x, value.x, scalar ), __kf_fixed_madd_q32( base.y, value.y, scalar ), __kf_fixed_madd_q32( base.z, value.z, scalar )};
    return result;
}

KF_FORCE_INLINE kf_vec3_t __kf_vec3_mul_div( kf_vec3_t value, kf_fixed_t multiplier, kf_fixed_t divisor )
{
    const kf_vec3_t result = {__kf_fixed_mul_div( value.x, multiplier, divisor ), __kf_fixed_mul_div( value.y, multiplier, divisor ), __kf_fixed_mul_div( value.z, multiplier, divisor )};
    return result;
}

kf_fixed_t kf_fixed_from_int( int64_t value )
{
    KF_DEBUG_REQUIRE( value >= INT16_MIN && value <= INT16_MAX, "kinefix integer conversion overflow" );
    if( value < INT16_MIN || value > INT16_MAX )
    {
        return 0;
    }
    return (kf_fixed_t)(value * KF_FIXED_SCALE);
}

kf_fixed_t kf_fixed_from_float( float value )
{
    const double scaled = (double)value * (double)KF_FIXED_SCALE;
    KF_DEBUG_REQUIRE( scaled == scaled, "kinefix float conversion requires a number" );
    if( scaled != scaled )
    {
        return 0;
    }
    KF_DEBUG_REQUIRE( scaled >= (double)INT32_MIN && scaled <= (double)INT32_MAX, "kinefix float conversion overflow" );
    if( scaled < (double)INT32_MIN || scaled > (double)INT32_MAX )
    {
        return 0;
    }
    return (kf_fixed_t)scaled;
}

double kf_fixed_to_double( kf_fixed_t value )
{
    return (double)value / (double)KF_FIXED_SCALE;
}

kf_fixed_t kf_fixed_neg( kf_fixed_t value )
{
    KF_DEBUG_REQUIRE( value != INT32_MIN, "kinefix negation overflow" );
    if( value == INT32_MIN )
    {
        return 0;
    }
    return -value;
}

kf_fixed_t kf_fixed_add( kf_fixed_t left, kf_fixed_t right )
{
    const int64_t result = (int64_t)left + (int64_t)right;
    KF_DEBUG_REQUIRE( result >= INT32_MIN && result <= INT32_MAX, "kinefix addition overflow" );
    if( result < INT32_MIN || result > INT32_MAX )
    {
        return 0;
    }
    return (kf_fixed_t)result;
}

kf_fixed_t kf_fixed_sub( kf_fixed_t left, kf_fixed_t right )
{
    const int64_t result = (int64_t)left - (int64_t)right;
    KF_DEBUG_REQUIRE( result >= INT32_MIN && result <= INT32_MAX, "kinefix subtraction overflow" );
    if( result < INT32_MIN || result > INT32_MAX )
    {
        return 0;
    }
    return (kf_fixed_t)result;
}

kf_fixed_t kf_fixed_mul( kf_fixed_t left, kf_fixed_t right )
{
    return __kf_narrow_q32( (int64_t)left * (int64_t)right );
}

static kf_bool_t __kf_fixed_try_div( kf_fixed_t left, kf_fixed_t right, kf_fixed_t * result )
{
    return __kf_ratio_q32_to_fixed( left, right, result );
}

kf_fixed_t kf_fixed_div( kf_fixed_t left, kf_fixed_t right )
{
    kf_fixed_t result;
    KF_DEBUG_REQUIRE( right != 0, "kinefix division by zero" );
    if( right == 0 )
    {
        return 0;
    }
    if( __kf_fixed_try_div( left, right, &result ) == KF_TRUE ) return result;
    KF_DEBUG_REQUIRE( 0, "kinefix division overflow" );
    return 0;
}

static uint64_t __kf_integer_sqrt_u64( uint64_t value )
{
    uint64_t result = 0;
    uint64_t bit = UINT64_C(1) << 62;
    while( bit > value ) bit >>= 2;
    while( bit != 0 )
    {
        if( value >= result + bit )
        {
            value -= result + bit;
            result = (result >> 1) + bit;
        }
        else
        {
            result >>= 1;
        }
        bit >>= 2;
    }
    return result;
}

kf_fixed_t kf_fixed_sqrt( kf_fixed_t value )
{
    KF_DEBUG_REQUIRE( value >= 0, "kinefix square root argument must be non-negative" );

    return (kf_fixed_t)__kf_integer_sqrt_u64( (uint64_t)(uint32_t)value * (uint64_t)KF_FIXED_SCALE );
}

kf_fixed_t kf_fixed_wrap_angle( kf_fixed_t angle )
{
    angle %= KF_FIXED_TWO_PI;
    if( angle > KF_FIXED_PI ) angle = kf_fixed_sub( angle, KF_FIXED_TWO_PI );
    if( angle < -KF_FIXED_PI ) angle = kf_fixed_add( angle, KF_FIXED_TWO_PI );
    return angle;
}

KF_FORCE_INLINE int64_t __kf_divide_round_nearest( int64_t numerator, int64_t denominator )
{
    const kf_bool_t negative = (kf_bool_t)(numerator < 0);
    const uint64_t magnitude = __kf_magnitude( numerator );
    const uint64_t rounded = (magnitude + (uint64_t)denominator / UINT64_C(2)) / (uint64_t)denominator;
    return negative == KF_TRUE ? -(int64_t)rounded : (int64_t)rounded;
}

kf_angle16_t kf_angle16_from_fixed_radians( kf_fixed_t radians )
{
    const int64_t wrapped = radians % KF_FIXED_TWO_PI;
    const int64_t scaled = wrapped * (int64_t)KF_ANGLE16_TURN;
    const int64_t ticks = __kf_divide_round_nearest( scaled, KF_FIXED_TWO_PI );
    return (kf_angle16_t)ticks;
}

kf_sangle16_t kf_sangle16_from_fixed_radians( kf_fixed_t radians )
{
    const kf_angle16_t wrapped = kf_angle16_from_fixed_radians( radians );
    if( wrapped <= (kf_angle16_t)INT16_MAX ) return (kf_sangle16_t)wrapped;
    return (kf_sangle16_t)((int32_t)wrapped - (int32_t)KF_ANGLE16_TURN);
}

kf_fixed_t kf_angle16_to_fixed_radians( kf_angle16_t angle )
{
    return (kf_fixed_t)(((int64_t)angle * KF_FIXED_TWO_PI) / (int64_t)KF_ANGLE16_TURN);
}

kf_fixed_t kf_sangle16_to_fixed_radians( kf_sangle16_t angle )
{
    return (kf_fixed_t)(((int64_t)angle * KF_FIXED_TWO_PI) / (int64_t)KF_ANGLE16_TURN);
}

kf_angle16_t kf_angle16_add( kf_angle16_t angle, kf_sangle16_t delta )
{
    return (kf_angle16_t)((int32_t)angle + (int32_t)delta);
}

kf_sangle16_t kf_sangle16_add_clamped( kf_sangle16_t angle, kf_sangle16_t delta, kf_sangle16_t minimum, kf_sangle16_t maximum )
{
    int32_t result;
    KF_DEBUG_REQUIRE( minimum <= maximum, "kinefix signed-angle clamp range is invalid" );
    if( minimum > maximum )
    {
        return angle;
    }
    result = (int32_t)angle + (int32_t)delta;
    if( result < (int32_t)minimum ) result = minimum;
    if( result > (int32_t)maximum ) result = maximum;
    return (kf_sangle16_t)result;
}

static kf_fixed_t __kf_angle16_sine( kf_angle16_t angle )
{
    const uint32_t quadrant = (uint32_t)angle >> 14;
    const uint32_t within_quadrant = (uint32_t)angle & UINT32_C(0x3FFF);
    const kf_bool_t negative = (kf_bool_t)(quadrant >= 2u);
    const uint32_t offset = (quadrant == 1u || quadrant == 3u)
        ? (uint32_t)KF_ANGLE16_QUARTER - within_quadrant
        : within_quadrant;
    const uint32_t index = offset >> 4;
    const uint32_t fraction = offset & UINT32_C(15);
    kf_fixed_t value = g_kf_sine_quarter_q16[index];
    if( fraction != 0u )
    {
        const kf_fixed_t difference = g_kf_sine_quarter_q16[index + 1u] - value;
        value += (difference * (kf_fixed_t)fraction) / INT32_C(16);
    }
    return negative == KF_TRUE ? -value : value;
}

void kf_angle16_sin_cos( kf_angle16_t angle, kf_fixed_t * sine, kf_fixed_t * cosine )
{
    if( sine != NULL ) *sine = __kf_angle16_sine( angle );
    if( cosine != NULL ) *cosine = __kf_angle16_sine( (kf_angle16_t)(angle + KF_ANGLE16_QUARTER) );
}

void kf_sangle16_sin_cos( kf_sangle16_t angle, kf_fixed_t * sine, kf_fixed_t * cosine )
{
    kf_angle16_sin_cos( (kf_angle16_t)angle, sine, cosine );
}

void kf_fixed_sin_cos( kf_fixed_t angle, kf_fixed_t * sine, kf_fixed_t * cosine )
{
    kf_angle16_sin_cos( kf_angle16_from_fixed_radians( angle ), sine, cosine );
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

KF_FORCE_INLINE kf_bool_t __kf_vec3_dot_q32( kf_vec3_t left, kf_vec3_t right, int64_t * result )
{
    int64_t sum;
    if( __kf_wide_add( (int64_t)left.x * (int64_t)right.x,
        (int64_t)left.y * (int64_t)right.y, &sum ) == KF_FALSE ) return KF_FALSE;
    return __kf_wide_add( sum, (int64_t)left.z * (int64_t)right.z, result );
}

static kf_fixed_t __kf_vec3_length_scaled_q32( kf_vec3_t value, __kf_q32_t scalar )
{
    int64_t square_q32;
    const uint64_t scalar_magnitude = __kf_magnitude( scalar );
    __kf_uint128_t scaled_square;
    uint64_t root;
    uint64_t result;
    KF_DEBUG_REQUIRE( scalar_magnitude <= (UINT64_C(1) << 32), "kinefix Q32.32 length scale is outside the unit interval" );
    if( scalar_magnitude > (UINT64_C(1) << 32) || __kf_vec3_dot_q32( value, value, &square_q32 ) == KF_FALSE || square_q32 < 0 ) return 0;
    if( scalar_magnitude == (UINT64_C(1) << 32) )
    {
        root = __kf_integer_sqrt_u64( (uint64_t)square_q32 );
        return __kf_signed_magnitude( root, (kf_bool_t)(scalar < 0) );
    }
    scaled_square = __kf_multiply_u64( (uint64_t)square_q32, scalar_magnitude * scalar_magnitude );
    root = __kf_integer_sqrt_u128( scaled_square );
    result = root >> 32;
    return __kf_signed_magnitude( result, (kf_bool_t)(scalar < 0) );
}

kf_fixed_t kf_vec3_dot( kf_vec3_t left, kf_vec3_t right )
{
    int64_t wide;
    if( __kf_vec3_dot_q32( left, right, &wide ) == KF_FALSE ) return 0;
    return __kf_narrow_q32( wide );
}

kf_vec3_t kf_vec3_cross( kf_vec3_t left, kf_vec3_t right )
{
    int64_t x;
    int64_t y;
    int64_t z;
    kf_vec3_t result;
    if( __kf_wide_sub( (int64_t)left.y * (int64_t)right.z,
        (int64_t)left.z * (int64_t)right.y, &x ) == KF_FALSE ) return (kf_vec3_t){0, 0, 0};
    if( __kf_wide_sub( (int64_t)left.z * (int64_t)right.x,
        (int64_t)left.x * (int64_t)right.z, &y ) == KF_FALSE ) return (kf_vec3_t){0, 0, 0};
    if( __kf_wide_sub( (int64_t)left.x * (int64_t)right.y,
        (int64_t)left.y * (int64_t)right.x, &z ) == KF_FALSE ) return (kf_vec3_t){0, 0, 0};
    result.x = __kf_narrow_q32( x );
    result.y = __kf_narrow_q32( y );
    result.z = __kf_narrow_q32( z );
    return result;
}

kf_fixed_t kf_vec3_length_squared( kf_vec3_t value ) { return kf_vec3_dot( value, value ); }
kf_fixed_t kf_vec3_length( kf_vec3_t value )
{
    int64_t square_q32;
    uint64_t root;
    if( __kf_vec3_dot_q32( value, value, &square_q32 ) == KF_FALSE || square_q32 < 0 ) return 0;
    root = __kf_integer_sqrt_u64( (uint64_t)square_q32 );
    return __kf_signed_magnitude( root, KF_FALSE );
}

kf_vec3_t kf_vec3_normalize( kf_vec3_t value )
{
    const kf_fixed_t length = kf_vec3_length( value );
    const kf_vec3_t zero = {0, 0, 0};
    return length == 0 ? zero : kf_vec3_div( value, length );
}

kf_fixed_t kf_segment_point_distance_squared( kf_vec3_t start, kf_vec3_t end, kf_vec3_t point )
{
    const kf_vec3_t segment = kf_vec3_sub( end, start );
    const kf_vec3_t relative = kf_vec3_sub( point, start );
    int64_t length_squared_q32;
    int64_t projection_q32;
    int64_t point_distance_q32;
    int64_t result_q32;
    __kf_uint128_t projection_squared;
    uint64_t correction_q32;
    if( __kf_vec3_dot_q32( segment, segment, &length_squared_q32 ) == KF_FALSE ||
        __kf_vec3_dot_q32( relative, segment, &projection_q32 ) == KF_FALSE ||
        __kf_vec3_dot_q32( relative, relative, &point_distance_q32 ) == KF_FALSE ) return 0;
    if( length_squared_q32 == 0 || projection_q32 <= 0 ) return __kf_narrow_q32( point_distance_q32 );
    if( projection_q32 >= length_squared_q32 )
    {
        int64_t end_distance_q32;
        const kf_vec3_t end_relative = kf_vec3_sub( point, end );
        if( __kf_vec3_dot_q32( end_relative, end_relative, &end_distance_q32 ) == KF_FALSE ) return 0;
        return __kf_narrow_q32( end_distance_q32 );
    }
    projection_squared = __kf_multiply_u64( (uint64_t)projection_q32, (uint64_t)projection_q32 );
    if( __kf_divide_u128_u64( projection_squared, (uint64_t)length_squared_q32, &correction_q32 ) == KF_FALSE ) return 0;
    KF_DEBUG_REQUIRE( correction_q32 <= (uint64_t)INT64_MAX, "kinefix segment projection overflow" );
    if( correction_q32 > (uint64_t)INT64_MAX || __kf_wide_sub( point_distance_q32, (int64_t)correction_q32, &result_q32 ) == KF_FALSE ) return 0;
    return __kf_narrow_q32( result_q32 );
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

KF_FORCE_INLINE kf_vec3_t __kf_normal_axis( size_t axis, int32_t sign )
{
    kf_vec3_t normal = {0, 0, 0};
    const kf_fixed_t value = sign < 0 ? -KF_FIXED_SCALE : KF_FIXED_SCALE;
    if( axis == 0 ) normal.x = value;
    else if( axis == 1 ) normal.y = value;
    else normal.z = value;
    return normal;
}

static kf_bool_t __kf_sweep_point_bounds( kf_vec3_t origin, kf_vec3_t displacement, const kf_fixed_t minimum[3], const kf_fixed_t maximum[3], uint32_t object_id, kf_hit_t * hit )
{
    const kf_fixed_t origins[3] = {origin.x, origin.y, origin.z};
    const kf_fixed_t deltas[3] = {displacement.x, displacement.y, displacement.z};
    const __kf_q32_t unit_q32 = INT64_C(1) << 32;
    __kf_q32_t entry_q32 = 0;
    __kf_q32_t exit_q32 = INT64_C(1) << 32;
    size_t entry_axis = 3;
    int32_t entry_sign = 0;
    kf_bool_t started_inside = KF_TRUE;
    size_t axis;

    for( axis = 0; axis != 3; ++axis )
    {
        const kf_fixed_t end = kf_fixed_add( origins[axis], deltas[axis] );
        __kf_q32_t axis_entry_q32 = 0;
        __kf_q32_t axis_exit_q32 = unit_q32;
        int32_t axis_sign = 0;

        if( origins[axis] <= minimum[axis] || origins[axis] >= maximum[axis] ) started_inside = KF_FALSE;

        if( deltas[axis] > 0 )
        {
            if( origins[axis] > maximum[axis] || end < minimum[axis] ) return KF_FALSE;
            if( origins[axis] < minimum[axis] )
            {
                if( __kf_ratio_fixed_to_q32( kf_fixed_sub( minimum[axis], origins[axis] ), deltas[axis], &axis_entry_q32 ) == KF_FALSE ) return KF_FALSE;
                axis_sign = -1;
            }
            else if( origins[axis] == minimum[axis] )
            {
                axis_sign = -1;
            }
            if( end > maximum[axis] && __kf_ratio_fixed_to_q32( kf_fixed_sub( maximum[axis], origins[axis] ), deltas[axis], &axis_exit_q32 ) == KF_FALSE ) return KF_FALSE;
        }
        else if( deltas[axis] < 0 )
        {
            if( origins[axis] < minimum[axis] || end > maximum[axis] ) return KF_FALSE;
            if( origins[axis] > maximum[axis] )
            {
                if( __kf_ratio_fixed_to_q32( kf_fixed_sub( maximum[axis], origins[axis] ), deltas[axis], &axis_entry_q32 ) == KF_FALSE ) return KF_FALSE;
                axis_sign = 1;
            }
            else if( origins[axis] == maximum[axis] )
            {
                axis_sign = 1;
            }
            if( end < minimum[axis] && __kf_ratio_fixed_to_q32( kf_fixed_sub( minimum[axis], origins[axis] ), deltas[axis], &axis_exit_q32 ) == KF_FALSE ) return KF_FALSE;
        }
        else if( origins[axis] <= minimum[axis] || origins[axis] >= maximum[axis] )
        {
            return KF_FALSE;
        }

        if( axis_entry_q32 > entry_q32 || (axis_entry_q32 == entry_q32 && entry_axis == 3 && axis_sign != 0) )
        {
            entry_q32 = axis_entry_q32;
            entry_axis = axis;
            entry_sign = axis_sign;
        }
        if( axis_exit_q32 < exit_q32 ) exit_q32 = axis_exit_q32;
        if( entry_q32 > exit_q32 ) return KF_FALSE;
    }

    if( exit_q32 <= 0 || entry_q32 > unit_q32 ) return KF_FALSE;
    if( hit != NULL )
    {
        *hit = (kf_hit_t){0};
        hit->object_id = object_id;
        hit->fraction = __kf_narrow_q32( entry_q32 );
        hit->position = __kf_vec3_madd_q32( origin, displacement, entry_q32 );
        hit->distance = __kf_vec3_length_scaled_q32( displacement, entry_q32 );
        hit->normal = entry_axis == 3 ? (kf_vec3_t){0, 0, 0} : __kf_normal_axis( entry_axis, entry_sign );
        hit->started_inside = started_inside;
    }
    return KF_TRUE;
}

KF_FORCE_INLINE kf_bool_t __kf_hit_precedes( const kf_hit_t * candidate, const kf_hit_t * selected, kf_bool_t has_selected )
{
    return (kf_bool_t)(has_selected == KF_FALSE || candidate->fraction < selected->fraction ||
        (candidate->fraction == selected->fraction && candidate->object_id < selected->object_id));
}

KF_FORCE_INLINE kf_bool_t __kf_capsule_valid( const kf_capsule_t * capsule )
{
    const kf_bool_t valid = (kf_bool_t)(capsule != NULL && capsule->radius >= 0 && (int64_t)capsule->height >= (int64_t)capsule->radius * INT64_C(2));
    KF_DEBUG_REQUIRE( valid == KF_TRUE, "kinefix capsule configuration is invalid" );
    return valid;
}

kf_bool_t kf_overlap_aabb_aabb( const kf_aabb_t * left, const kf_aabb_t * right )
{
    if( left == NULL || right == NULL ) return KF_FALSE;
    return (kf_bool_t)(left->maximum[0] > right->minimum[0] && left->minimum[0] < right->maximum[0] &&
        left->maximum[1] > right->minimum[1] && left->minimum[1] < right->maximum[1] &&
        left->maximum[2] > right->minimum[2] && left->minimum[2] < right->maximum[2]);
}

kf_bool_t kf_overlap_sphere_aabb( const kf_sphere_t * sphere, const kf_aabb_t * box )
{
    kf_fixed_t dx;
    kf_fixed_t dy;
    kf_fixed_t dz;
    int64_t distance_squared;
    if( sphere == NULL || box == NULL || sphere->radius < 0 ) return KF_FALSE;
    dx = sphere->position.x < box->minimum[0] ? kf_fixed_sub( box->minimum[0], sphere->position.x ) :
        (sphere->position.x > box->maximum[0] ? kf_fixed_sub( sphere->position.x, box->maximum[0] ) : 0);
    dy = sphere->position.y < box->minimum[1] ? kf_fixed_sub( box->minimum[1], sphere->position.y ) :
        (sphere->position.y > box->maximum[1] ? kf_fixed_sub( sphere->position.y, box->maximum[1] ) : 0);
    dz = sphere->position.z < box->minimum[2] ? kf_fixed_sub( box->minimum[2], sphere->position.z ) :
        (sphere->position.z > box->maximum[2] ? kf_fixed_sub( sphere->position.z, box->maximum[2] ) : 0);
    if( __kf_vec3_dot_q32( (kf_vec3_t){dx, dy, dz}, (kf_vec3_t){dx, dy, dz}, &distance_squared ) == KF_FALSE ) return KF_FALSE;
    return (kf_bool_t)(distance_squared < (int64_t)sphere->radius * (int64_t)sphere->radius);
}

kf_bool_t kf_overlap_capsule_aabb( const kf_capsule_t * capsule, const kf_aabb_t * box )
{
    kf_fixed_t segment_minimum_y;
    kf_fixed_t segment_maximum_y;
    kf_fixed_t dx;
    kf_fixed_t dy;
    kf_fixed_t dz;
    int64_t distance_squared;
    if( box == NULL || __kf_capsule_valid( capsule ) == KF_FALSE ) return KF_FALSE;
    segment_minimum_y = kf_fixed_add( capsule->position.y, capsule->radius );
    segment_maximum_y = kf_fixed_sub( kf_fixed_add( capsule->position.y, capsule->height ), capsule->radius );
    dx = capsule->position.x < box->minimum[0] ? kf_fixed_sub( box->minimum[0], capsule->position.x ) :
        (capsule->position.x > box->maximum[0] ? kf_fixed_sub( capsule->position.x, box->maximum[0] ) : 0);
    dy = segment_maximum_y < box->minimum[1] ? kf_fixed_sub( box->minimum[1], segment_maximum_y ) :
        (segment_minimum_y > box->maximum[1] ? kf_fixed_sub( segment_minimum_y, box->maximum[1] ) : 0);
    dz = capsule->position.z < box->minimum[2] ? kf_fixed_sub( box->minimum[2], capsule->position.z ) :
        (capsule->position.z > box->maximum[2] ? kf_fixed_sub( capsule->position.z, box->maximum[2] ) : 0);
    if( __kf_vec3_dot_q32( (kf_vec3_t){dx, dy, dz}, (kf_vec3_t){dx, dy, dz}, &distance_squared ) == KF_FALSE ) return KF_FALSE;
    return (kf_bool_t)(distance_squared < (int64_t)capsule->radius * (int64_t)capsule->radius);
}

kf_bool_t kf_overlap_sphere_capsule( const kf_sphere_t * sphere, const kf_capsule_t * capsule )
{
    kf_fixed_t segment_minimum_y;
    kf_fixed_t segment_maximum_y;
    kf_fixed_t closest_y;
    kf_fixed_t combined_radius;
    kf_vec3_t closest;
    kf_vec3_t relative;
    int64_t distance_squared;
    if( sphere == NULL || sphere->radius < 0 || __kf_capsule_valid( capsule ) == KF_FALSE ) return KF_FALSE;
    segment_minimum_y = kf_fixed_add( capsule->position.y, capsule->radius );
    segment_maximum_y = kf_fixed_sub( kf_fixed_add( capsule->position.y, capsule->height ), capsule->radius );
    closest_y = kf_fixed_clamp( sphere->position.y, segment_minimum_y, segment_maximum_y );
    closest = (kf_vec3_t){capsule->position.x, closest_y, capsule->position.z};
    combined_radius = kf_fixed_add( sphere->radius, capsule->radius );
    relative = kf_vec3_sub( sphere->position, closest );
    if( __kf_vec3_dot_q32( relative, relative, &distance_squared ) == KF_FALSE ) return KF_FALSE;
    return (kf_bool_t)(distance_squared < (int64_t)combined_radius * (int64_t)combined_radius);
}

kf_bool_t kf_overlap_capsule_capsule( const kf_capsule_t * left, const kf_capsule_t * right )
{
    kf_fixed_t left_minimum_y;
    kf_fixed_t left_maximum_y;
    kf_fixed_t right_minimum_y;
    kf_fixed_t right_maximum_y;
    kf_fixed_t dy;
    kf_fixed_t combined_radius;
    kf_vec3_t relative;
    int64_t distance_squared;
    if( __kf_capsule_valid( left ) == KF_FALSE || __kf_capsule_valid( right ) == KF_FALSE ) return KF_FALSE;
    left_minimum_y = kf_fixed_add( left->position.y, left->radius );
    left_maximum_y = kf_fixed_sub( kf_fixed_add( left->position.y, left->height ), left->radius );
    right_minimum_y = kf_fixed_add( right->position.y, right->radius );
    right_maximum_y = kf_fixed_sub( kf_fixed_add( right->position.y, right->height ), right->radius );
    dy = left_maximum_y < right_minimum_y ? kf_fixed_sub( right_minimum_y, left_maximum_y ) :
        (right_maximum_y < left_minimum_y ? kf_fixed_sub( left_minimum_y, right_maximum_y ) : 0);
    combined_radius = kf_fixed_add( left->radius, right->radius );
    relative = (kf_vec3_t){kf_fixed_sub( left->position.x, right->position.x ), dy, kf_fixed_sub( left->position.z, right->position.z )};
    if( __kf_vec3_dot_q32( relative, relative, &distance_squared ) == KF_FALSE ) return KF_FALSE;
    return (kf_bool_t)(distance_squared < (int64_t)combined_radius * (int64_t)combined_radius);
}

kf_bool_t kf_world_overlap_capsule( const kf_aabb_t * boxes, size_t count, const kf_capsule_t * capsule, uint32_t * brush_id )
{
    size_t index;
    kf_bool_t found = KF_FALSE;
    uint32_t selected_id = UINT32_MAX;
    for( index = 0; index != count; ++index )
    {
        if( kf_overlap_capsule_aabb( capsule, boxes + index ) == KF_FALSE ) continue;
        if( found == KF_FALSE || boxes[index].id < selected_id )
        {
            found = KF_TRUE;
            selected_id = boxes[index].id;
        }
    }
    if( found == KF_TRUE && brush_id != NULL ) *brush_id = selected_id;
    return found;
}

kf_bool_t kf_raycast_aabb( const kf_ray_t * ray, const kf_aabb_t * box, kf_hit_t * hit )
{
    kf_fixed_t direction_length;
    kf_vec3_t displacement;
    kf_bool_t result;
    if( ray == NULL || box == NULL || ray->maximum_distance <= 0 ) return KF_FALSE;
    direction_length = kf_vec3_length( ray->direction );
    if( direction_length == 0 ) return KF_FALSE;
    displacement = __kf_vec3_mul_div( ray->direction, ray->maximum_distance, direction_length );
    result = __kf_sweep_point_bounds( ray->origin, displacement, box->minimum, box->maximum, box->id, hit );
    return result;
}

kf_bool_t kf_world_raycast( const kf_aabb_t * boxes, size_t count, const kf_ray_t * ray, kf_hit_t * hit )
{
    size_t index;
    kf_bool_t found = KF_FALSE;
    kf_hit_t selected;
    for( index = 0; index != count; ++index )
    {
        kf_hit_t candidate;
        if( kf_raycast_aabb( ray, boxes + index, &candidate ) == KF_FALSE ) continue;
        if( __kf_hit_precedes( &candidate, &selected, found ) == KF_TRUE )
        {
            selected = candidate;
            found = KF_TRUE;
        }
    }
    if( found == KF_TRUE && hit != NULL ) *hit = selected;
    return found;
}

kf_bool_t kf_sweep_sphere_aabb_hit( const kf_sphere_t * sphere, kf_vec3_t displacement, const kf_aabb_t * box, kf_hit_t * hit )
{
    kf_fixed_t minimum[3];
    kf_fixed_t maximum[3];
    kf_bool_t result;
    if( sphere == NULL || box == NULL || sphere->radius < 0 ) return KF_FALSE;
    minimum[0] = kf_fixed_sub( box->minimum[0], sphere->radius );
    minimum[1] = kf_fixed_sub( box->minimum[1], sphere->radius );
    minimum[2] = kf_fixed_sub( box->minimum[2], sphere->radius );
    maximum[0] = kf_fixed_add( box->maximum[0], sphere->radius );
    maximum[1] = kf_fixed_add( box->maximum[1], sphere->radius );
    maximum[2] = kf_fixed_add( box->maximum[2], sphere->radius );
    result = __kf_sweep_point_bounds( sphere->position, displacement, minimum, maximum, box->id, hit );
    return result;
}

kf_bool_t kf_world_sweep_sphere_hit( const kf_aabb_t * boxes, size_t count, const kf_sphere_t * sphere, kf_vec3_t displacement, kf_hit_t * hit )
{
    size_t index;
    kf_bool_t found = KF_FALSE;
    kf_hit_t selected;
    for( index = 0; index != count; ++index )
    {
        kf_hit_t candidate;
        if( kf_sweep_sphere_aabb_hit( sphere, displacement, boxes + index, &candidate ) == KF_FALSE ) continue;
        if( __kf_hit_precedes( &candidate, &selected, found ) == KF_TRUE )
        {
            selected = candidate;
            found = KF_TRUE;
        }
    }
    if( found == KF_TRUE && hit != NULL ) *hit = selected;
    return found;
}

static kf_bool_t __kf_append_quadratic_root( uint64_t left, kf_bool_t left_negative, uint64_t right, kf_bool_t right_negative, uint64_t denominator, kf_bool_t denominator_negative, __kf_q32_t roots[2], size_t * count )
{
    uint64_t numerator;
    kf_bool_t numerator_negative;
    if( __kf_signed_sum_magnitude( left, left_negative, right, right_negative, &numerator, &numerator_negative ) == KF_FALSE ) return KF_FALSE;
    if( __kf_ratio_magnitude_to_q32( numerator, numerator_negative, denominator, denominator_negative, roots + *count ) == KF_FALSE ) return KF_FALSE;
    ++*count;
    return KF_TRUE;
}

static size_t __kf_quadratic_roots_q32( __kf_q32_t a, __kf_q32_t b, __kf_q32_t c, __kf_q32_t roots[2] )
{
    const uint64_t a_magnitude = __kf_magnitude( a );
    const uint64_t b_magnitude = __kf_magnitude( b );
    const uint64_t c_magnitude = __kf_magnitude( c );
    const kf_bool_t a_negative = (kf_bool_t)(a < 0);
    const kf_bool_t b_negative = (kf_bool_t)(b < 0);
    const kf_bool_t c_negative = (kf_bool_t)(c < 0);
    __kf_uint128_t discriminant;
    __kf_uint128_t four_ac;
    uint64_t square_root;
    uint64_t denominator;
    size_t count = 0;
    if( a == 0 )
    {
        if( b == 0 ) return 0;
        if( __kf_ratio_magnitude_to_q32( c_magnitude, (kf_bool_t)(c_negative == KF_FALSE && c != 0), b_magnitude, b_negative, roots ) == KF_FALSE ) return 0;
        return 1;
    }
    discriminant = __kf_multiply_u64( b_magnitude, b_magnitude );
    if( __kf_shift_left_two_u128( __kf_multiply_u64( a_magnitude, c_magnitude ), &four_ac ) == KF_FALSE ) return 0;
    if( a_negative != c_negative )
    {
        if( __kf_add_u128( discriminant, four_ac, &discriminant ) == KF_FALSE ) return 0;
    }
    else
    {
        if( __kf_compare_u128( discriminant, four_ac ) < 0 ) return 0;
        discriminant = __kf_sub_u128( discriminant, four_ac );
    }
    square_root = __kf_integer_sqrt_u128( discriminant );
    KF_DEBUG_REQUIRE( a_magnitude <= UINT64_MAX / UINT64_C(2), "kinefix quadratic denominator overflow" );
    if( a_magnitude > UINT64_MAX / UINT64_C(2) ) return 0;
    denominator = a_magnitude * UINT64_C(2);
    (void)__kf_append_quadratic_root( b_magnitude, (kf_bool_t)(b_negative == KF_FALSE && b != 0), square_root, KF_TRUE, denominator, a_negative, roots, &count );
    (void)__kf_append_quadratic_root( b_magnitude, (kf_bool_t)(b_negative == KF_FALSE && b != 0), square_root, KF_FALSE, denominator, a_negative, roots, &count );
    return count;
}

static void __kf_consider_capsule_root( __kf_q32_t root, kf_bool_t restrict_y, int64_t y_q48, kf_fixed_t minimum_y, kf_fixed_t maximum_y, kf_bool_t * found, __kf_q32_t * selected )
{
    if( root < 0 || root > (INT64_C(1) << 32) ) return;
    if( restrict_y == KF_TRUE && (y_q48 < (int64_t)minimum_y * (INT64_C(1) << 32) || y_q48 > (int64_t)maximum_y * (INT64_C(1) << 32)) ) return;
    if( *found == KF_FALSE || root < *selected )
    {
        *found = KF_TRUE;
        *selected = root;
    }
}

kf_bool_t kf_sweep_sphere_capsule( const kf_sphere_t * sphere, kf_vec3_t displacement, const kf_capsule_t * capsule, uint32_t object_id, kf_hit_t * hit )
{
    kf_fixed_t combined_radius;
    __kf_q32_t radius_squared_q32;
    kf_fixed_t segment_minimum_y;
    kf_fixed_t segment_maximum_y;
    __kf_q32_t roots[2];
    __kf_q32_t selected = 0;
    kf_bool_t found = KF_FALSE;
    size_t root_count;
    size_t index;
    kf_vec3_t relative;
    __kf_q32_t displacement_length_squared_q32;
    __kf_q32_t a_q32;
    __kf_q32_t b_q32;
    __kf_q32_t c_q32;
    kf_vec3_t centers[2];
    __kf_q32_t wide;

    if( sphere == NULL || sphere->radius < 0 || __kf_capsule_valid( capsule ) == KF_FALSE ) return KF_FALSE;
    if( __kf_vec3_dot_q32( displacement, displacement, &displacement_length_squared_q32 ) == KF_FALSE ) return KF_FALSE;
    if( displacement_length_squared_q32 == 0 )
    {
        if( kf_overlap_sphere_capsule( sphere, capsule ) == KF_FALSE ) return KF_FALSE;
        if( hit != NULL )
        {
            *hit = (kf_hit_t){0};
            hit->object_id = object_id;
            hit->position = sphere->position;
            hit->started_inside = KF_TRUE;
        }
        return KF_TRUE;
    }
    combined_radius = kf_fixed_add( sphere->radius, capsule->radius );
    radius_squared_q32 = (int64_t)combined_radius * (int64_t)combined_radius;
    segment_minimum_y = kf_fixed_add( capsule->position.y, capsule->radius );
    segment_maximum_y = kf_fixed_sub( kf_fixed_add( capsule->position.y, capsule->height ), capsule->radius );

    if( kf_overlap_sphere_capsule( sphere, capsule ) == KF_TRUE )
    {
        selected = 0;
        found = KF_TRUE;
    }
    else
    {
        relative = kf_vec3_sub( sphere->position, capsule->position );
        if( __kf_wide_add( (int64_t)displacement.x * (int64_t)displacement.x,
            (int64_t)displacement.z * (int64_t)displacement.z, &a_q32 ) == KF_FALSE ) return KF_FALSE;
        if( __kf_wide_add( (int64_t)relative.x * (int64_t)displacement.x,
            (int64_t)relative.z * (int64_t)displacement.z, &wide ) == KF_FALSE ||
            __kf_wide_add( wide, wide, &b_q32 ) == KF_FALSE ) return KF_FALSE;
        if( __kf_wide_add( (int64_t)relative.x * (int64_t)relative.x,
            (int64_t)relative.z * (int64_t)relative.z, &wide ) == KF_FALSE ||
            __kf_wide_sub( wide, radius_squared_q32, &c_q32 ) == KF_FALSE ) return KF_FALSE;
        root_count = __kf_quadratic_roots_q32( a_q32, b_q32, c_q32, roots );
        if( root_count != 0 && roots[0] >= 0 && roots[0] <= (INT64_C(1) << 32) )
        {
            int64_t y_q48;
            if( __kf_fixed_madd_q32_wide( sphere->position.y, displacement.y, roots[0], &y_q48 ) == KF_FALSE ) return KF_FALSE;
            __kf_consider_capsule_root( roots[0], KF_TRUE, y_q48, segment_minimum_y, segment_maximum_y, &found, &selected );
        }

        centers[0] = (kf_vec3_t){capsule->position.x, segment_minimum_y, capsule->position.z};
        centers[1] = (kf_vec3_t){capsule->position.x, segment_maximum_y, capsule->position.z};
        for( index = 0; index != 2; ++index )
        {
            relative = kf_vec3_sub( sphere->position, centers[index] );
            if( __kf_vec3_dot_q32( relative, displacement, &wide ) == KF_FALSE ||
                __kf_wide_add( wide, wide, &b_q32 ) == KF_FALSE ) return KF_FALSE;
            if( __kf_vec3_dot_q32( relative, relative, &wide ) == KF_FALSE ||
                __kf_wide_sub( wide, radius_squared_q32, &c_q32 ) == KF_FALSE ) return KF_FALSE;
            root_count = __kf_quadratic_roots_q32( displacement_length_squared_q32, b_q32, c_q32, roots );
            if( root_count != 0 ) __kf_consider_capsule_root( roots[0], KF_FALSE, 0, 0, 0, &found, &selected );
        }
    }

    if( found == KF_FALSE ) return KF_FALSE;
    if( hit != NULL )
    {
        kf_fixed_t closest_y;
        kf_vec3_t closest;
        *hit = (kf_hit_t){0};
        hit->object_id = object_id;
        hit->fraction = __kf_narrow_q32( selected );
        hit->position = __kf_vec3_madd_q32( sphere->position, displacement, selected );
        hit->distance = __kf_vec3_length_scaled_q32( displacement, selected );
        closest_y = kf_fixed_clamp( hit->position.y, segment_minimum_y, segment_maximum_y );
        closest = (kf_vec3_t){capsule->position.x, closest_y, capsule->position.z};
        hit->normal = kf_vec3_normalize( kf_vec3_sub( hit->position, closest ) );
        if( hit->normal.x == 0 && hit->normal.y == 0 && hit->normal.z == 0 ) hit->normal = kf_vec3_mul( kf_vec3_normalize( displacement ), -KF_FIXED_SCALE );
        hit->started_inside = (kf_bool_t)(selected == 0 && kf_overlap_sphere_capsule( sphere, capsule ) == KF_TRUE);
    }
    return KF_TRUE;
}

kf_bool_t kf_raycast_capsule( const kf_ray_t * ray, const kf_capsule_t * capsule, uint32_t object_id, kf_hit_t * hit )
{
    kf_fixed_t direction_length;
    kf_vec3_t displacement;
    kf_sphere_t point;
    if( ray == NULL || ray->maximum_distance <= 0 || __kf_capsule_valid( capsule ) == KF_FALSE ) return KF_FALSE;
    direction_length = kf_vec3_length( ray->direction );
    if( direction_length == 0 ) return KF_FALSE;
    displacement = __kf_vec3_mul_div( ray->direction, ray->maximum_distance, direction_length );
    point.position = ray->origin;
    point.radius = 0;
    return kf_sweep_sphere_capsule( &point, displacement, capsule, object_id, hit );
}

kf_bool_t kf_sweep_capsule_aabb( const kf_capsule_t * capsule, kf_vec3_t displacement, const kf_aabb_t * box, kf_hit_t * hit )
{
    kf_fixed_t minimum[3];
    kf_fixed_t maximum[3];
    kf_bool_t result;
    if( box == NULL || __kf_capsule_valid( capsule ) == KF_FALSE ) return KF_FALSE;
    minimum[0] = kf_fixed_sub( box->minimum[0], capsule->radius );
    minimum[1] = kf_fixed_sub( box->minimum[1], capsule->height );
    minimum[2] = kf_fixed_sub( box->minimum[2], capsule->radius );
    maximum[0] = kf_fixed_add( box->maximum[0], capsule->radius );
    maximum[1] = box->maximum[1];
    maximum[2] = kf_fixed_add( box->maximum[2], capsule->radius );
    result = __kf_sweep_point_bounds( capsule->position, displacement, minimum, maximum, box->id, hit );
    return result;
}

kf_bool_t kf_world_sweep_capsule( const kf_aabb_t * boxes, size_t count, const kf_capsule_t * capsule, kf_vec3_t displacement, kf_hit_t * hit )
{
    size_t index;
    kf_bool_t found = KF_FALSE;
    kf_hit_t selected;
    for( index = 0; index != count; ++index )
    {
        kf_hit_t candidate;
        if( kf_sweep_capsule_aabb( capsule, displacement, boxes + index, &candidate ) == KF_FALSE ) continue;
        if( __kf_hit_precedes( &candidate, &selected, found ) == KF_TRUE )
        {
            selected = candidate;
            found = KF_TRUE;
        }
    }
    if( found == KF_TRUE && hit != NULL ) *hit = selected;
    return found;
}

kf_bool_t kf_aabb_overlaps_character( const kf_aabb_t * box, kf_vec3_t position, kf_fixed_t half_width, kf_fixed_t height )
{
    const kf_capsule_t capsule = {position, half_width, height};
    return kf_overlap_capsule_aabb( &capsule, box );
}

kf_bool_t kf_world_character_collides( const kf_aabb_t * boxes, size_t count, kf_vec3_t position, kf_fixed_t half_width, kf_fixed_t height )
{
    const kf_capsule_t capsule = {position, half_width, height};
    return kf_world_overlap_capsule( boxes, count, &capsule, NULL );
}

kf_bool_t kf_world_find_step_top( const kf_aabb_t * boxes, size_t count, kf_vec3_t candidate, kf_fixed_t half_width, kf_fixed_t height, kf_fixed_t current_y, kf_fixed_t maximum_step, kf_fixed_t * top )
{
    size_t index;
    kf_bool_t found = KF_FALSE;
    kf_fixed_t selected = current_y;
    for( index = 0; index != count; ++index )
    {
        const kf_aabb_t * box = boxes + index;
        const kf_fixed_t box_top = box->maximum[1];
        if( kf_aabb_overlaps_character( box, candidate, half_width, height ) == KF_FALSE ) continue;
        if( box_top <= current_y || kf_fixed_sub( box_top, current_y ) > maximum_step ) continue;
        selected = kf_fixed_max( selected, box_top );
        found = KF_TRUE;
    }
    if( top != NULL ) *top = selected;
    return found;
}

kf_bool_t kf_sweep_sphere_aabb( kf_vec3_t start, kf_vec3_t end, kf_fixed_t radius, const kf_aabb_t * box, kf_fixed_t * hit_time )
{
    const kf_sphere_t sphere = {start, radius};
    kf_hit_t hit;
    const kf_bool_t result = kf_sweep_sphere_aabb_hit( &sphere, kf_vec3_sub( end, start ), box, &hit );
    if( result == KF_TRUE && hit_time != NULL ) *hit_time = hit.fraction;
    return result;
}

kf_bool_t kf_world_sweep_sphere( const kf_aabb_t * boxes, size_t count, kf_vec3_t start, kf_vec3_t end, kf_fixed_t radius, kf_fixed_t * hit_time, uint32_t * brush_id )
{
    const kf_sphere_t sphere = {start, radius};
    kf_hit_t hit;
    const kf_bool_t result = kf_world_sweep_sphere_hit( boxes, count, &sphere, kf_vec3_sub( end, start ), &hit );
    if( result == KF_TRUE )
    {
        if( hit_time != NULL ) *hit_time = hit.fraction;
        if( brush_id != NULL ) *brush_id = hit.object_id;
    }
    return result;
}

kf_bool_t kf_world_line_blocked( const kf_aabb_t * boxes, size_t count, kf_vec3_t start, kf_vec3_t end, kf_fixed_t minimum_time, kf_fixed_t maximum_time )
{
    const kf_vec3_t delta = kf_vec3_sub( end, start );
    const kf_fixed_t length = kf_vec3_length( delta );
    const kf_ray_t ray = {start, delta, length};
    kf_hit_t hit;
    if( length == 0 || kf_world_raycast( boxes, count, &ray, &hit ) == KF_FALSE ) return KF_FALSE;
    return (kf_bool_t)(hit.fraction > minimum_time && hit.fraction < maximum_time);
}

static const kf_aabb_t * __kf_world_find_box( const kf_aabb_t * boxes, size_t count, uint32_t id )
{
    size_t index;
    for( index = 0; index != count; ++index ) if( boxes[index].id == id ) return boxes + index;
    return NULL;
}

KF_FORCE_INLINE kf_fixed_t __kf_character_safe_fraction( kf_fixed_t fraction, kf_fixed_t distance, kf_fixed_t skin )
{
    int64_t numerator;
    int64_t quotient;
    if( fraction <= 0 || distance <= 0 || skin <= 0 ) return kf_fixed_max( fraction, 0 );
    if( __kf_wide_sub( (int64_t)fraction * (int64_t)distance, (int64_t)skin * (int64_t)KF_FIXED_SCALE, &numerator ) == KF_FALSE ) return 0;
    quotient = numerator / distance;
    if( quotient <= 0 ) return 0;
    KF_DEBUG_REQUIRE( quotient <= INT32_MAX, "kinefix safe fraction overflow" );
    return quotient > INT32_MAX ? 0 : (kf_fixed_t)quotient;
}

static kf_bool_t __kf_character_try_step( kf_character_body_t * body, const kf_character_config_t * config, const kf_aabb_t * boxes, size_t count, kf_vec3_t displacement, const kf_hit_t * blocking_hit )
{
    const kf_aabb_t * step = __kf_world_find_box( boxes, count, blocking_hit->object_id );
    kf_capsule_t capsule;
    kf_vec3_t rise;
    kf_hit_t hit;
    kf_fixed_t step_height;
    if( step == NULL || body->grounded == KF_FALSE ) return KF_FALSE;
    step_height = kf_fixed_sub( step->maximum[1], body->position.y );
    if( step_height <= 0 || step_height > config->step_height ) return KF_FALSE;

    capsule.position = body->position;
    capsule.radius = config->radius;
    capsule.height = config->height;
    rise = (kf_vec3_t){0, step_height, 0};
    if( kf_world_sweep_capsule( boxes, count, &capsule, rise, &hit ) == KF_TRUE && hit.object_id != step->id ) return KF_FALSE;
    capsule.position = kf_vec3_add( capsule.position, rise );
    if( kf_world_overlap_capsule( boxes, count, &capsule, NULL ) == KF_TRUE ) return KF_FALSE;
    if( kf_world_sweep_capsule( boxes, count, &capsule, displacement, &hit ) == KF_TRUE ) return KF_FALSE;
    body->position = kf_vec3_add( capsule.position, displacement );
    return KF_TRUE;
}

static void __kf_character_move_horizontal( kf_character_body_t * body, const kf_character_config_t * config, const kf_aabb_t * boxes, size_t count, kf_fixed_t delta, kf_bool_t x_axis, kf_character_result_t * result )
{
    kf_capsule_t capsule;
    kf_vec3_t displacement = {0, 0, 0};
    kf_hit_t hit;
    kf_fixed_t fraction;
    if( delta == 0 ) return;
    if( x_axis == KF_TRUE ) displacement.x = delta;
    else displacement.z = delta;
    capsule.position = body->position;
    capsule.radius = config->radius;
    capsule.height = config->height;
    if( kf_world_sweep_capsule( boxes, count, &capsule, displacement, &hit ) == KF_FALSE )
    {
        body->position = kf_vec3_add( body->position, displacement );
        return;
    }
    if( __kf_character_try_step( body, config, boxes, count, displacement, &hit ) == KF_TRUE )
    {
        result->stepped = KF_TRUE;
        return;
    }
    fraction = __kf_character_safe_fraction( hit.fraction, kf_fixed_abs( delta ), config->support_epsilon );
    body->position = __kf_vec3_madd( body->position, displacement, fraction );
    if( result->blocked == KF_FALSE ) result->hit = hit;
    result->blocked = KF_TRUE;
    if( x_axis == KF_TRUE ) body->velocity.x = 0;
    else body->velocity.z = 0;
}

void kf_character_step( kf_character_body_t * body, const kf_character_config_t * config, const kf_aabb_t * boxes, size_t count, kf_character_result_t * result )
{
    kf_vec3_t displacement;
    kf_capsule_t capsule;
    kf_vec3_t vertical_displacement;
    kf_hit_t vertical_hit;
    kf_fixed_t boundary;
    *result = (kf_character_result_t){0};
    if( body == NULL || config == NULL || __kf_capsule_valid( &(kf_capsule_t){body->position, config->radius, config->height} ) == KF_FALSE ) return;
    if( body->grounded == KF_FALSE )
    {
        body->velocity.y = __kf_fixed_msub( body->velocity.y, config->gravity, config->tick_seconds );
    }
    result->previous_vertical_velocity = body->velocity.y;
    displacement = kf_vec3_mul( body->velocity, config->tick_seconds );
    __kf_character_move_horizontal( body, config, boxes, count, displacement.x, KF_TRUE, result );
    __kf_character_move_horizontal( body, config, boxes, count, displacement.z, KF_FALSE, result );

    capsule.position = body->position;
    capsule.radius = config->radius;
    capsule.height = config->height;
    vertical_displacement = (kf_vec3_t){0, displacement.y, 0};
    if( displacement.y != 0 && kf_world_sweep_capsule( boxes, count, &capsule, vertical_displacement, &vertical_hit ) == KF_TRUE )
    {
        body->position = __kf_vec3_madd( body->position, vertical_displacement, vertical_hit.fraction );
        result->landed = (kf_bool_t)(body->grounded == KF_FALSE && displacement.y < 0);
        result->hit_ceiling = (kf_bool_t)(displacement.y > 0);
        if( result->blocked == KF_FALSE ) result->hit = vertical_hit;
        result->blocked = KF_TRUE;
        body->velocity.y = 0;
        body->grounded = (kf_bool_t)(displacement.y < 0);
    }
    else
    {
        kf_capsule_t support;
        body->position = kf_vec3_add( body->position, vertical_displacement );
        support.position = body->position;
        support.position.y = kf_fixed_sub( support.position.y, config->support_epsilon );
        support.radius = config->radius;
        support.height = config->height;
        body->grounded = kf_world_overlap_capsule( boxes, count, &support, NULL );
    }

    boundary = kf_fixed_sub( config->arena_half_extent, config->radius );
    if( body->position.x < -boundary ) { body->position.x = -boundary; body->velocity.x = 0; }
    if( body->position.x > boundary ) { body->position.x = boundary; body->velocity.x = 0; }
    if( body->position.z < -boundary ) { body->position.z = -boundary; body->velocity.z = 0; }
    if( body->position.z > boundary ) { body->position.z = boundary; body->velocity.z = 0; }
}
