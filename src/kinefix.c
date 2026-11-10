#include "kinefix_internal.h"

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

typedef int64_t __kf_int64_t;
typedef uint64_t __kf_uint64_t;
typedef __kf_int64_t __kf_q32_t;

static kf_bool_t __kf_ratio_magnitude_to_fixed( __kf_uint64_t numerator, kf_bool_t numerator_negative, __kf_uint64_t denominator, kf_bool_t denominator_negative, kf_fixed_t * result )
{
    __kf_uint64_t quotient;
    __kf_uint64_t limit;
    kf_bool_t negative;
    if( denominator == 0 || result == NULL )
    {
        return KF_FALSE;
    }
    /* A quotient that does not fit returns UINT64_MAX, which the limit rejects. */
    quotient = kf_internal_wide_quotient( kf_internal_wide_make( numerator >> 48, numerator << KF_FIXED_FRACTION_BITS ), kf_internal_wide_make( 0, denominator ) );
    negative = (kf_bool_t)(numerator_negative != denominator_negative && quotient != 0);
    limit = negative == KF_TRUE ? (UINT64_C(1) << 31) : (__kf_uint64_t)INT32_MAX;
    if( quotient > limit )
    {
        return KF_FALSE;
    }
    if( negative == KF_FALSE )
    {
        *result = (kf_fixed_t)quotient;
    }
    else if( quotient == (UINT64_C(1) << 31) )
    {
        *result = INT32_MIN;
    }
    else
    {
        *result = -(kf_fixed_t)quotient;
    }
    return KF_TRUE;
}

KF_FORCE_INLINE kf_bool_t __kf_ratio_q32_to_fixed( __kf_q32_t numerator, __kf_q32_t denominator, kf_fixed_t * result )
{
    return __kf_ratio_magnitude_to_fixed( kf_internal_magnitude( numerator ), (kf_bool_t)(numerator < 0), kf_internal_magnitude( denominator ), (kf_bool_t)(denominator < 0), result );
}

KF_FORCE_INLINE kf_fixed_t __kf_signed_magnitude( __kf_uint64_t value, kf_bool_t negative )
{
    const __kf_uint64_t limit = negative == KF_TRUE ? (UINT64_C(1) << 31) : (__kf_uint64_t)INT32_MAX;
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
KF_FORCE_INLINE kf_fixed_t __kf_narrow_q32( __kf_int64_t value )
{
    const kf_bool_t negative = (kf_bool_t)(value < 0);
    const __kf_uint64_t magnitude = kf_internal_magnitude( value ) >> KF_FIXED_FRACTION_BITS;
    return __kf_signed_magnitude( magnitude, negative );
}

KF_FORCE_INLINE kf_bool_t __kf_wide_add( __kf_int64_t left, __kf_int64_t right, __kf_int64_t * result )
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

KF_FORCE_INLINE kf_bool_t __kf_wide_sub( __kf_int64_t left, __kf_int64_t right, __kf_int64_t * result )
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
    __kf_int64_t wide;
    if( __kf_wide_add( (__kf_int64_t)base * (__kf_int64_t)KF_FIXED_SCALE, (__kf_int64_t)left * (__kf_int64_t)right, &wide ) == KF_FALSE )
    {
        return 0;
    }
    return __kf_narrow_q32( wide );
}

KF_FORCE_INLINE kf_fixed_t __kf_fixed_msub( kf_fixed_t base, kf_fixed_t left, kf_fixed_t right )
{
    __kf_int64_t wide;
    if( __kf_wide_sub( (__kf_int64_t)base * (__kf_int64_t)KF_FIXED_SCALE, (__kf_int64_t)left * (__kf_int64_t)right, &wide ) == KF_FALSE )
    {
        return 0;
    }
    return __kf_narrow_q32( wide );
}

KF_FORCE_INLINE kf_fixed_t __kf_fixed_madd2( kf_fixed_t base, kf_fixed_t first, kf_fixed_t first_scalar, kf_fixed_t second, kf_fixed_t second_scalar )
{
    __kf_int64_t wide;
    if( __kf_wide_add( (__kf_int64_t)base * (__kf_int64_t)KF_FIXED_SCALE, (__kf_int64_t)first * (__kf_int64_t)first_scalar, &wide ) == KF_FALSE )
    {
        return 0;
    }
    if( __kf_wide_add( wide, (__kf_int64_t)second * (__kf_int64_t)second_scalar, &wide ) == KF_FALSE )
    {
        return 0;
    }
    return __kf_narrow_q32( wide );
}

KF_FORCE_INLINE kf_fixed_t __kf_fixed_mul_div( kf_fixed_t value, kf_fixed_t multiplier, kf_fixed_t divisor )
{
    __kf_int64_t quotient;
    KF_DEBUG_REQUIRE( divisor != 0, "kinefix multiply-divide by zero" );
    if( divisor == 0 )
    {
        return 0;
    }
    quotient = (__kf_int64_t)value * (__kf_int64_t)multiplier / (__kf_int64_t)divisor;
    KF_DEBUG_REQUIRE( quotient >= INT32_MIN && quotient <= INT32_MAX, "kinefix multiply-divide overflow" );
    if( quotient < INT32_MIN || quotient > INT32_MAX )
    {
        return 0;
    }
    return (kf_fixed_t)quotient;
}

KF_FORCE_INLINE kf_vec3_t __kf_vec3_madd( kf_vec3_t base, kf_vec3_t value, kf_fixed_t scalar )
{
    const kf_vec3_t result = {__kf_fixed_madd( base.x, value.x, scalar ), __kf_fixed_madd( base.y, value.y, scalar ), __kf_fixed_madd( base.z, value.z, scalar )};
    return result;
}

KF_FORCE_INLINE kf_vec3_t __kf_vec3_msub( kf_vec3_t base, kf_vec3_t value, kf_fixed_t scalar )
{
    const kf_vec3_t result = {__kf_fixed_msub( base.x, value.x, scalar ), __kf_fixed_msub( base.y, value.y, scalar ), __kf_fixed_msub( base.z, value.z, scalar )};
    return result;
}

KF_FORCE_INLINE kf_vec3_t __kf_vec3_madd2( kf_vec3_t base, kf_vec3_t first, kf_fixed_t first_scalar, kf_vec3_t second, kf_fixed_t second_scalar )
{
    const kf_vec3_t result = {
        __kf_fixed_madd2( base.x, first.x, first_scalar, second.x, second_scalar ),
        __kf_fixed_madd2( base.y, first.y, first_scalar, second.y, second_scalar ),
        __kf_fixed_madd2( base.z, first.z, first_scalar, second.z, second_scalar )};
    return result;
}

KF_FORCE_INLINE kf_vec3_t __kf_vec3_mul_div( kf_vec3_t value, kf_fixed_t multiplier, kf_fixed_t divisor )
{
    const kf_vec3_t result = {__kf_fixed_mul_div( value.x, multiplier, divisor ), __kf_fixed_mul_div( value.y, multiplier, divisor ), __kf_fixed_mul_div( value.z, multiplier, divisor )};
    return result;
}

kf_fixed_t kf_fixed_from_int( __kf_int64_t value )
{
    KF_DEBUG_REQUIRE( value >= INT16_MIN && value <= INT16_MAX, "kinefix integer conversion overflow" );
    if( value < INT16_MIN || value > INT16_MAX )
    {
        return 0;
    }
    return (kf_fixed_t)(value * KF_FIXED_SCALE);
}

kf_fixed_t kf_fixed_from_ratio( __kf_int64_t numerator, __kf_int64_t denominator )
{
    kf_fixed_t result;
    KF_DEBUG_REQUIRE( denominator != 0, "kinefix ratio conversion division by zero" );
    if( denominator == 0 )
    {
        return 0;
    }
    if( __kf_ratio_q32_to_fixed( numerator, denominator, &result ) == KF_TRUE )
    {
        return result;
    }
    KF_DEBUG_REQUIRE( 0, "kinefix ratio conversion overflow" );
    return 0;
}

kf_fixed_t kf_fixed_from_float( float value )
{
    kf_fixed_t result = kf_fixed_from_double( (double)value );
    return result;
}

kf_fixed_t kf_fixed_from_double( double value )
{
    double scaled = value * (double)KF_FIXED_SCALE;
    KF_DEBUG_REQUIRE( scaled == scaled, "kinefix double conversion requires a number" );
    if( scaled != scaled )
    {
        return 0;
    }
    KF_DEBUG_REQUIRE( scaled >= (double)INT32_MIN && scaled <= (double)INT32_MAX, "kinefix double conversion overflow" );
    if( scaled < (double)INT32_MIN || scaled > (double)INT32_MAX )
    {
        return 0;
    }
    return (kf_fixed_t)scaled;
}

int32_t kf_fixed_to_int( kf_fixed_t value )
{
    return value / KF_FIXED_SCALE;
}

float kf_fixed_to_float( kf_fixed_t value )
{
    return (float)value / (float)KF_FIXED_SCALE;
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
    const __kf_int64_t result = (__kf_int64_t)left + (__kf_int64_t)right;
    KF_DEBUG_REQUIRE( result >= INT32_MIN && result <= INT32_MAX, "kinefix addition overflow" );
    if( result < INT32_MIN || result > INT32_MAX )
    {
        return 0;
    }
    return (kf_fixed_t)result;
}

kf_fixed_t kf_fixed_sub( kf_fixed_t left, kf_fixed_t right )
{
    const __kf_int64_t result = (__kf_int64_t)left - (__kf_int64_t)right;
    KF_DEBUG_REQUIRE( result >= INT32_MIN && result <= INT32_MAX, "kinefix subtraction overflow" );
    if( result < INT32_MIN || result > INT32_MAX )
    {
        return 0;
    }
    return (kf_fixed_t)result;
}

kf_fixed_t kf_fixed_mul( kf_fixed_t left, kf_fixed_t right )
{
    return __kf_narrow_q32( (__kf_int64_t)left * (__kf_int64_t)right );
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
    if( __kf_fixed_try_div( left, right, &result ) == KF_TRUE )
    {
        return result;
    }
    KF_DEBUG_REQUIRE( 0, "kinefix division overflow" );
    return 0;
}

kf_fixed_t kf_fixed_add_mul( kf_fixed_t base, kf_fixed_t value, kf_fixed_t scalar )
{
    return __kf_fixed_madd( base, value, scalar );
}

kf_fixed_t kf_fixed_sub_mul( kf_fixed_t base, kf_fixed_t value, kf_fixed_t scalar )
{
    return __kf_fixed_msub( base, value, scalar );
}

kf_fixed_t kf_fixed_add_mul2( kf_fixed_t base, kf_fixed_t first, kf_fixed_t first_scalar, kf_fixed_t second, kf_fixed_t second_scalar )
{
    return __kf_fixed_madd2( base, first, first_scalar, second, second_scalar );
}

kf_fixed_t kf_fixed_mul_div( kf_fixed_t value, kf_fixed_t multiplier, kf_fixed_t divisor )
{
    return __kf_fixed_mul_div( value, multiplier, divisor );
}

kf_fixed_t kf_fixed_lerp( kf_fixed_t from, kf_fixed_t to, kf_fixed_t factor )
{
    __kf_int64_t wide;
    KF_DEBUG_REQUIRE( factor >= 0 && factor <= KF_FIXED_SCALE, "kinefix lerp factor is outside the unit interval" );
    if( __kf_wide_add( (__kf_int64_t)from * (__kf_int64_t)(KF_FIXED_SCALE - factor), (__kf_int64_t)to * (__kf_int64_t)factor, &wide ) == KF_FALSE )
    {
        return 0;
    }
    return __kf_narrow_q32( wide );
}

static __kf_uint64_t __kf_integer_sqrt_u64( __kf_uint64_t value )
{
    __kf_uint64_t result = 0;
    __kf_uint64_t bit = UINT64_C(1) << 62;
    while( bit > value )
    {
        bit >>= 2;
    }
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

    return (kf_fixed_t)__kf_integer_sqrt_u64( (__kf_uint64_t)(uint32_t)value * (__kf_uint64_t)KF_FIXED_SCALE );
}

kf_fixed_t kf_fixed_wrap_angle( kf_fixed_t angle )
{
    angle %= KF_FIXED_TWO_PI;
    if( angle > KF_FIXED_PI )
    {
        angle = kf_fixed_sub( angle, KF_FIXED_TWO_PI );
    }
    if( angle < -KF_FIXED_PI )
    {
        angle = kf_fixed_add( angle, KF_FIXED_TWO_PI );
    }
    return angle;
}

KF_FORCE_INLINE __kf_int64_t __kf_divide_round_nearest( __kf_int64_t numerator, __kf_int64_t denominator )
{
    const kf_bool_t negative = (kf_bool_t)(numerator < 0);
    const __kf_uint64_t magnitude = kf_internal_magnitude( numerator );
    const __kf_uint64_t rounded = (magnitude + (__kf_uint64_t)denominator / UINT64_C(2)) / (__kf_uint64_t)denominator;
    return negative == KF_TRUE ? -(__kf_int64_t)rounded : (__kf_int64_t)rounded;
}

kf_angle16_t kf_angle16_from_fixed_radians( kf_fixed_t radians )
{
    const __kf_int64_t wrapped = radians % KF_FIXED_TWO_PI;
    const __kf_int64_t scaled = wrapped * (__kf_int64_t)KF_ANGLE16_TURN;
    const __kf_int64_t ticks = __kf_divide_round_nearest( scaled, KF_FIXED_TWO_PI );
    return (kf_angle16_t)ticks;
}

kf_sangle16_t kf_sangle16_from_fixed_radians( kf_fixed_t radians )
{
    const kf_angle16_t wrapped = kf_angle16_from_fixed_radians( radians );
    if( wrapped <= (kf_angle16_t)INT16_MAX )
    {
        return (kf_sangle16_t)wrapped;
    }
    return (kf_sangle16_t)((int32_t)wrapped - (int32_t)KF_ANGLE16_TURN);
}

kf_fixed_t kf_angle16_to_fixed_radians( kf_angle16_t angle )
{
    return (kf_fixed_t)(((__kf_int64_t)angle * KF_FIXED_TWO_PI) / (__kf_int64_t)KF_ANGLE16_TURN);
}

kf_fixed_t kf_sangle16_to_fixed_radians( kf_sangle16_t angle )
{
    return (kf_fixed_t)(((__kf_int64_t)angle * KF_FIXED_TWO_PI) / (__kf_int64_t)KF_ANGLE16_TURN);
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
    if( result < (int32_t)minimum )
    {
        result = minimum;
    }
    if( result > (int32_t)maximum )
    {
        result = maximum;
    }
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
    if( sine != NULL )
    {
        *sine = __kf_angle16_sine( angle );
    }
    if( cosine != NULL )
    {
        *cosine = __kf_angle16_sine( (kf_angle16_t)(angle + KF_ANGLE16_QUARTER) );
    }
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

kf_vec3_t kf_vec3_neg( kf_vec3_t value )
{
    const kf_vec3_t result = {kf_fixed_neg( value.x ), kf_fixed_neg( value.y ), kf_fixed_neg( value.z )};
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

kf_vec3_t kf_vec3_add_mul( kf_vec3_t base, kf_vec3_t value, kf_fixed_t scalar )
{
    return __kf_vec3_madd( base, value, scalar );
}

kf_vec3_t kf_vec3_sub_mul( kf_vec3_t base, kf_vec3_t value, kf_fixed_t scalar )
{
    return __kf_vec3_msub( base, value, scalar );
}

kf_vec3_t kf_vec3_add_mul2( kf_vec3_t base, kf_vec3_t first, kf_fixed_t first_scalar, kf_vec3_t second, kf_fixed_t second_scalar )
{
    return __kf_vec3_madd2( base, first, first_scalar, second, second_scalar );
}

kf_vec3_t kf_vec3_mul_div( kf_vec3_t value, kf_fixed_t multiplier, kf_fixed_t divisor )
{
    return __kf_vec3_mul_div( value, multiplier, divisor );
}

kf_vec3_t kf_vec3_lerp( kf_vec3_t from, kf_vec3_t to, kf_fixed_t factor )
{
    const kf_vec3_t result = {kf_fixed_lerp( from.x, to.x, factor ), kf_fixed_lerp( from.y, to.y, factor ), kf_fixed_lerp( from.z, to.z, factor )};
    return result;
}

kf_bool_t kf_vec3_equal( kf_vec3_t left, kf_vec3_t right )
{
    return (kf_bool_t)(left.x == right.x && left.y == right.y && left.z == right.z);
}

KF_FORCE_INLINE kf_bool_t __kf_vec3_dot_q32( kf_vec3_t left, kf_vec3_t right, __kf_int64_t * result )
{
    __kf_int64_t sum;
    if( __kf_wide_add( (__kf_int64_t)left.x * (__kf_int64_t)right.x,
        (__kf_int64_t)left.y * (__kf_int64_t)right.y, &sum ) == KF_FALSE ) return KF_FALSE;
    return __kf_wide_add( sum, (__kf_int64_t)left.z * (__kf_int64_t)right.z, result );
}

kf_fixed_t kf_vec3_dot( kf_vec3_t left, kf_vec3_t right )
{
    __kf_int64_t wide;
    if( __kf_vec3_dot_q32( left, right, &wide ) == KF_FALSE )
    {
        return 0;
    }
    return __kf_narrow_q32( wide );
}

kf_vec3_t kf_vec3_cross( kf_vec3_t left, kf_vec3_t right )
{
    __kf_int64_t x;
    __kf_int64_t y;
    __kf_int64_t z;
    kf_vec3_t result;
    if( __kf_wide_sub( (__kf_int64_t)left.y * (__kf_int64_t)right.z,
        (__kf_int64_t)left.z * (__kf_int64_t)right.y, &x ) == KF_FALSE ) return (kf_vec3_t){0, 0, 0};
    if( __kf_wide_sub( (__kf_int64_t)left.z * (__kf_int64_t)right.x,
        (__kf_int64_t)left.x * (__kf_int64_t)right.z, &y ) == KF_FALSE ) return (kf_vec3_t){0, 0, 0};
    if( __kf_wide_sub( (__kf_int64_t)left.x * (__kf_int64_t)right.y,
        (__kf_int64_t)left.y * (__kf_int64_t)right.x, &z ) == KF_FALSE ) return (kf_vec3_t){0, 0, 0};
    result.x = __kf_narrow_q32( x );
    result.y = __kf_narrow_q32( y );
    result.z = __kf_narrow_q32( z );
    return result;
}

kf_fixed_t kf_vec3_length_squared( kf_vec3_t value )
{
    return kf_vec3_dot( value, value );
}
kf_fixed_t kf_vec3_length( kf_vec3_t value )
{
    __kf_int64_t square_q32;
    __kf_uint64_t root;
    if( __kf_vec3_dot_q32( value, value, &square_q32 ) == KF_FALSE || square_q32 < 0 )
    {
        return 0;
    }
    root = __kf_integer_sqrt_u64( (__kf_uint64_t)square_q32 );
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
    __kf_int64_t length_squared_q32;
    __kf_int64_t projection_q32;
    __kf_int64_t point_distance_q32;
    __kf_int64_t result_q32;
    __kf_uint64_t correction_q32;
    if( __kf_vec3_dot_q32( segment, segment, &length_squared_q32 ) == KF_FALSE ||
        __kf_vec3_dot_q32( relative, segment, &projection_q32 ) == KF_FALSE ||
        __kf_vec3_dot_q32( relative, relative, &point_distance_q32 ) == KF_FALSE ) return 0;
    if( length_squared_q32 == 0 || projection_q32 <= 0 )
    {
        return __kf_narrow_q32( point_distance_q32 );
    }
    if( projection_q32 >= length_squared_q32 )
    {
        __kf_int64_t end_distance_q32;
        const kf_vec3_t end_relative = kf_vec3_sub( point, end );
        if( __kf_vec3_dot_q32( end_relative, end_relative, &end_distance_q32 ) == KF_FALSE )
        {
            return 0;
        }
        return __kf_narrow_q32( end_distance_q32 );
    }
    /* The projection is below the squared length, so projection^2 / length^2
       stays below the projection itself. */
    correction_q32 = kf_internal_wide_quotient( kf_internal_wide_multiply( (__kf_uint64_t)projection_q32, (__kf_uint64_t)projection_q32 ), kf_internal_wide_make( 0, (__kf_uint64_t)length_squared_q32 ) );
    KF_DEBUG_REQUIRE( correction_q32 <= (__kf_uint64_t)INT64_MAX, "kinefix segment projection overflow" );
    if( correction_q32 > (__kf_uint64_t)INT64_MAX || __kf_wide_sub( point_distance_q32, (__kf_int64_t)correction_q32, &result_q32 ) == KF_FALSE )
    {
        return 0;
    }
    return __kf_narrow_q32( result_q32 );
}

__kf_uint64_t kf_splitmix64( __kf_uint64_t value )
{
    value += UINT64_C(0x9E3779B97F4A7C15);
    value = (value ^ (value >> 30)) * UINT64_C(0xBF58476D1CE4E5B9);
    value = (value ^ (value >> 27)) * UINT64_C(0x94D049BB133111EB);
    return value ^ (value >> 31);
}

void kf_pcg32_seed( kf_pcg32_t * random, __kf_uint64_t seed, __kf_uint64_t stream )
{
    if( random == NULL )
    {
        return;
    }
    random->state = 0;
    random->increment = (stream << 1u) | UINT64_C(1);
    (void)kf_pcg32_next( random );
    random->state += seed;
    (void)kf_pcg32_next( random );
}

uint32_t kf_pcg32_next( kf_pcg32_t * random )
{
    const __kf_uint64_t old = random->state;
    const uint32_t shifted = (uint32_t)(((old >> 18u) ^ old) >> 27u);
    const uint32_t rotation = (uint32_t)(old >> 59u);
    random->state = old * UINT64_C(6364136223846793005) + random->increment;
    return (shifted >> rotation) | (shifted << ((0u - rotation) & 31u));
}

uint32_t kf_pcg32_bounded( kf_pcg32_t * random, uint32_t bound )
{
    uint32_t threshold;
    if( bound == 0 )
    {
        return 0;
    }
    threshold = (0u - bound) % bound;
    for( ;; )
    {
        const uint32_t value = kf_pcg32_next( random );
        if( value >= threshold )
        {
            return value % bound;
        }
    }
}

void kf_pcg32_restore( kf_pcg32_t * random, __kf_uint64_t state, __kf_uint64_t stream )
{
    if( random == NULL )
    {
        return;
    }
    random->state = state;
    random->increment = stream | UINT64_C(1);
}
