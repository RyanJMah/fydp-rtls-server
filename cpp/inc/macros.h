#pragma once

#include <stdio.h>
#include <cstring>

#define UNUSED          __attribute__((unused))
#define ALWAYS_INLINE   inline __attribute__((always_inline))

#define STR_IMPL_(x) #x      // stringify argument
#define STR(x) STR_IMPL_(x)  // indirection to expand argument macros

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

// Assert macros from: https://github.com/apple/darwin-xnu/blob/main/EXTERNAL_HEADERS/AssertMacros.h
#define require_noerr(err_code, exception_label)                                \
    do                                                                          \
    {                                                                           \
        if ( __builtin_expect(0 != (err_code), 0) )                             \
        {                                                                       \
            printf("Error %d at %s:%d\n", err_code, __FILENAME__, __LINE__);    \
            goto exception_label;                                               \
        }                                                                       \
    } while ( 0 )

#define require_noerr_action(err_code, exception_label, action)                 \
    do                                                                          \
    {                                                                           \
        if ( __builtin_expect(0 != (err_code), 0) )                             \
        {                                                                       \
            {                                                                   \
                action;                                                         \
            }                                                                   \
            printf("Error %d at %s:%d\n", err_code, __FILENAME__, __LINE__);    \
            goto exception_label;                                               \
        }                                                                       \
    } while ( 0 )

#include "DetourStatus.h"
#define require_dt_success(err_code, exception_label)                           \
    do                                                                          \
    {                                                                           \
        if ( __builtin_expect(DT_SUCCESS != (err_code), 0) )                    \
        {                                                                       \
            printf("Error 0x%x at %s:%d\n", err_code, __FILENAME__, __LINE__);  \
            goto exception_label;                                               \
        }                                                                       \
    } while ( 0 )


#define require(assertion, exception_label)                                             \
    do                                                                                  \
    {                                                                                   \
        if ( __builtin_expect(!(assertion), 0) )                                        \
        {                                                                               \
            printf("Failed assertion at %s:%d\n", __FILENAME__, __LINE__);              \
            goto exception_label;                                                       \
        }                                                                               \
    } while ( 0 )

#define require_action(assertion, exceptionLabel, action)                               \
    do                                                                                  \
    {                                                                                   \
        if ( __builtin_expect(!(assertion), 0) )                                        \
        {                                                                               \
            {                                                                           \
                action;                                                                 \
            }                                                                           \
            printf("Failed assertion at %s:%d\n", __FILENAME__, __LINE__);              \
            goto exceptionLabel;                                                        \
        }                                                                               \
    } while ( 0 )

#define goto_error(exception_label, msg)    \
    do                                      \
    {                                       \
        printf("%s\n", msg);                \
        goto exception_label;               \
    } while ( 0 )
