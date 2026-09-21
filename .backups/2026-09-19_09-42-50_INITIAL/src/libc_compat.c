/*
 * libc_compat.c
 *
 * Minimal libc memory/string functions for bare-metal builds.
 *
 * Use this only when the S32DS project is linked without
 * standard newlib / libc support.
 */

#include <stddef.h>


/* ========================================================================== */
/* MEMSET                                                                     */
/* ========================================================================== */

void *memset(
    void *dest,
    int value,
    size_t count)
{
    unsigned char *p;

    p =
        (unsigned char *)dest;

    while(count-- != 0U)
    {
        *p++ =
            (unsigned char)value;
    }

    return dest;
}


/* ========================================================================== */
/* MEMCPY                                                                     */
/* ========================================================================== */

void *memcpy(
    void *dest,
    const void *src,
    size_t count)
{
    unsigned char *d;
    const unsigned char *s;

    d =
        (unsigned char *)dest;

    s =
        (const unsigned char *)src;

    while(count-- != 0U)
    {
        *d++ =
            *s++;
    }

    return dest;
}


/* ========================================================================== */
/* MEMMOVE                                                                    */
/* ========================================================================== */

void *memmove(
    void *dest,
    const void *src,
    size_t count)
{
    unsigned char *d;
    const unsigned char *s;

    d =
        (unsigned char *)dest;

    s =
        (const unsigned char *)src;


    if(d == s)
    {
        return dest;
    }


    if(d < s)
    {
        while(count-- != 0U)
        {
            *d++ =
                *s++;
        }
    }
    else
    {
        d += count;

        s += count;

        while(count-- != 0U)
        {
            *--d =
                *--s;
        }
    }

    return dest;
}


/* ========================================================================== */
/* STRLEN                                                                     */
/* ========================================================================== */

size_t strlen(const char *s)
{
    size_t n = 0U;


    while(*s != '\0')
    {
        n++;
        s++;
    }

    return n;
}
