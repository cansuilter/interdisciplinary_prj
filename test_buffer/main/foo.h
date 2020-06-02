#ifndef FOO_DOT_H    /* This is an "include guard" */
#define FOO_DOT_H    /* prevents the file from being included twice. */
                     /* Including a header file twice causes all kinds */
                     /* of interesting problems.*/

#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"

/**
 * This is a function declaration.
 * It tells the compiler that the function exists somewhere.
 */
volatile int mert;
RingbufHandle_t buf_handle;
int buffer_creator(void);
void buffer_read(void);
#endif /* FOO_DOT_H */
