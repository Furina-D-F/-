#include <stddef.h>

static int errno_value;

int *__errno(void)
{
    return &errno_value;
}

void *memset(void *destination, int value, size_t size)
{
    unsigned char *bytes = destination;

    for (size_t index = 0; index < size; index++) {
        bytes[index] = (unsigned char) value;
    }

    return destination;
}

void *memcpy(void *destination, const void *source, size_t size)
{
    unsigned char *destination_bytes = destination;
    const unsigned char *source_bytes = source;

    for (size_t index = 0; index < size; index++) {
        destination_bytes[index] = source_bytes[index];
    }

    return destination;
}