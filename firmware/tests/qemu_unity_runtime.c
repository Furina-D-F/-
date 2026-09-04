#include <stddef.h>

void *memset(void *destination, int value, size_t length)
{
    unsigned char *bytes = destination;
    for (size_t index = 0U; index < length; index++) {
        bytes[index] = (unsigned char) value;
    }
    return destination;
}

void *memcpy(void *destination, const void *source, size_t length)
{
    unsigned char *destination_bytes = destination;
    const unsigned char *source_bytes = source;
    for (size_t index = 0U; index < length; index++) {
        destination_bytes[index] = source_bytes[index];
    }
    return destination;
}
