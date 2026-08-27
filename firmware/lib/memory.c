#include <stddef.h>

void *memset(void *destination, int value, size_t size)
{
    unsigned char *bytes = destination;

    for (size_t index = 0; index < size; index++) {
        bytes[index] = (unsigned char) value;
    }

    return destination;
}