#ifndef PROVIZIO_RADAR_API_ERRNO
#define PROVIZIO_RADAR_API_ERRNO

#include <errno.h>

#define PROVIZIO_E_TIMEOUT EAGAIN        // Operation timed out
#define PROVIZIO_E_SKIPPED ERANGE        // Packet skipped and ignored
#define PROVIZIO_E_OUT_OF_CONTEXTS EBUSY // Not enough contexts
#define PROVIZIO_E_PROTOCOL EPROTO       // Protocol error
#define PROVIZIO_E_ARGUMENT EINVAL       // Invalid argument
#define PROVIZIO_E_NOT_PERMITTED EPERM   // Operation not permitted

#endif // PROVIZIO_RADAR_API_ERRNO
