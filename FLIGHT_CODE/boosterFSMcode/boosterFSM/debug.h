#ifndef DEBUG_H__
#define DEBUG_H__

#ifdef DEBUG
#   define DEBUG_PRINT(msg) do { \
    Serial.print("[DEBUG]: ");   \
    Serial.println(msg);         \
    } while (0)
    const bool __debug_enabled = true;
#else
#   define DEBUG_PRINT(msg)
    const bool __debug_enabled = false;
#endif

#endif // DEBUG_H__