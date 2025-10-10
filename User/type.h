#ifndef __USER_TYPE__
#define __USER_TYPE__

#include "def.h"
#include <stdbool.h>
#include <stdint.h>

typedef float f32;
typedef double f64;
typedef uint32_t u32;
typedef uint8_t u8;
typedef uint16_t u16;
typedef int32_t i32;
typedef int8_t i8;
typedef int16_t i16;

extern const u32 TICK_PER_SECOND;
extern const f32 SECOND_PER_TICK;

typedef void (*proc)(void);

typedef struct {
  u8 dat[DMA_BUFFER_SIZE];
  u16 len;
} buf;

typedef struct {
  proc procs[PROC_LIST_SIZE];
  u32 state;
} procList;

// CAN Header Types
typedef struct {
  u32 StdId;              // Standard Identifier
  u32 ExtId;              // Extended Identifier
  u32 IDE;                // Identifier Type
  u32 RTR;                // Remote Transmission Request
  u32 DLC;                // Data Length Code
  u32 TransmitGlobalTime; // Transmit Global Time
} canTxH;

typedef struct {
  u32 StdId;            // Standard Identifier
  u32 ExtId;            // Extended Identifier
  u32 IDE;              // Identifier Type
  u32 RTR;              // Remote Transmission Request
  u32 DLC;              // Data Length Code
  u32 FilterMatchIndex; // Filter Match Index
  u32 Timestamp;        // Timestamp
} canRxH;

#endif /* __USER_TYPE__ */
