#ifndef __FFWD_ORIG_H__
#define __FFWD_ORIG_H__

#include "qspinlock.h"
#include "komb.h"

// Delegation settings
#define MAX_CORES 96
#define CORES_PER_SOCKET 16
#define DELEGATION_CPU 0

#define PRINT_DEBUG_DELEGATION 0
#define DEBUG_DELEGATION 0
#define FFWD_STATS 0 //1

#define CACHELINE_ALIGNEMENT 64

#define DEFINE_FFWD(x)                                                      \
	struct orig_qspinlock(x) = (struct orig_qspinlock)__ORIG_QSPIN_LOCK_UNLOCKED

void ffwd_send_request(void (*ffwd_write_cs)(u32, u32, void*), u32 src_value, u32 dst_value, void* stats);

#if FFWD_STATS
extern void ffwd_print_stats(void);
#endif

/********************************************************************************
 * Delegation
 * ******************************************************************************/

struct ffwd_orig_delegation_request {
	union {
		struct {
			uint32_t cpu_id;                 // 4
			uint32_t socket_id;              // 4
			uint32_t cpu_id_on_socket;			// 4
			void (*req_func_ptr)(u32, u32, void*);// Function pointer
			u32 arg1;
			u32 arg2;
			void* arg3;
		};
		char alignment[CACHELINE_ALIGNEMENT];             // pad to 128-byte
	};
	union {
		bool toggle;                // 1 client write only; request active when != request toggle
		char alignment1[CACHELINE_ALIGNEMENT];
	};
};

extern struct ffwd_orig_delegation_request ffwd_orig_delegation_requests; 

struct ffwd_orig_response {
	union {
		u64 toggle;
		char alignment[CACHELINE_ALIGNEMENT];
	};
} ____cacheline_aligned;

struct ffwd_orig_delegation_server {
	struct ffwd_orig_response ffwd_orig_responses[MAX_CORES / CORES_PER_SOCKET]; // 1 toggle bit for each thread
};

extern void ffwd_orig_delegate_init(void);
extern void ffwd_orig_delegate_exit(void);
#endif
