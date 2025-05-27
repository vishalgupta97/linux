// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2024 Vishal Gupta

inline const char *get_str_ltype(enum fds_lock_type ltype)
{
	switch (ltype) {
	case FDS_SPINLOCK:
		return "SPINLOCK";
	case FDS_MUTEX:
		return "MUTEX";
	case FDS_READ_SEM:
		return "SEMAPHORE-READ";
	case FDS_WRITE_SEM:
		return "SEMAPHORE-WRITE";
	default:
		return "UNDEFINED";
	}
}

inline const char *get_str_lockm(enum fds_lock_mechanisms lockm)
{
	switch (lockm) {
	case FDS_QSPINLOCK:
		return "QSPINLOCK";
	case FDS_CNA:
		return "CNA";
	case FDS_TCLOCK:
		return "TCLOCK";
	case FDS_TDLOCK:
		return "TDLOCK";
	case FDS_PERCPU:
		return "PERCPU";
	default:
		return "UNDEFINED";
	}
}
