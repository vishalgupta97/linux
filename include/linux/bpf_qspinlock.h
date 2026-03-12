#ifndef __BPF_QSPINLOCK_H
#define __BPF_QSPINLOCK_H

#include <asm-generic/qspinlock_types.h>

void bpf_qspinlock_lock(struct qspinlock *lock);
void bpf_qspinlock_unlock(struct qspinlock *lock);

void __internal__bpf_spin_lock(struct qspinlock *lock);
void __internal__bpf_spin_unlock(struct qspinlock *lock);

#endif //__BPF_QSPINLOCK_H

