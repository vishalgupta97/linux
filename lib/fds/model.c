// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2024 Vishal Gupta

#include "fds.h"

#if FDS_HOSTNAME == FDS_SRV1
#include <fds/srv1_spinlock_random_forest.h>
#include <fds/srv1_rwsem_random_forest.h>
#define SPINLOCK_MODEL predict_srv1_spinlock_random_forest
#define RWSEM_MODEL predict_srv1_rwsem_random_forest
#elif FDS_HOSTNAME == FDS_SRV8
#include <fds/srv8_spinlock_random_forest.h>
#include <fds/srv8_rwsem_random_forest.h>
#define SPINLOCK_MODEL predict_srv8_spinlock_random_forest
#define RWSEM_MODEL predict_srv8_rwsem_random_forest
#elif FDS_HOSTNAME == FDS_SRV9
#include <fds/srv9_spinlock_random_forest.h>
#include <fds/srv9_rwsem_random_forest.h>
#define SPINLOCK_MODEL predict_srv9_spinlock_random_forest
#define RWSEM_MODEL predict_srv9_rwsem_random_forest
#else
#error "Model for current host not found"
#endif

inline enum fds_lock_mechanisms
get_optimal_spinlock_random_forest_classifier(struct lock_stat *tmp)
{
	int feature_vector[2];
	feature_vector[0] = cpumask_weight(&tmp->contending_cpus);
	feature_vector[1] = (tmp->counter) / (fds_monitor_time / 1000) /
			    (feature_vector[0]);
	int optimal_index = SPINLOCK_MODEL(feature_vector);
	enum fds_lock_mechanisms next_lock_type = FDS_QSPINLOCK;
	switch (optimal_index) {
	case 0:
		next_lock_type = FDS_QSPINLOCK;
		break; //AQS
	case 1:
		next_lock_type = FDS_QSPINLOCK;
		break;
	case 2:
		next_lock_type = FDS_TCLOCK;
		break; //TCLOCK
	case 3:
		next_lock_type = FDS_TCLOCK;
		break; //TDLOCK
	}

	printk(KERN_ALERT
	       "SPINLOCK CPUCNT:%d RPS:%d Max RPS: %lld Min RPS: %lld next_lock: %s\n",
	       feature_vector[0], feature_vector[1],
	       (tmp->max_counter) / (fds_monitor_time / 1000),
	       (tmp->min_counter) / (fds_monitor_time / 1000),
	       get_str_lockm(next_lock_type));
	return next_lock_type;
}

inline enum fds_lock_mechanisms
get_optimal_rwsem_random_forest_classifier(struct lock_stat *tmp)
{
	int feature_vector[3];
	feature_vector[0] =
		(tmp->counter * 100) / (tmp->read_counter + tmp->counter);
	feature_vector[1] = cpumask_weight(&tmp->contending_cpus);
	feature_vector[2] = (tmp->counter + tmp->read_counter) /
			    (fds_monitor_time / 1000) / (feature_vector[1]);
	int optimal_index = RWSEM_MODEL(feature_vector);
	enum fds_lock_mechanisms next_lock_type = FDS_QSPINLOCK;
	switch (optimal_index) {
	case 0:
		next_lock_type = FDS_PERCPU;
		break;
	case 1:
		next_lock_type = FDS_QSPINLOCK;
		break;
	case 2:
		next_lock_type = FDS_TCLOCK;
		break;
	case 3:
		next_lock_type = FDS_TCLOCK;
		break; //TDLOCK
	}

	printk(KERN_ALERT
	       "RWSEM RWRATIO: %d CPUCNT:%d RPS:%d Max RPS: %lld Min RPS: %lld next_lock: %s\n",
	       feature_vector[0], feature_vector[1], feature_vector[2],
	       (tmp->max_counter) / (fds_monitor_time / 1000),
	       (tmp->min_counter) / (fds_monitor_time / 1000),
	       get_str_lockm(next_lock_type));
	return next_lock_type;
}
