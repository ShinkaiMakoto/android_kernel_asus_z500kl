/* drivers/misc/lowmemorykiller.c
 *
 * The lowmemorykiller driver lets user-space specify a set of memory thresholds
 * where processes with a range of oom_score_adj values will get killed. Specify
 * the minimum oom_score_adj values in
 * /sys/module/lowmemorykiller/parameters/adj and the number of free pages in
 * /sys/module/lowmemorykiller/parameters/minfree. Both files take a comma
 * separated list of numbers in ascending order.
 *
 * For example, write "0,8" to /sys/module/lowmemorykiller/parameters/adj and
 * "1024,4096" to /sys/module/lowmemorykiller/parameters/minfree to kill
 * processes with a oom_score_adj value of 8 or higher when the free memory
 * drops below 4096 pages and kill processes with a oom_score_adj value of 0 or
 * higher when the free memory drops below 1024 pages.
 *
 * The driver considers memory used for caches to be free, but if a large
 * percentage of the cached memory is locked this can be very inaccurate
 * and processes may not get killed until the normal oom killer is triggered.
 *
 * Copyright (C) 2007-2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/oom.h>
#include <linux/sched.h>
#include <linux/rcupdate.h>
#include <linux/notifier.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/swap.h>
#include <linux/fs.h>
#include <linux/cpuset.h>
#include <linux/show_mem_notifier.h>
#include <linux/vmpressure.h>
#include <linux/hugetlb_inline.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#define CREATE_TRACE_POINTS
#include <trace/events/almk.h>

#ifdef CONFIG_HIGHMEM
#define _ZONE ZONE_HIGHMEM
#else
#define _ZONE ZONE_NORMAL
#endif

#define CREATE_TRACE_POINTS
#include "trace/lowmemorykiller.h"

static uint32_t lowmem_debug_level = 1;
static short lowmem_adj[6] = {
	0,
	1,
	6,
	12,
};
static int lowmem_adj_size = 4;
static int lowmem_minfree[6] = {
	3 * 512,	/* 6MB */
	2 * 1024,	/* 8MB */
	4 * 1024,	/* 16MB */
	16 * 1024,	/* 64MB */
};
static int lowmem_minfree_size = 4;
static int lmk_fast_run = 1;

static unsigned long lowmem_deathpending_timeout;
static unsigned long dump_meminfo_timeout;
static unsigned long dump_vmpressure_timeout = 0;
static struct workqueue_struct *dumpmem_workqueue;
static void dump_meminfo(struct work_struct *work);
static DECLARE_WORK(dumpmem_work, dump_meminfo);
#ifdef LMK_DEBUG
static unsigned long vm_pressure[7] = {0};
static unsigned long shrink_count = 0;
static unsigned long kill_count = 0;
#endif
static unsigned long vm_pressure_printed = 79;

#define lowmem_print(level, x...)			\
	do {						\
		if (lowmem_debug_level >= (level))	\
			pr_info(x);			\
	} while (0)

#define PSS_SHIFT 12
struct mem_size_stats {
    struct vm_area_struct *vma;
    unsigned long resident;
    unsigned long shared_clean;
    unsigned long shared_dirty;
    unsigned long private_clean;
    unsigned long private_dirty;
    unsigned long referenced;
    unsigned long anonymous;
    unsigned long anonymous_thp;
    unsigned long swap;
    unsigned long nonlinear;
    u64 pss;
};

#if 0
static unsigned long get_mm_pss(struct mm_struct *mm)
{
    unsigned long pss;
    struct vm_area_struct *vma = mm->mmap;
    struct mem_size_stats mss;
    struct mm_walk smaps_walk = {
            .pmd_entry = smaps_pte_range,
            .mm = vma->vm_mm,
            .private = &mss,
    };
    memset(&mss, 0, sizeof(struct mem_size_stats));
    while (vma)
    {
        mss.vma = vma;
        if (vma->vm_mm && !is_vm_hugetlb_page(vma))
            walk_page_range(vma->vm_start, vma->vm_end, &smaps_walk);
        vma = vma->vm_next;
    }
    pss = (unsigned long)(mss.pss >> (10 + PSS_SHIFT));
    return pss;
}
#endif

static void dump_meminfo(struct work_struct *work)
{
    struct task_struct *tsk;
    int tasksize;

    lowmem_print(1,"This is dump by LMK while adj = 0\n");
    for_each_process(tsk){
        struct task_struct *p;
        int oom_score_adj;

        if (tsk->flags & PF_KTHREAD)
            continue;

        p = find_lock_task_mm(tsk);
        if (!p)
            continue;

        oom_score_adj = p->signal->oom_score_adj;

        tasksize = get_mm_rss(p->mm);

        task_unlock(p);
        if (tasksize <= 0)
            continue;
        lowmem_print(1, "PID:%d P_Name:%s Adj:%d Size:%d KB\n"
          ,p->pid, p->comm, oom_score_adj, tasksize *4);
    }
}


static atomic_t shift_adj = ATOMIC_INIT(0);
static short adj_max_shift = 353;
module_param_named(adj_max_shift, adj_max_shift, short,
	S_IRUGO | S_IWUSR);

/* User knob to enable/disable adaptive lmk feature */
static int enable_adaptive_lmk;
module_param_named(enable_adaptive_lmk, enable_adaptive_lmk, int,
	S_IRUGO | S_IWUSR);

/*
 * This parameter controls the behaviour of LMK when vmpressure is in
 * the range of 90-94. Adaptive lmk triggers based on number of file
 * pages wrt vmpressure_file_min, when vmpressure is in the range of
 * 90-94. Usually this is a pseudo minfree value, higher than the
 * highest configured value in minfree array.
 */
static int vmpressure_file_min;
module_param_named(vmpressure_file_min, vmpressure_file_min, int,
	S_IRUGO | S_IWUSR);

enum {
	VMPRESSURE_NO_ADJUST = 0,
	VMPRESSURE_ADJUST_ENCROACH,
	VMPRESSURE_ADJUST_NORMAL,
};

int adjust_minadj(short *min_score_adj)
{
	int ret = VMPRESSURE_NO_ADJUST;

	if (!enable_adaptive_lmk)
		return 0;

	if (atomic_read(&shift_adj) &&
		(*min_score_adj > adj_max_shift)) {
		if (*min_score_adj == OOM_SCORE_ADJ_MAX + 1)
			ret = VMPRESSURE_ADJUST_ENCROACH;
		else
			ret = VMPRESSURE_ADJUST_NORMAL;
		*min_score_adj = adj_max_shift;
	}
	atomic_set(&shift_adj, 0);

	return ret;
}

static int lmk_vmpressure_notifier(struct notifier_block *nb,
			unsigned long action, void *data)
{
	int other_free, other_file;
	unsigned long pressure = action;
	int array_size = ARRAY_SIZE(lowmem_adj);

#ifdef LMK_DEBUG
	if (pressure >= 95)
		vm_pressure[0]++;
	else if (pressure >= 90)
		vm_pressure[1]++;
	else if (pressure >= 85)
		vm_pressure[2]++;
	else if (pressure >= 80)
		vm_pressure[3]++;
	else if (pressure >= 70)
		vm_pressure[4]++;
	else if (pressure >= 60)
		vm_pressure[5]++;
	else if (pressure > 1)
		vm_pressure[6]++;
#endif
	if (!enable_adaptive_lmk)
		return 0;

    if (pressure > vm_pressure_printed ||
            (!time_before_eq(jiffies, dump_vmpressure_timeout) && pressure >= 80)) {
        dump_vmpressure_timeout = jiffies + 3*HZ;
        vm_pressure_printed = pressure;

        lowmem_print(1, "VMPressure=%lu\n" \
                "    NR_FREE:        %8lu KB\n" \
                "    NR_FILE:        %8lu KB\n" \
                "    NR_SHMEM        %8lu KB\n" \
                "    swapcache:      %8lu KB\n" \
                "    totalreserve:   %8lu KB\n" \
                "    NR_FILE_MAPPED: %8lu KB\n",
                pressure,
                global_page_state(NR_FREE_PAGES)*(PAGE_SIZE / 1024),
                global_page_state(NR_FILE_PAGES)*(PAGE_SIZE / 1024),
                global_page_state(NR_SHMEM)*(PAGE_SIZE / 1024),
                total_swapcache_pages()*(PAGE_SIZE / 1024),
                totalreserve_pages*(PAGE_SIZE / 1024),
                global_page_state(NR_FILE_MAPPED)*(PAGE_SIZE / 1024)
        );
    }

	if (pressure >= 95) {
		other_file = global_page_state(NR_FILE_PAGES) -
			global_page_state(NR_SHMEM) -
			total_swapcache_pages();
		other_free = global_page_state(NR_FREE_PAGES);

		atomic_set(&shift_adj, 1);
		trace_almk_vmpressure(pressure, other_free, other_file);
	} else if (pressure >= 90) {
		if (lowmem_adj_size < array_size)
			array_size = lowmem_adj_size;
		if (lowmem_minfree_size < array_size)
			array_size = lowmem_minfree_size;

		other_file = global_page_state(NR_FILE_PAGES) -
			global_page_state(NR_SHMEM) -
			total_swapcache_pages();

		other_free = global_page_state(NR_FREE_PAGES);

		if ((other_free < lowmem_minfree[array_size - 1]) &&
			(other_file < vmpressure_file_min)) {
				atomic_set(&shift_adj, 1);
				trace_almk_vmpressure(pressure, other_free,
					other_file);
		}
	} else if (atomic_read(&shift_adj)) {
		/*
		 * shift_adj would have been set by a previous invocation
		 * of notifier, which is not followed by a lowmem_shrink yet.
		 * Since vmpressure has improved, reset shift_adj to avoid
		 * false adaptive LMK trigger.
		 */
		trace_almk_vmpressure(pressure, other_free, other_file);
		atomic_set(&shift_adj, 0);
	}

	return 0;
}

static struct notifier_block lmk_vmpr_nb = {
	.notifier_call = lmk_vmpressure_notifier,
};

static int test_task_flag(struct task_struct *p, int flag)
{
	struct task_struct *t;

	for_each_thread(p, t) {
		task_lock(t);
		if (test_tsk_thread_flag(t, flag)) {
			task_unlock(t);
			return 1;
		}
		task_unlock(t);
	}

	return 0;
}

static DEFINE_MUTEX(scan_mutex);

int can_use_cma_pages(gfp_t gfp_mask)
{
	int can_use = 0;
	int mtype = allocflags_to_migratetype(gfp_mask);
	int i = 0;
	int *mtype_fallbacks = get_migratetype_fallbacks(mtype);

	if (is_migrate_cma(mtype)) {
		can_use = 1;
	} else {
		for (i = 0;; i++) {
			int fallbacktype = mtype_fallbacks[i];

			if (is_migrate_cma(fallbacktype)) {
				can_use = 1;
				break;
			}

			if (fallbacktype == MIGRATE_RESERVE)
				break;
		}
	}
	return can_use;
}

void tune_lmk_zone_param(struct zonelist *zonelist, int classzone_idx,
					int *other_free, int *other_file,
					int use_cma_pages)
{
	struct zone *zone;
	struct zoneref *zoneref;
	int zone_idx;

	for_each_zone_zonelist(zone, zoneref, zonelist, MAX_NR_ZONES) {
		zone_idx = zonelist_zone_idx(zoneref);
		if (zone_idx == ZONE_MOVABLE) {
			if (!use_cma_pages && other_free)
				*other_free -=
				    zone_page_state(zone, NR_FREE_CMA_PAGES);
			continue;
		}

		if (zone_idx > classzone_idx) {
			if (other_free != NULL)
				*other_free -= zone_page_state(zone,
							       NR_FREE_PAGES);
			if (other_file != NULL)
				*other_file -= zone_page_state(zone,
							       NR_FILE_PAGES)
					- zone_page_state(zone, NR_SHMEM)
					- zone_page_state(zone, NR_SWAPCACHE);
		} else if (zone_idx < classzone_idx) {
			if (zone_watermark_ok(zone, 0, 0, classzone_idx, 0) &&
			    other_free) {
				if (!use_cma_pages) {
					*other_free -= min(
					  zone->lowmem_reserve[classzone_idx] +
					  zone_page_state(
					    zone, NR_FREE_CMA_PAGES),
					  zone_page_state(
					    zone, NR_FREE_PAGES));
				} else {
					*other_free -=
					  zone->lowmem_reserve[classzone_idx];
				}
			} else {
				if (other_free)
					*other_free -=
					  zone_page_state(zone, NR_FREE_PAGES);
			}
		}
	}
}

#ifdef CONFIG_HIGHMEM
void adjust_gfp_mask(gfp_t *gfp_mask)
{
	struct zone *preferred_zone;
	struct zonelist *zonelist;
	enum zone_type high_zoneidx;

	if (current_is_kswapd()) {
		zonelist = node_zonelist(0, *gfp_mask);
		high_zoneidx = gfp_zone(*gfp_mask);
		first_zones_zonelist(zonelist, high_zoneidx, NULL,
				&preferred_zone);

		if (high_zoneidx == ZONE_NORMAL) {
			if (zone_watermark_ok_safe(preferred_zone, 0,
					high_wmark_pages(preferred_zone), 0,
					0))
				*gfp_mask |= __GFP_HIGHMEM;
		} else if (high_zoneidx == ZONE_HIGHMEM) {
			*gfp_mask |= __GFP_HIGHMEM;
		}
	}
}
#else
void adjust_gfp_mask(gfp_t *unused)
{
}
#endif

void tune_lmk_param(int *other_free, int *other_file, struct shrink_control *sc)
{
	gfp_t gfp_mask;
	struct zone *preferred_zone;
	struct zonelist *zonelist;
	enum zone_type high_zoneidx, classzone_idx;
	unsigned long balance_gap;
	int use_cma_pages;

	gfp_mask = sc->gfp_mask;
	adjust_gfp_mask(&gfp_mask);

	zonelist = node_zonelist(0, gfp_mask);
	high_zoneidx = gfp_zone(gfp_mask);
	first_zones_zonelist(zonelist, high_zoneidx, NULL, &preferred_zone);
	classzone_idx = zone_idx(preferred_zone);
	use_cma_pages = can_use_cma_pages(gfp_mask);

	balance_gap = min(low_wmark_pages(preferred_zone),
			  (preferred_zone->present_pages +
			   KSWAPD_ZONE_BALANCE_GAP_RATIO-1) /
			   KSWAPD_ZONE_BALANCE_GAP_RATIO);

	if (likely(current_is_kswapd() && zone_watermark_ok(preferred_zone, 0,
			  high_wmark_pages(preferred_zone) + SWAP_CLUSTER_MAX +
			  balance_gap, 0, 0))) {
		if (lmk_fast_run)
			tune_lmk_zone_param(zonelist, classzone_idx, other_free,
				       other_file, use_cma_pages);
		else
			tune_lmk_zone_param(zonelist, classzone_idx, other_free,
				       NULL, use_cma_pages);

		if (zone_watermark_ok(preferred_zone, 0, 0, _ZONE, 0)) {
			if (!use_cma_pages) {
				*other_free -= min(
				  preferred_zone->lowmem_reserve[_ZONE]
				  + zone_page_state(
				    preferred_zone, NR_FREE_CMA_PAGES),
				  zone_page_state(
				    preferred_zone, NR_FREE_PAGES));
			} else {
				*other_free -=
				  preferred_zone->lowmem_reserve[_ZONE];
			}
		} else {
			*other_free -= zone_page_state(preferred_zone,
						      NR_FREE_PAGES);
		}

		lowmem_print(4, "lowmem_shrink of kswapd tunning for highmem "
			     "ofree %d, %d\n", *other_free, *other_file);
	} else {
		tune_lmk_zone_param(zonelist, classzone_idx, other_free,
			       other_file, use_cma_pages);

		if (!use_cma_pages) {
			*other_free -=
			  zone_page_state(preferred_zone, NR_FREE_CMA_PAGES);
		}

		lowmem_print(4, "lowmem_shrink tunning for others ofree %d, "
			     "%d\n", *other_free, *other_file);
	}
}

static int lowmem_shrink(struct shrinker *s, struct shrink_control *sc)
{
	struct task_struct *tsk;
	struct task_struct *selected = NULL;
	struct task_struct *selected_qos = NULL;
	int rem = 0;
	int tasksize;
	int i;
	int ret = 0;
	short min_score_adj = OOM_SCORE_ADJ_MAX + 1;
	int minfree = 0;
	int selected_tasksize = 0, selected_qos_tasksize;
	short selected_oom_score_adj, selected_qos_oom_score_adj;
	bool selected_mark_as_prefer_kill = 0;
	int array_size = ARRAY_SIZE(lowmem_adj);
	int other_free;
	int other_file;
	unsigned long nr_to_scan = sc->nr_to_scan;

	if (nr_to_scan > 0) {
		if (mutex_lock_interruptible(&scan_mutex) < 0)
			return 0;
	}
#ifdef LMK_DEBUG
	shrink_count++;
#endif
	other_free = global_page_state(NR_FREE_PAGES);

	if (global_page_state(NR_SHMEM) + total_swapcache_pages() <
		global_page_state(NR_FILE_PAGES))
		other_file = global_page_state(NR_FILE_PAGES) -
						global_page_state(NR_SHMEM) -
						total_swapcache_pages();
	else
		other_file = 0;

	tune_lmk_param(&other_free, &other_file, sc);

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;
	if (lowmem_minfree_size < array_size)
		array_size = lowmem_minfree_size;
	for (i = 0; i < array_size; i++) {
		minfree = lowmem_minfree[i];
		if (other_free < minfree && other_file < minfree) {
			min_score_adj = lowmem_adj[i];
			break;
		}
	}
	if (nr_to_scan > 0) {
		ret = adjust_minadj(&min_score_adj);
		lowmem_print(3, "lowmem_shrink %lu, %x, ofree %d %d, ma %hd\n",
				nr_to_scan, sc->gfp_mask, other_free,
				other_file, min_score_adj);
	}

	rem = global_page_state(NR_ACTIVE_ANON) +
		global_page_state(NR_ACTIVE_FILE) +
		global_page_state(NR_INACTIVE_ANON) +
		global_page_state(NR_INACTIVE_FILE);
	if (nr_to_scan <= 0 || min_score_adj == OOM_SCORE_ADJ_MAX + 1) {
		lowmem_print(5, "lowmem_shrink %lu, %x, return %d\n",
			     nr_to_scan, sc->gfp_mask, rem);

		if (nr_to_scan > 0)
			mutex_unlock(&scan_mutex);

		if ((min_score_adj == OOM_SCORE_ADJ_MAX + 1) &&
			(nr_to_scan > 0))
			trace_almk_shrink(0, ret, other_free, other_file, 0);

		return rem;
	}
	selected_oom_score_adj = min_score_adj;

	rcu_read_lock();
	for_each_process(tsk) {
		struct task_struct *p;
		short oom_score_adj;
		short oom_qos;
		unsigned long task_life_time;
		struct timespec delta;
		short oom_qos_endurance;
		bool oom_qos_prefer_kill;

		if (tsk->flags & PF_KTHREAD)
			continue;

		/* if task no longer has any memory ignore it */
		if (test_task_flag(tsk, TIF_MM_RELEASED))
			continue;

		if (time_before_eq(jiffies, lowmem_deathpending_timeout)) {
			if (test_task_flag(tsk, TIF_MEMDIE)) {
				rcu_read_unlock();
				/* give the system time to free up the memory */
				msleep_interruptible(20);
				mutex_unlock(&scan_mutex);
				return 0;
			}
		}

		p = find_lock_task_mm(tsk);
		if (!p)
			continue;

		oom_score_adj = p->signal->oom_score_adj;
		oom_qos = p->signal->oom_qos;
		get_oom_qos_endurance(oom_qos_endurance, oom_qos);
		oom_qos_prefer_kill = is_prefer_to_kill(oom_qos);
		if (oom_score_adj < min_score_adj) {
			task_unlock(p);
			continue;
		}
		tasksize = get_mm_rss(p->mm);
		task_unlock(p);
		if (tasksize <= 0)
		    continue;
		//ASUS_BSP:ignore media if it's working
		if(oom_score_adj == 117 && !strcmp(p->comm,"d.process.media")){
		    lowmem_print(2, "adj == 117 and media, not killed !!!!\n");
		    continue;
		}
		if(!strcmp(p->comm,"FinalizerDaemon")){
			lowmem_print(2, "FinalizerDaemon, not killed !!!!\n");
			continue;
		}

		if(!strcmp(p->comm,"GC")){
			lowmem_print(2, "GC, not killed !!!!\n");
			continue;
		}
		/* Exame QOS level of this task */
		if (oom_qos_endurance > 0 && oom_score_adj >= 470) {
			get_monotonic_boottime(&delta);
			delta = timespec_sub(delta, p->real_start_time);
			task_life_time = timespec_to_jiffies(&delta);
			if (task_life_time < oom_qos_endurance*HZ) {
				lowmem_print(3, "skip '%s' (%d), adj %hd, for QOS. lifeTime(ms) %u, \n",
						p->comm, p->pid, oom_score_adj,
						jiffies_to_msecs(task_life_time));
				/* This is a candicate for killing. */
				if (selected_qos) {
					if (oom_score_adj < selected_qos_oom_score_adj)
						continue;
					if (oom_score_adj == selected_qos_oom_score_adj &&
							tasksize <= selected_qos_tasksize) {
						continue;
					}
				}
				selected_qos = p;
				selected_qos_tasksize = tasksize;
				selected_qos_oom_score_adj = oom_score_adj;
				continue;
			}
		}
		if (selected) {
			if (oom_score_adj < selected_oom_score_adj)
				continue;
			//For foreground and previous app, choice app allocating lower memory
			if (oom_score_adj == 0 || oom_score_adj == 411) {
				if (oom_score_adj == selected_oom_score_adj &&
						tasksize > selected_tasksize)
					continue;
			} else {
				if (oom_score_adj == selected_oom_score_adj) {
					/* the same adj level */
					if (selected_mark_as_prefer_kill &&
							!oom_qos_prefer_kill)
						continue;
					if (tasksize <= selected_tasksize) {
						continue;
					}
				}
			}
		}
		selected = p;
		selected_tasksize = tasksize;
		selected_oom_score_adj = oom_score_adj;
		selected_mark_as_prefer_kill = oom_qos_prefer_kill;
		lowmem_print(3, "select '%s' (%d), adj %hd, size %d, to kill\n",
				p->comm, p->pid, oom_score_adj, tasksize);
	}
	/* make a decision between selected and selected_qos */
	if (selected && selected_qos && selected_oom_score_adj < 470) {
		/* Previous app or more critical */
		if (selected_qos_oom_score_adj > selected_oom_score_adj ||
				(selected_qos_oom_score_adj == selected_oom_score_adj &&
				selected_qos_tasksize > selected_tasksize)) {
			selected = selected_qos;
			selected_tasksize = selected_qos_tasksize;
			selected_oom_score_adj = selected_qos_oom_score_adj;
		}
	}
	if (selected) {
		long cache_size = other_file * (long)(PAGE_SIZE / 1024);
		long cache_limit = minfree * (long)(PAGE_SIZE / 1024);
		long free = other_free * (long)(PAGE_SIZE / 1024);
                if (selected_oom_score_adj <= 0)
                {
                    lowmem_print(1,"dump_timeout: %lu , jiffies: %lu\n",dump_meminfo_timeout,jiffies);
                    if (!time_before_eq(jiffies, dump_meminfo_timeout) || dump_meminfo_timeout==0)
                    {
                        dump_meminfo_timeout = jiffies + 5*HZ;
                        queue_work(dumpmem_workqueue, &dumpmem_work);
                    }
                }

		trace_lowmemory_kill(selected, cache_size, cache_limit, free);
		lowmem_print(1, "Killing '%s' (%d), adj %hd,\n" \
				"   to free %ldkB on behalf of '%s' (%d) because\n" \
				"   cache %ldkB is below limit %ldkB for oom_score_adj %hd\n" \
				"   Free memory is %ldkB above reserved.\n" \
				"   Free CMA is %ldkB\n" \
				"   Total reserve is %ldkB\n" \
				"   Total free pages is %ldkB\n" \
				"   Total file cache is %ldkB\n" \
				"   Slab Reclaimable is %ldkB\n" \
				"   Slab UnReclaimable is %ldkB\n" \
				"   Total Slab is %ldkB\n" \
				"   GFP mask is 0x%x\n",
			     selected->comm, selected->pid,
			     selected_oom_score_adj,
			     selected_tasksize * (long)(PAGE_SIZE / 1024),
			     current->comm, current->pid,
			     cache_size, cache_limit,
			     min_score_adj,
			     free ,
			     global_page_state(NR_FREE_CMA_PAGES) *
				(long)(PAGE_SIZE / 1024),
			     totalreserve_pages * (long)(PAGE_SIZE / 1024),
			     global_page_state(NR_FREE_PAGES) *
				(long)(PAGE_SIZE / 1024),
			     global_page_state(NR_FILE_PAGES) *
				(long)(PAGE_SIZE / 1024),
			     global_page_state(NR_SLAB_RECLAIMABLE) *
				(long)(PAGE_SIZE / 1024),
			     global_page_state(NR_SLAB_UNRECLAIMABLE) *
				(long)(PAGE_SIZE / 1024),
			     global_page_state(NR_SLAB_RECLAIMABLE) *
				(long)(PAGE_SIZE / 1024) +
			     global_page_state(NR_SLAB_UNRECLAIMABLE) *
				(long)(PAGE_SIZE / 1024),
			     sc->gfp_mask);

#ifdef LMK_DEBUG
		kill_count++;
#endif
		if (lowmem_debug_level >= 2 && selected_oom_score_adj == 0) {
			show_mem(SHOW_MEM_FILTER_NODES);
			dump_tasks(NULL, NULL);
			show_mem_call_notifiers();
		}

		lowmem_deathpending_timeout = jiffies + HZ;
		send_sig(SIGKILL, selected, 0);
		set_tsk_thread_flag(selected, TIF_MEMDIE);
		rem -= selected_tasksize;
		rcu_read_unlock();
		/* give the system time to free up the memory */
		msleep_interruptible(20);
		trace_almk_shrink(selected_tasksize, ret,
			other_free, other_file, selected_oom_score_adj);
	} else {
		trace_almk_shrink(1, ret, other_free, other_file, 0);
		rcu_read_unlock();
	}

	lowmem_print(4, "lowmem_shrink %lu, %x, return %d\n",
		     nr_to_scan, sc->gfp_mask, rem);
	mutex_unlock(&scan_mutex);
	return rem;
}

static int lmk_proc_show(struct seq_file *m, void *v)
{
    unsigned long p = 0;
    int other_file, other_free;
#ifdef LMK_DEBUG
    int i;
#endif

	if (global_page_state(NR_SHMEM) + total_swapcache_pages() <
		global_page_state(NR_FILE_PAGES))
		other_file = global_page_state(NR_FILE_PAGES) -
						global_page_state(NR_SHMEM) -
						total_swapcache_pages();
	else
		other_file = 0;
	other_free = global_page_state(NR_FREE_PAGES);

#ifdef LMK_DEBUG
	seq_printf(m, "vmpressure: ");
	for (i = 0; i < 7; i++) {
		seq_printf(m, " %lu", vm_pressure[i]);
	}
	seq_printf(m, "\nshrink:         %8lu\n", shrink_count);
	seq_printf(m, "killed:         %8lu\n", kill_count);
#endif
    seq_printf(m, "pressure:       %8lu\n", p);
    seq_printf(m, "reserve_KB:     %8lu\n", (totalreserve_pages)*(PAGE_SIZE / 1024));
    seq_printf(m, "ofile_KB:       %8lu\n", other_file*(PAGE_SIZE / 1024));
    seq_printf(m, "ofree_KB:       %8lu\n", other_free*(PAGE_SIZE / 1024));
    return 0;
}

static int lmk_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, lmk_proc_show, NULL);
}

static const struct file_operations lmk_proc_fops = {
    .owner = THIS_MODULE,
    .open = lmk_proc_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};

static struct shrinker lowmem_shrinker = {
	.shrink = lowmem_shrink,
	.seeks = DEFAULT_SEEKS * 16
};

static int __init lowmem_init(void)
{
	register_shrinker(&lowmem_shrinker);
	vmpressure_notifier_register(&lmk_vmpr_nb);
        dumpmem_workqueue = create_workqueue("dumpmem_wq");
        proc_create("lmkinfo", 0444, NULL, &lmk_proc_fops);
	return 0;
}

static void __exit lowmem_exit(void)
{
        destroy_workqueue(dumpmem_workqueue);
	unregister_shrinker(&lowmem_shrinker);
}

#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES
static short lowmem_oom_adj_to_oom_score_adj(short oom_adj)
{
	if (oom_adj == OOM_ADJUST_MAX)
		return OOM_SCORE_ADJ_MAX;
	else
		return (oom_adj * OOM_SCORE_ADJ_MAX) / -OOM_DISABLE;
}

static void lowmem_autodetect_oom_adj_values(void)
{
	int i;
	short oom_adj;
	short oom_score_adj;
	int array_size = ARRAY_SIZE(lowmem_adj);

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;

	if (array_size <= 0)
		return;

	oom_adj = lowmem_adj[array_size - 1];
	if (oom_adj > OOM_ADJUST_MAX)
		return;

	oom_score_adj = lowmem_oom_adj_to_oom_score_adj(oom_adj);
	if (oom_score_adj <= OOM_ADJUST_MAX)
		return;

	lowmem_print(1, "lowmem_shrink: convert oom_adj to oom_score_adj:\n");
	for (i = 0; i < array_size; i++) {
		oom_adj = lowmem_adj[i];
		oom_score_adj = lowmem_oom_adj_to_oom_score_adj(oom_adj);
		lowmem_adj[i] = oom_score_adj;
		lowmem_print(1, "oom_adj %d => oom_score_adj %d\n",
			     oom_adj, oom_score_adj);
	}
}

static int lowmem_adj_array_set(const char *val, const struct kernel_param *kp)
{
	int ret;

	ret = param_array_ops.set(val, kp);

	/* HACK: Autodetect oom_adj values in lowmem_adj array */
	lowmem_autodetect_oom_adj_values();

	return ret;
}

static int lowmem_adj_array_get(char *buffer, const struct kernel_param *kp)
{
	return param_array_ops.get(buffer, kp);
}

static void lowmem_adj_array_free(void *arg)
{
	param_array_ops.free(arg);
}

static struct kernel_param_ops lowmem_adj_array_ops = {
	.set = lowmem_adj_array_set,
	.get = lowmem_adj_array_get,
	.free = lowmem_adj_array_free,
};

static const struct kparam_array __param_arr_adj = {
	.max = ARRAY_SIZE(lowmem_adj),
	.num = &lowmem_adj_size,
	.ops = &param_ops_short,
	.elemsize = sizeof(lowmem_adj[0]),
	.elem = lowmem_adj,
};
#endif

module_param_named(cost, lowmem_shrinker.seeks, int, S_IRUGO | S_IWUSR);
#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES
__module_param_call(MODULE_PARAM_PREFIX, adj,
		    &lowmem_adj_array_ops,
		    .arr = &__param_arr_adj,
		    S_IRUGO | S_IWUSR, -1);
__MODULE_PARM_TYPE(adj, "array of short");
#else
module_param_array_named(adj, lowmem_adj, short, &lowmem_adj_size,
			 S_IRUGO | S_IWUSR);
#endif
module_param_array_named(minfree, lowmem_minfree, uint, &lowmem_minfree_size,
			 S_IRUGO | S_IWUSR);
module_param_named(debug_level, lowmem_debug_level, uint, S_IRUGO | S_IWUSR);
module_param_named(lmk_fast_run, lmk_fast_run, int, S_IRUGO | S_IWUSR);

module_init(lowmem_init);
module_exit(lowmem_exit);

MODULE_LICENSE("GPL");

