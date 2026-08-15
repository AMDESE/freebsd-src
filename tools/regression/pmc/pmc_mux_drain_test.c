/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Regression test for the hwpmc multiplex "switch-off" drain.
 *
 * Background
 * ----------
 * Evicting a process mux-group off hardware (either during rotation via
 * pmu_pp_schedule_out -> pmc_rotation_drain, or at teardown via
 * pmc_release_pmc_descriptor -> pmc_wait_for_pmc_idle) waits until every
 * sibling PMC's pm_runcount falls to zero.  pm_runcount only drops when
 * the CPUs that currently hold the counter run csw_out.  The historical
 * drain just pause()d on the calling CPU and hoped the remote targets
 * would context-switch on their own.
 *
 * That is unsafe: a counting-mode target thread that is pinned and CPU
 * bound, and is the only runnable thread on its core, is never
 * involuntarily preempted, so it never runs csw_out.  pm_runcount then
 * never reaches zero and the drain spins until the INVARIANTS cap trips
 * and panics the machine (or, on a production kernel, stalls for a long
 * time while holding pmc_sx exclusive).
 *
 * This test manufactures exactly that condition:
 *   - spawn a handful of worker threads, each PINNED to a distinct CPU
 *     and boosted to real-time priority, spinning in a pure CPU loop;
 *   - attach two mux groups (TC counting mode) whose union oversubscribes
 *     the core PMC pool, so the rotation kthread must evict a whole group
 *     every mux period;
 *   - drive the mux period as low as the kernel allows so eviction (and
 *     therefore the drain) happens thousands of times per second;
 *   - then tear the groups down (stop + release) while the workers are
 *     STILL spinning, exercising the pmc_wait_for_pmc_idle drain with the
 *     counters still loaded on the pinned RT workers.
 *
 * On a kernel with the passive drain this reliably panics (INVARIANTS) or
 * wedges under pmc_sx.  On a kernel that actively migrates onto each
 * loaded CPU to force the resident target off (pmc_select_cpu at PRI_MIN
 * preempts even an RT worker), every eviction and the teardown complete
 * promptly and the test prints OK.
 *
 * Build:  cc -o pmc_mux_drain_test pmc_mux_drain_test.c -lpmc -lpthread
 * Run:    sudo ./pmc_mux_drain_test   (requires hwpmc loaded, AMD CPU)
 *
 * Exit codes: 0 = pass, 1 = fail, 77 = skip.
 */

#include <sys/types.h>
#include <sys/param.h>
#include <sys/cpuset.h>
#include <sys/rtprio.h>
#include <sys/sysctl.h>

#include <err.h>
#include <errno.h>
#include <pmc.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define	MAX_GROUPS	2
#define	MAX_PER_GROUP	8
#define	MAX_WORKERS	8

/*
 * How long the machine is allowed to be busy before we declare the drain
 * wedged.  On the fixed kernel the whole run finishes in a couple of
 * seconds; on a production (non-INVARIANTS) kernel with the old passive
 * drain the pmc syscalls block under pmc_sx and this watchdog fires.
 * (With INVARIANTS the old kernel panics outright before we get here.)
 */
#define	WATCHDOG_SECS	60
#define	STRESS_SECS	3

static const char *event_pool[] = {
	"instructions",
	"unhalted-cycles",
	"branches",
	"branch-misses",
	"cache-references",
	"cache-misses",
	"l1d-loads",
	"l1d-load-misses",
	"l2-cache-references",
	"l2-cache-misses",
	"dispatch-stalls",
	"fp-retired",
	"branches-retired",
	"de-no-dispatch-per-slot",
	"ls_alloc_mab_count",
	"ls_not_halted_cyc",
	"ls_dispatch.all",
	"ls_mab_alloc.ls",
	"ls_mab_alloc.hwpf",
	"ls_mab_alloc.all",
	"ls_int_taken",
	"ls_stlf",
};
#define	POOL_SIZE	(int)(sizeof(event_pool) / sizeof(event_pool[0]))

struct pmu_grp {
	uint32_t	gid;
	int		nevents;
	pmc_id_t	ids[MAX_PER_GROUP];
	const char	*names[MAX_PER_GROUP];
};

static volatile int	g_stop;		/* tell workers to exit */
static volatile int	g_running;	/* workers that have spun up */

static int
is_amd(void)
{
	char buf[64];
	size_t s = sizeof(buf);

	if (sysctlbyname("kern.hwpmc.cpuid", buf, &s, NULL, 0) != 0)
		return (0);
	return (strstr(buf, "AuthenticAMD") != NULL ||
	    strstr(buf, "HygonGenuine") != NULL);
}

static int
probe_core_pmcs(void)
{
	pmc_id_t ids[64];
	int n = 0;

	while (n < (int)(sizeof(ids) / sizeof(ids[0]))) {
		if (pmc_allocate("instructions", PMC_MODE_TC, 0,
		    PMC_CPU_ANY, &ids[n], 0) < 0)
			break;
		n++;
	}
	for (int i = 0; i < n; i++)
		(void)pmc_release(ids[i]);
	return (n);
}

/*
 * Pinned, real-time-priority, CPU-bound worker.  Nothing here ever
 * blocks, so once this thread owns its core it will not voluntarily or
 * (short of a higher-priority thread) involuntarily context-switch --
 * which is precisely the state that starves the passive drain.
 */
static void *
worker(void *arg)
{
	cpuset_t mask;
	struct rtprio rtp;
	int cpu = (int)(intptr_t)arg;
	volatile uint64_t spin = 0;

	CPU_ZERO(&mask);
	CPU_SET(cpu, &mask);
	if (cpuset_setaffinity(CPU_LEVEL_WHICH, CPU_WHICH_TID, -1,
	    sizeof(mask), &mask) != 0)
		warn("cpuset_setaffinity cpu=%d", cpu);

	rtp.type = RTP_PRIO_REALTIME;
	rtp.prio = 30;			/* above timeshare, below clock */
	if (rtprio_thread(RTP_SET, 0, &rtp) != 0)
		warn("rtprio_thread cpu=%d (continuing at normal prio)", cpu);

	__atomic_add_fetch(&g_running, 1, __ATOMIC_SEQ_CST);

	while (g_stop == 0) {
		for (int k = 0; k < 100000; k++)
			spin++;
	}
	return (NULL);
}

static int
build_group(struct pmu_grp *g, int n_target, int pool_start)
{
	int i;

	if (pmc_group_create(&g->gid) < 0) {
		warn("pmc_group_create");
		return (-1);
	}
	g->nevents = 0;
	for (i = pool_start; i < POOL_SIZE && g->nevents < n_target; i++) {
		uint32_t flags = 0;
		pmc_id_t id;

		if (g->nevents == 0)
			flags |= PMC_F_GROUP_MUX;
		if (pmc_allocate_group(event_pool[i], PMC_MODE_TC, flags,
		    PMC_CPU_ANY, &id, 0) < 0)
			continue;
		if (pmc_group_add(g->gid, id, g->nevents == 0) < 0) {
			(void)pmc_release(id);
			continue;
		}
		g->ids[g->nevents] = id;
		g->names[g->nevents] = event_pool[i];
		g->nevents++;
	}
	return (g->nevents);
}

static void
release_group(struct pmu_grp *g)
{
	int i;

	for (i = 0; i < g->nevents; i++)
		(void)pmc_release(g->ids[i]);
	g->nevents = 0;
}

static void
on_watchdog(int sig __unused)
{
	static const char msg[] =
	    "\nFAIL: watchdog fired -- a mux drain is wedged waiting on "
	    "pm_runcount (a pinned RT target never context-switched out). "
	    "This is the passive-drain bug.\n";

	(void)write(STDERR_FILENO, msg, sizeof(msg) - 1);
	_exit(1);
}

int
main(void)
{
	struct pmu_grp grps[MAX_GROUPS];
	pthread_t tid[MAX_WORKERS];
	int core, ncpu, nworkers, saved_period, new_period;
	int per_group, total, i, j;
	size_t s;

	memset(grps, 0, sizeof(grps));

	if (pmc_init() < 0)
		err(1, "pmc_init");
	if (!is_amd()) {
		printf("SKIP: non-AMD CPU\n");
		return (77);
	}

	s = sizeof(ncpu);
	if (sysctlbyname("hw.ncpu", &ncpu, &s, NULL, 0) != 0 || ncpu < 2) {
		printf("SKIP: need >= 2 CPUs to pin a lone RT target\n");
		return (77);
	}

	core = probe_core_pmcs();
	if (core < 2) {
		printf("SKIP: only %d core PMCs available\n", core);
		return (77);
	}

	per_group = (core * 2) / 3;
	if (per_group < 2)
		per_group = 2;
	if (per_group > MAX_PER_GROUP)
		per_group = MAX_PER_GROUP;
	total = per_group * MAX_GROUPS;
	if (total <= core) {
		printf("SKIP: %d-counter CPU; cannot oversubscribe with "
		    "%d groups of %d\n", core, MAX_GROUPS, per_group);
		return (77);
	}

	/*
	 * Leave CPU 0 for the rotation kthread, this main thread and the
	 * rest of the system; pin the workers onto CPUs 1..nworkers.
	 */
	nworkers = ncpu - 1;
	if (nworkers > MAX_WORKERS)
		nworkers = MAX_WORKERS;

	printf("mux drain stress: %d groups x %d events on %d HW core "
	    "counters, %d pinned RT workers (cpus 1..%d)\n",
	    MAX_GROUPS, per_group, core, nworkers, nworkers);

	/* Build the groups from disjoint slices of the event pool. */
	for (i = 0; i < MAX_GROUPS; i++) {
		int got = build_group(&grps[i], per_group, i * per_group);

		if (got < per_group) {
			fprintf(stderr, "SKIP: only %d events for group %d "
			    "(needed %d)\n", got, i, per_group);
			for (j = 0; j <= i; j++)
				release_group(&grps[j]);
			return (77);
		}
	}

	for (i = 0; i < MAX_GROUPS; i++) {
		if (pmc_group_commit(grps[i].gid) < 0) {
			warn("pmc_group_commit g%d", i);
			for (j = 0; j < MAX_GROUPS; j++)
				release_group(&grps[j]);
			return (1);
		}
	}

	for (i = 0; i < MAX_GROUPS; i++)
		if (pmc_attach(grps[i].ids[0], getpid()) < 0) {
			warn("pmc_attach g%d leader", i);
			for (int k = 0; k < MAX_GROUPS; k++)
				release_group(&grps[k]);
			return (1);
		}

	/* Fastest rotation the kernel accepts, to hammer the drain. */
	new_period = 1;
	s = sizeof(saved_period);
	saved_period = -1;
	if (sysctlbyname("kern.hwpmc.mux_period_ms", &saved_period, &s,
	    &new_period, sizeof(new_period)) == 0)
		printf("set kern.hwpmc.mux_period_ms = %d (was %d)\n",
		    new_period, saved_period);
	else
		saved_period = -1;

	/* Arm the watchdog before we do anything that can wedge. */
	signal(SIGALRM, on_watchdog);
	alarm(WATCHDOG_SECS);

	/* Spin up the pinned RT workers and wait for them to be busy. */
	g_stop = 0;
	g_running = 0;
	for (i = 0; i < nworkers; i++)
		if (pthread_create(&tid[i], NULL, worker,
		    (void *)(intptr_t)(i + 1)) != 0)
			err(1, "pthread_create %d", i);
	while (__atomic_load_n(&g_running, __ATOMIC_SEQ_CST) < nworkers)
		usleep(1000);

	for (i = 0; i < MAX_GROUPS; i++)
		if (pmc_start(grps[i].ids[0]) < 0) {
			warn("pmc_start g%d leader", i);
			g_stop = 1;
			for (j = 0; j < nworkers; j++)
				pthread_join(tid[j], NULL);
			goto restore;
		}

	/*
	 * Phase A -- rotation drain stress.  With mux_period_ms=1 the
	 * rotation kthread evicts a whole group ~1000 times/second; each
	 * eviction drains the pinned RT workers that hold the counters.
	 * A passive-drain kernel dies inside this sleep.
	 */
	printf("stressing rotation drain for %d s ...\n", STRESS_SECS);
	sleep(STRESS_SECS);

	/*
	 * Phase B -- teardown drain stress.  Stop, then release every
	 * sibling while the RT workers are STILL spinning, so the counters
	 * are loaded on pinned cores when pmc_wait_for_pmc_idle runs.
	 */
	printf("tearing down while workers still spinning ...\n");
	for (i = 0; i < MAX_GROUPS; i++)
		(void)pmc_stop(grps[i].ids[0]);
	for (i = 0; i < MAX_GROUPS; i++) {
		for (j = 0; j < grps[i].nevents; j++)
			(void)pmc_release(grps[i].ids[j]);
		grps[i].nevents = 0;
	}

	/* Survived both drains -- now let the workers go. */
	g_stop = 1;
	for (i = 0; i < nworkers; i++)
		pthread_join(tid[i], NULL);

	alarm(0);
	if (saved_period >= 0)
		(void)sysctlbyname("kern.hwpmc.mux_period_ms", NULL, NULL,
		    &saved_period, sizeof(saved_period));

	printf("pmc_mux_drain_test: OK (rotation + teardown drains completed "
	    "against %d pinned RT targets)\n", nworkers);
	return (0);

restore:
	alarm(0);
	for (i = 0; i < MAX_GROUPS; i++)
		release_group(&grps[i]);
	if (saved_period >= 0)
		(void)sysctlbyname("kern.hwpmc.mux_period_ms", NULL, NULL,
		    &saved_period, sizeof(saved_period));
	return (1);
}
