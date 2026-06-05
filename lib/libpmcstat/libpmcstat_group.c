/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
 *
 * Brace-list event-group parser used by pmcstat -b.
 *
 * Grammar:
 *     spec    := group | event
 *     group   := '{' event ( ',' event )* '}'
 *     event   := <whatever pmcstat already accepts as a counter spec,
 *                 e.g. "instructions:k" or "ex_ret_instr,thread=0">
 *
 * Note: commas inside an event spec (e.g. unit masks) collide with
 * the sibling separator, so this v1 parser splits ONLY on top-level
 * commas - any '{' / ',' / '}' inside a quoted token is preserved.
 * Callers that need per-event attributes can put them after a ':'.
 */

#include <sys/types.h>
#include <sys/cpuset.h>

#include <ctype.h>
#include <err.h>
#include <errno.h>
#include <pmc.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "libpmcstat.h"

static char *
xstrndup(const char *s, size_t n)
{
	char *o = malloc(n + 1);

	if (o == NULL)
		return (NULL);
	memcpy(o, s, n);
	o[n] = '\0';
	return (o);
}

int
pmcstat_parse_event_group(const char *spec, char ***out_events, size_t *n_out)
{
	const char *p, *start, *end;
	char **events = NULL;
	size_t cap = 0, n = 0;

	if (spec == NULL || out_events == NULL || n_out == NULL) {
		errno = EINVAL;
		return (-1);
	}
	*out_events = NULL;
	*n_out = 0;

	while (*spec != '\0' && isspace((unsigned char)*spec))
		spec++;
	if (*spec != '{') {
		/* Not a group - caller handles single event. */
		return (1);
	}

	end = strrchr(spec, '}');
	if (end == NULL) {
		errno = EINVAL;
		return (-1);
	}
	p = spec + 1;
	while (p < end) {
		while (p < end && (isspace((unsigned char)*p) || *p == ','))
			p++;
		if (p >= end)
			break;
		start = p;
		while (p < end && *p != ',')
			p++;
		while (p > start && isspace((unsigned char)*(p - 1)))
			p--;
		if (p == start) {
			while (p < end && *p != ',')
				p++;
			continue;
		}
		if (n == cap) {
			size_t ncap = cap == 0 ? 4 : cap * 2;
			char **ne = realloc(events, ncap * sizeof(*events));
			if (ne == NULL)
				goto fail;
			events = ne;
			cap = ncap;
		}
		events[n] = xstrndup(start, p - start);
		if (events[n] == NULL)
			goto fail;
		n++;
		while (p < end && *p == ',')
			p++;
	}
	if (n < 2) {
		/* Single-element brace list -> not really a group. */
		pmcstat_free_event_group(events, n);
		return (1);
	}
	*out_events = events;
	*n_out = n;
	return (0);

fail:
	pmcstat_free_event_group(events, n);
	errno = ENOMEM;
	return (-1);
}

void
pmcstat_free_event_group(char **events, size_t n)
{
	size_t i;

	if (events == NULL)
		return;
	for (i = 0; i < n; i++)
		free(events[i]);
	free(events);
}

/*
 * Append one event entry to args->pa_events.  This factors out the
 * per-event setup so brace-list expansion can reuse it without
 * duplicating the inline body of pmcstat's getopt switch.  Only the
 * minimum subset of fields needed to drive pmc_allocate_group is
 * filled in here; the rest is set by pmcstat itself before the
 * allocation loop runs.
 */
void
pmcstat_add_one_event(int option, const char *spec, struct pmcstat_args *pa,
    int groupid, int is_leader)
{
	struct pmcstat_ev *ev;
	size_t c;

	ev = calloc(1, sizeof(*ev));
	if (ev == NULL)
		err(1, "calloc pmcstat_ev");

	switch (option) {
	case 'p': ev->ev_mode = PMC_MODE_TC; break;
	case 's': ev->ev_mode = PMC_MODE_SC; break;
	case 'P': ev->ev_mode = PMC_MODE_TS; break;
	case 'S': ev->ev_mode = PMC_MODE_SS; break;
	default:  ev->ev_mode = PMC_MODE_TC; break;
	}

	if (option == 'P' || option == 'p')
		pa->pa_flags |= FLAG_HAS_PROCESS_PMCS;
	if (option == 'P' || option == 'S')
		pa->pa_flags |= FLAG_HAS_SAMPLING_PMCS;
	if (option == 'p' || option == 's')
		pa->pa_flags |= FLAG_HAS_COUNTING_PMCS;
	if (option == 's' || option == 'S')
		pa->pa_flags |= FLAG_HAS_SYSTEM_PMCS;

	ev->ev_spec = strdup(spec);
	if (ev->ev_spec == NULL)
		err(1, "strdup ev_spec");
	c = strcspn(spec, ", \t");
	ev->ev_name = strndup(spec, c);
	if (ev->ev_name == NULL)
		err(1, "strndup ev_name");

	ev->ev_pmcid = PMC_ID_INVALID;
	ev->ev_cpu = (option == 'S' || option == 's') ? 0 : PMC_CPU_ANY;
	ev->ev_groupid = groupid;
	ev->ev_is_leader = is_leader;
	ev->ev_flags = 0;

	STAILQ_INSERT_TAIL(&pa->pa_events, ev, ev_next);
}
