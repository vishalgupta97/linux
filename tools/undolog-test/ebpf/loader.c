// SPDX-License-Identifier: GPL-2.0
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <bpf/libbpf.h>
#include <bpf/bpf.h>
#include "rcuhashbash.skel.h"

#define MAX_ENTRIES_PER_BUCKET 64

static volatile int keep_running = 1;
static void sig_handler(int sig) { keep_running = 0; }

static void usage(const char *prog)
{
    fprintf(stderr,
            "Usage: %s --buckets <N> --entries-per-bucket <M>\n"
            "  --buckets              number of hash buckets (max 1024)\n"
            "  --entries-per-bucket   entries per bucket     (max %d)\n",
            prog, MAX_ENTRIES_PER_BUCKET);
}

int main(int argc, char **argv)
{
    uint32_t num_buckets = 0, entries_per_bucket = 0;
    uint32_t total_entries = 0;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--buckets") && i + 1 < argc)
            num_buckets = (uint32_t)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--entries-per-bucket") && i + 1 < argc)
            entries_per_bucket = (uint32_t)atoi(argv[++i]);
        else { usage(argv[0]); return 1; }
    }

    if (!num_buckets || !entries_per_bucket) { usage(argv[0]); return 1; }
    if (num_buckets > 1024) {
        fprintf(stderr, "buckets must be <= 1024\n"); return 1;
    }
    if (entries_per_bucket > MAX_ENTRIES_PER_BUCKET) {
        fprintf(stderr, "entries-per-bucket must be <= %d\n",
                MAX_ENTRIES_PER_BUCKET); return 1;
    }

    total_entries = num_buckets * entries_per_bucket;

    /* ── Load BPF skeleton ──────────────────────────────────── */
    struct rcuhashbash_bpf *skel = rcuhashbash_bpf__open_and_load();
    if (!skel) {
        fprintf(stderr, "Failed to load BPF skeleton: %s\n", strerror(errno));
        return 1;
    }

    skel->bss->init_total_entries = total_entries;

    printf("Allocating %u buckets × %u entries = %u total entries...\n",
           num_buckets, entries_per_bucket,
           total_entries);

    /* ── Pre-initialize arena-backed entries via syscall prog ─ */
    {
        int prog_fd = bpf_program__fd(skel->progs.init_entries_arena);
        LIBBPF_OPTS(bpf_test_run_opts, opts);

        if (prog_fd < 0) {
            fprintf(stderr, "Failed to get init_entries_arena fd: %d\n", prog_fd);
            goto cleanup;
        }

        if (bpf_prog_test_run_opts(prog_fd, &opts)) {
            perror("bpf_prog_test_run_opts(init_entries_arena)");
            goto cleanup;
        }
    }

    /* ── Attach ─────────────────────────────────────────────── */
    printf("Attaching to bpf_attachement_point...\n");
    if (rcuhashbash_bpf__attach(skel)) {
        fprintf(stderr, "Failed to attach: %s\n", strerror(errno));
        goto cleanup;
    }

    printf("Attached. Send SIGINT to detach.\n");
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);
    while (keep_running) sleep(1);
    printf("\nDetaching...\n");

cleanup:
    rcuhashbash_bpf__destroy(skel);
    return 0;
}
