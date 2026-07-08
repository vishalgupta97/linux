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
#include <fcntl.h>

#define MAX_ENTRIES_PER_BUCKET 64

#define SYSCTL_PATH "/proc/sys/net/core/bpf_spin_lock_timeout"

/* ------------------------------------------------------------------ */
/* sysctl + config detection helpers                                    */
/* ------------------------------------------------------------------ */

static int __read_sysctl(void)
{
	int fd, val = 0;
	char buf[32];

	fd = open(SYSCTL_PATH, O_RDONLY);
	if (fd < 0)
		return -1;
	if (read(fd, buf, sizeof(buf)) > 0)
		val = atoi(buf);
	close(fd);
	return val;
}

static int __write_sysctl(int val)
{
	int fd;
	char buf[32];

	fd = open(SYSCTL_PATH, O_WRONLY);
	if (fd < 0)
		return -1;
	snprintf(buf, sizeof(buf), "%d\n", val);
	if (write(fd, buf, strlen(buf)) < 0) {
		close(fd);
		return -1;
	}
	close(fd);
	return 0;
}


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

    int old_timeout = __read_sysctl();
    __write_sysctl(1000); //1s

    /* ── Load BPF skeleton ──────────────────────────────────── */
    struct rcuhashbash_bpf *skel = rcuhashbash_bpf__open_and_load();
    if (!skel) {
        fprintf(stderr, "Failed to load BPF skeleton: %s\n", strerror(errno));
        return 1;
    }

    int cfg_fd    = bpf_map__fd(skel->maps.num_bucket_config);
    int entries_fd = bpf_map__fd(skel->maps.entries);

    /* ── Write num_buckets into config ──────────────────────── */
    uint32_t cfg_key = 0;
    if (bpf_map_update_elem(cfg_fd, &cfg_key, &num_buckets, BPF_ANY)) {
        perror("bpf_map_update_elem(config)"); goto cleanup;
    }

    cfg_key = 1;
    if (bpf_map_update_elem(cfg_fd, &cfg_key, &entries_per_bucket, BPF_ANY)) {
        perror("bpf_map_update_elem(config)"); goto cleanup;
    }

    printf("Allocating %u buckets × %u entries = %u total entries...\n",
           num_buckets, entries_per_bucket,
           num_buckets * entries_per_bucket);

    /* ── Pre-populate bucket_count and entries maps ─────────── */
    for (uint32_t b = 0; b < num_buckets; b++) {
        for (uint32_t i = 0; i < entries_per_bucket; i++) {
	    uint32_t ekey = b * entries_per_bucket + i;
            uint64_t initial_value = (uint64_t)b * entries_per_bucket + i;

            if (bpf_map_update_elem(entries_fd, &ekey,
                                    &initial_value, BPF_ANY)) {
                perror("bpf_map_update_elem(entries)"); goto cleanup;
            }
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
    __write_sysctl(old_timeout);

cleanup:
    rcuhashbash_bpf__destroy(skel);
    return 0;
}
