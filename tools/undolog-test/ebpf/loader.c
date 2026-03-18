// loader.c
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <bpf/libbpf.h>
#include <bpf/bpf.h>
#include <errno.h>
#include <signal.h>

static volatile bool exiting = false;

void handle_signal(int sig) {
    exiting = true;
}

int main(int argc, char **argv)
{
    if(argc < 2)
        return 1;

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    int btf_id = atoi(argv[1]);

    struct bpf_object *obj;
    struct bpf_program *prog, *prog1;
    struct bpf_link *link, *link1;
    int btf_fd, err;

    /* Open the module BTF to resolve fentry target in kmod, not vmlinux */
    btf_fd = bpf_btf_get_fd_by_id(btf_id);
    if (btf_fd < 0) {
        fprintf(stderr, "Failed to get BTF fd for mymodule. "
                        "Is the module loaded and BTF embedded? %d error:%d %s\n", btf_id, btf_fd, strerror(errno));
        return 2;
    }

    obj = bpf_object__open_file("ebpf/fentry_mymodule.bpf.o", NULL);
    if (libbpf_get_error(obj)) {
        fprintf(stderr, "Failed to open BPF object\n");
        return 3;
    }

    prog = bpf_object__find_program_by_name(obj, "trace_init_ht");
    prog1 = bpf_object__find_program_by_name(obj, "trace_attach_cs_ht");
    if (!prog || !prog1) {
        fprintf(stderr, "Failed to find BPF program\n");
        return 4;
    }

    /* Point the program at the module's BTF */
    //bpf_program__set_attach_target(prog, btf_fd, "baseline");

    err = bpf_object__load(obj);
    if (err) {
        fprintf(stderr, "Failed to load BPF object: %d\n", err);
        return 5;
    }

    link = bpf_program__attach(prog);
    link1 = bpf_program__attach(prog1);
    if (libbpf_get_error(link) || libbpf_get_error(link1)) {
        fprintf(stderr, "Failed to attach BPF program\n");
        return 6;
    }

    printf("Attached! Reading trace output from /sys/kernel/debug/tracing/trace_pipe\n");
    printf("Press Ctrl+C to stop.\n");

    while (!exiting) { 
       sleep(1);
    }

    bpf_link__destroy(link);
    bpf_link__destroy(link1);
    bpf_object__close(obj);
    close(btf_fd);
    return 0;
}

