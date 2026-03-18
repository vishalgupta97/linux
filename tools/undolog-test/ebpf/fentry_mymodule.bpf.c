// fentry_mymodule.bpf.c
#include "vmlinux.h"
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_tracing.h>

char LICENSE[] SEC("license") = "GPL";

SEC("fentry/init_ht")
int BPF_PROG(trace_init_ht)
{
    __u64 pid_tgid = bpf_get_current_pid_tgid();
    __u32 pid  = (__u32)(pid_tgid & 0xFFFFFFFF);
    __u32 tgid = (__u32)(pid_tgid >> 32);
    __u32 cpu  = bpf_get_smp_processor_id();

    bpf_printk("fentry init_ht: pid=%u tgid=%u cpu=%u\n",
               pid, tgid, cpu);
    return 0;
}

SEC("fentry/attach_cs_ht")
int BPF_PROG(trace_attach_cs_ht, int src_value, int dst_value, void *stats)
{
    __u64 pid_tgid = bpf_get_current_pid_tgid();
    __u32 pid  = (__u32)(pid_tgid & 0xFFFFFFFF);
    __u32 tgid = (__u32)(pid_tgid >> 32);
    __u32 cpu  = bpf_get_smp_processor_id();

    bpf_printk("fentry init_ht: pid=%u tgid=%u cpu=%u src=%d dst=%d\n",
               pid, tgid, cpu, src_value, dst_value);
    return 0;
}

