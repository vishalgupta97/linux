#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define DEVICE_PATH "/dev/bpf_rcuht_sync"
#define BPF_READY _IO('B', 1)

int main() {
    int fd = open(DEVICE_PATH, O_RDWR);
    if (fd < 0) {
        perror("open");
        return 1;
    }
    
    if (ioctl(fd, BPF_READY) < 0) {
        perror("ioctl BPF_READY");
        close(fd);
        return 1;
    }
    
    printf("BPF ready signal sent! Module kthread unblocked.\n");
    close(fd);
    return 0;
}

