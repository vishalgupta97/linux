import errno
import sys
import os
import subprocess
from subprocess import check_output

cpuinfo = [dict(map(str.strip, line.split(":", 1))
                for line in block.splitlines())
           for block in open("/proc/cpuinfo", "r").read().split("\n\n")
           if len(block.strip())]
#lscpu_res = subprocess.Popen('lscpu', shell=True, stdout=subprocess.PIPE).stdout.readlines() 
lscpu_res = check_output(['lscpu']).decode("utf-8").split("\n")
#print(lscpu_res)
for line in lscpu_res:
    if 'Socket(s):' in line:
        (__tmp__, sockets) = line.split(':')
        num_sockets = int(sockets)
    elif 'Core(s) per socket:' in line:
        (__tmp__, cores_per_socket) = line.split(':')
        num_cores_per_socket = int(cores_per_socket)

# Keep only primary hyperthreads
primaries = set()
for cpu in cpuinfo:
    processor = cpu["processor"]
    try:
        s = open("/sys/devices/system/cpu/cpu%s/topology/thread_siblings_list" % processor).read()
    except EnvironmentError as e:
        if e.errno == errno.ENOENT:
            primaries.add(processor)
            continue
        raise
    ss = ''
    try:
        ss = set(map(int, s.split("-")))
    except:
        ss = set(map(int, s.split(",")))
    if int(processor) == min(ss):
        primaries.add(processor)
cpuinfo = [cpu for cpu in cpuinfo if cpu["processor"] in primaries]

def seq(cpuinfo):
    packages = {}
    package_ids = set()
    for cpu in cpuinfo:
        if "physical id" in cpu:
            package_id = int(cpu["physical id"])
            packages.setdefault(package_id, []).append(cpu)
            if cpu["processor"] == "0":
                cpu0_package_id = int(package_id)
            package_ids.add(int(package_id))
        else:
            yield cpu
    for cpu in packages[cpu0_package_id]:
        yield cpu
    package_ids.remove(cpu0_package_id)

    for package_id in package_ids:
        for cpu in packages[package_id]:
            yield cpu

if __name__ == "__main__":
    cpu_count = len(cpuinfo)
    with open("include/topology.h", "w") as f:
        f.write("#define online_cpus (%s)\n" % cpu_count)
        f.write("#define online_sockets (%s)\n" % num_sockets)
        f.write("#define num_cores_per_socket (%s)\n" % num_cores_per_socket)
    with open("include/cpuseq.h", "w") as f:
        f.write("int cpuseq[] = { %s" % ",".join(cpu["processor"] for cpu in seq(cpuinfo)) +" };\n")
