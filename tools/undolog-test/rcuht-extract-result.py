#!/usr/bin/python3

import sys
import os
import os.path
import re
import optparse

lock = "spinlock"
lock_type = "spinlock-bucket" #spinlock"
server_name = "srv9"

if lock_type == "spinlock":
    bench = ["table_spinlock","table_aqs","table_cna","table_komb","table_ffwd","table_komb_delegation"]
elif lock_type == "spinlock-bucket":
    bench = ["spinlock", "aqs", "cna", "komb", "ffwd", "komb_delegation"]
else:
   raise Exception("Unknown lock_type") 

runs = [1]
rw_writes = [100] #0,1,20,50,100]
time = 30
buckets = 4 #1024
entries = 16 #4096
parent_dir="../"

if server_name == "srv10":
    cores = 96
    core = [1, 2, 4, 8, 16, 32, 47, 60, 72, 84, 94]
elif server_name == "srv9":
    cores = 96
    core = [1, 2, 4, 8, 16, 32, 47, 62, 78, 86, 94]
elif server_name == "srv1":
    cores = 224
    core = [1, 2, 4, 8, 14, 27, 54, 81, 108, 135, 162, 189, 216]
elif server_name == "srv5":
    cores = 56
    core = [1, 2, 4, 8, 16, 27, 41, 55]
elif server_name == "srv8":
    cores = 128
    core = [1, 2, 4, 8, 16, 32, 47, 53, 63, 78, 94, 110, 126]


result_folder=parent_dir+server_name+"/results-"+lock+"-"+str(cores)+"cores-"+str(time)+"seconds/"+str(buckets)+"buckets-"+str(entries)+"entries/"

if __name__ == "__main__":
	for writes in rw_writes:
		out_file_name= result_folder+lock_type+"-"+str(writes)+"-percent-writes.csv"
		out_file= open(out_file_name,"w")
		index=0
		plot_file.write("plot ")
		for b in bench:
			out_file.write("# "+ b+"\n")
			out_file.write("# Cores, Throughput (Jobs/us)\n")
			index = index + 1
			for c in core:
				total_ops = 0
				total_runs = 0
				for r in runs:
					path = result_folder +"/"+b + "/"+str(writes)+"percent_writes/core." + str(c)
					if os.path.isfile(path):
						with open(path, "r") as f:
							for line in f:
								if "summary: total:" not in line:
									continue
								line = line.split()
								ops = float(line[3])*1000/float(line[5])
								total_ops += ops
								total_runs += 1
				if total_runs != 0:
					out_file.write(str(c)+","+str(total_ops/total_runs)+"\n")
				else:
					out_file.write(str(c)+",0\n")
			out_file.write("\n\n")
