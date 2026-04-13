
export cores=(8) #2 4 8 14 28 42 56 70 84 98 112)
export ncores=112
export root_dir=`pwd`
export results_dir=${root_dir}/doc/results/srv1vm
export runtime=30
export kernel=`uname -r | sed 's/5.14.16-//'`

#export cores=(1 2 4 8 12 24 36 48 60 72 84 96)
#export ncores=96
#export root_dir=`pwd`
#export results_dir=${root_dir}/doc/results/srv10vm
#export runtime=10
#export kernel=`uname -r | sed 's/5.14.16-//'`
#
