
export cores=(1 2 4 8 14 28 42 56 112) #1 2 4 8 16 32 64 80 96 112 128) # 2 4 8 12 16 20 28 56 84 112 128 168 224)
#export mutex_cores=(1 2 4 8 12 16 20 28 56 84 112 128 168 224 336 448 576 672 800 896)
export ncores=112
export root_dir=`pwd`
export results_dir=${root_dir}/doc/results/srv1vm
export runtime=10
export kernel=`uname -r | sed 's/5.14.16-//'`

## For python-environment
export python_env_cores='[56]' #'[1,2,4,8,12,16,20,28,56,84,112,128,168,224]'
