make -j100 test_progs || exit
sudo sysctl -w kernel.softlockup_panic=1
sudo ./test_progs -v -a spin_lock_mt_timeout
