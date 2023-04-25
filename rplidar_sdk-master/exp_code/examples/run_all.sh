sudo chmod -R 666 /dev/ttyUSB1
sudo sysctl -w kernel.shmmax=2147483648
sudo sysctl -w net.core.wmem_max=2500000



sudo python3 rx_multi_threads.py
