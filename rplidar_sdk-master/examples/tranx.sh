sudo sysctl -w kernel.shmmax=2147483648
sudo sysctl -w net.core.wmem_max=2500000
sudo sysctl -w net.core.rmem_max=50000000
sudo python3 -u wifi_tx.py