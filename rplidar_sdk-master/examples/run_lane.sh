# sudo chmod -R 666 /dev/ttyUSB$1
sudo sysctl -w kernel.shmmax=2147483648
sudo sysctl -w net.core.wmem_max=2500000
sudo sysctl -w net.core.rmem_max=50000000

sudo gnome-terminal --tab -e 'bash -c "sudo python3 multi_threads/GPS.py; exec bash;"'
python3 wifi_lane_front.py


# sudo wireshark -k -i /home/hu/Desktop/memory_file_sys/wifi_fifo.pcap;

