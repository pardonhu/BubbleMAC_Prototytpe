sudo chmod -R 666 /dev/ttyUSB$1
sudo rm app/ultra_simple/main.cpp
sudo cp app/ultra_simple/main_origin.cpp app/ultra_simple/main.cpp
sudo make
sudo ldconfig
./output/Linux/Release/ultra_simple --channel --serial /dev/ttyUSB$1 115200
