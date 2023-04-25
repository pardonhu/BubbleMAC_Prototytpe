cd workarea
git clone --recursive https://github.com/gnuradio/gnuradio.git
cd gnuradio
git clone https://github.com/gnuradio/volk.git
# 选择版本3.8
git checkout maint-3.8
cd volk
git checkout v2.2.1
cd ..
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 ../
make -j4 # make -j4 will use 4 threads in the build
make test
sudo make install
sudo ldconfig
