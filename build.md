## step1 dependencies
sudo apt-get install \
    git \
    cmake \
    build-essential \
    libboost-program-options-dev \
    libboost-filesystem-dev \
    libboost-graph-dev \
    libboost-regex-dev \
    libboost-system-dev \
    libboost-test-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    libfreeimage-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libglew-dev \
    qtbase5-dev \
    libqt5opengl5-dev \
    libcgal-dev

sudo apt-get install libcgal-qt5-dev

## step2 Ceres Solver
sudo apt-get install libatlas-base-dev libsuitesparse-dev
scp -r ceres-solver xxx@xxx:xxx
cd ceres-solver
git checkout $(git describe --tags) # Checkout the latest release
mkdir build
cd build
cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
make -j
sudo make install

## step3 colmap
scp -r colmap xxx@xxx:xxx
cd colmap
git checkout dev
mkdir build
***conda user should do somthing here! check "for conda user"
CC=/usr/bin/gcc-7 CXX=/usr/bin/g++-7 cmake ..
make -j16
sudo apt-key adv --keyserver http://dk.archive.ubuntu.com/ubuntu --recv-keys 3B4FE6ACC0B21F32 40976EAF437D05B5
sudo make install

# for conda user:
1. .bashrc comment conda related:
    #export PATH="/home/cocoa_kang/anaconda3/bin:$PATH"
    #. /home/cocoa_kang/anaconda3/etc/profile.d/conda.sh
    #conda activate torch_cocoa
2. log out and log in
3. ldd /usr/lib/gcc/x86_64-linux-gnu/7/../../../x86_64-linux-gnu/libfreeimage.so
find this line libtiff.so.5 => /usr/lib/x86_64-linux-gnu/libtiff.so.5 (0x00007fe622da8000)
4. locate libtiff.so
5. sudo ln -sf /snap/gnome-3-28-1804/145/usr/lib/x86_64-linux-gnu/libtiff.so.5.3.0 /usr/lib/x86_64-linux-gnu/libtiff.so.5
6. uncomment .bashrc after installation is completed

/usr/local/lib/libtiff.so.5

version requirements: 
gcc/g++ 5
cgal 4.7
