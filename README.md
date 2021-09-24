This is a modified version of COLMAP 3.5 to take in multi-channel input data.

It is for the paper [Learning Efficient Photometric Feature Transform for Multi-view Stereo](https://arxiv.org/abs/2103.14794)

The repo is a night release version, still under refinement for final release.

----
## Requirements
The combination below is demonstrated. Other versions may cause compiling problems.  
| libs | version |
| -----| ------- |
| cuda | 10.0.130 |
| cudnn | 7.6.3.30 for cuda10|
| gcc/g++  | 5 |
| cgal | 4.7 |  
| boost | 1.58 |  
| ceres |1.14.0 | 

TIPS:  
- For Ubuntu 20+, no entry of gcc/g++ lower than 7 are provided. One should add xenial repository as described [here](https://askubuntu.com/questions/1235819/ubuntu-20-04-gcc-version-lower-than-gcc-7).
- Source codes of demonstrated version of ceres and CGAL are provided in the folder named 3rd_packages. If you use these codes, the recommended order of installing is: gcc/g++ -> cuda/cudnn -> boost -> ceres -> cgal

----
## Installation
### Ubuntu
1. Install required libs by:
    ```
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
    sudo apt-get install libatlas-base-dev libsuitesparse-dev
    ```
    Note: By automatic installation, the version of boost may not meet the version required.
2. Install Ceres Solver  
    First unpack the source code of Ceres in 3rd_packages, then:
    ```
    cd ceres-solver
    git checkout $(git describe --tags) # Checkout the latest release
    mkdir build
    cd build
    cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
    make -j
    sudo make install
    ```
3. Compile COLMAP
    ```
    mkdir build
    cd build
    CC=/usr/bin/gcc-5 CXX=/usr/bin/g++-5 cmake ..
    make -j16
    sudo make install
    ```
    Note: If you have Anaconda installed, the compiler may complain the missing library of libtiff.so. That is because Anaconda changes the system PATH and hides the required libtiff.so.5. You need to find this library file and manually link that file to /usr/lib/x86_64-linux-gnu/libtiff.so.5

### Windows
TODO: finish the compilation tutorial

----
## Usage
Since we only modified the channel of images for patch match, only the *Undistort Image* & *Patch Match Stereo* parts are different. The other parts are same with original COLMAP.
### Channel of input images
The channel of input images is specified in line 50 of src/mvs/patch_match_cuda.cu.
If you want to use a different channel number, recompilation is required.
### Undistort
    ```
    colmap image_undistorter --image_path $IMAGE_PATH --input_path $INPUT_PATH 
    ```
$IMAGE_PATH contains the multi-channel images. The name of multi-channel images should be same as the images used in SFM and an extra .bin is appended. For example, if the image names used in SFM are 0.png, 1.png,... 10.png. Then there should be 0.png.bin, 1.png.bin,... 10.png.bin in $IMAGE_PATH. The format of each bin file is:  
3 int: image_width, image_height, image_channel  
(iamge_width*image_height*image_channel) floats (row major)  

$INPUT_PATH contains the dense folder exported after SFM, by use the order in COLMAP GUI: file -> export as txt. The folder should contains 3 txt files: cameras.txt, images.txt and points.txt
### Patch Match Stereo
    The order 
