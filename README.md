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
- Source codes of demonstrated version of ceres and CGAL are provided in the folder named 3rd_packages.