# ov2slam_wasm

This branch is the **WebAssembly** version of [OV²SLAM](https://github.com/ov2slam/ov2slam). Check the original repo for detail.

And now it supports:
- mono/stereo odometry
- mono/stereo odometry with graph optimize

**No Loop-Closure**.
**Single-Threading**.
**Still on working**

#### notes


#### build
Make sure you have installed:
- OpenCV 4.x
- Eigen 3.4.0
- OpenGV
- Sophus
- Ceres-Solver

And build them with [emscripten](https://emscripten.org/).

And install them to the emsdk dir.

Then run the following commands:
```sh
# clone the repo
cd ov2slam_universal
git checkout origin/wasm

mkdir build
cd build

# activate emsdk
source /path/to/emsdk/emsdk_env.sh

emcmake cmake ..
emmake make -j4
```