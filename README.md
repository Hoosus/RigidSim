# RigidSim
A rigid body simulator + renderer, project of Computer Animation class

## Build

Clone the repo
```
git clone --recursive git@github.com:Hoosus/RigidSim.git
```

~~Download Optix 7.6, and move all the header files in place.~~
~~See `src/compute/src/backends/cuda/optix/README.md`~~

```
mkdir build
cd build
cmake -DCMAKE_USE_OPENSSL=ON .. -D OptiX_DIR=/home/xuejun/NVIDIA-OptiX-SDK-7.6.0-linux64-x86_64 -D NVTT_DIR=/home/xuejun/nvtt
```

## Run

```
./bin/app -b cuda -s 1600x1200 -t 0.1 -n 10 --fps 1 --dump "../1.mp4" 
```