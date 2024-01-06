# RigidSim
A rigid body simulator + renderer, project of Computer Animation class

## Build

Clone the repo
```
git clone --recursive git@github.com:Hoosus/RigidSim.git
```

Download Optix 7.6 (7.7, 8.0 or higher not supported due to LuisaCompute), and move all the header files in place.
See `src/compute/src/backends/cuda/optix/README.md`

```
mkdir build
cd build
cmake -DCMAKE_USE_OPENSSL=ON .. -D OptiX_DIR=~/NVIDIA-OptiX-SDK-7.6.0-linux64-x86_64
```

## Run

```
./bin/app -b cuda -s 1600x1200 -t 1 --dump "./1.jpg" 
```