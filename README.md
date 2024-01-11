# RigidSim
A rigid body simulator + renderer, project of Computer Animation class

## Build

Clone the repo
```
git clone --recursive git@github.com:Hoosus/RigidSim.git
```

Then build the object

```
mkdir build
cd build
cmake ..
```

You may need to install dependencies for LuisaCompute (including NVTT from NVIDIA). Just follow the instructions suggested by CMAKE.

## Run

```
./bin/app -b cuda -s 1600x1600 -t 0.05 -n 180 --fps 60 --dump "../1.mp4"
```

`-b` adjusts the backend, `-s` determines the resolution, `-t` adjusts the timestep, `-n` decides the total number of frames to be rendered.