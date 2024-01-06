# RigidSim
A rigid body simulator + renderer, project of Computer Animation class

## Build

```
cmake -DCMAKE_USE_OPENSSL=ON .. -D OptiX_DIR=~/NVIDIA-OptiX-SDK-7.6.0-linux64-x86_64
```

## Run

```
./bin/app -b cuda -s 1600x1200 -t 1 --dump "./1.jpg" 
```