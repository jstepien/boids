Boids
=====

See http://www.red3d.com/cwr/boids/ for some background.

Requirements
------------

For the GPU version you'll need

 * [CUDA](http://www.nvidia.com/object/cuda_home.html)
 * [CUDPP](http://code.google.com/p/cudpp/)

Both the CPU and the GPU versions require

 * [SDL](http://www.libsdl.org/)

The JavaScript version depends on

 * [Emscripten](http://emscripten.org/)
 * [LLVM and Clang](http://www.llvm.org/)

Building
--------

There are three main `make` targets:

  * `cpu`
  * `gpu`
  * `boids.js`

OpenMP is supported – if you're using GCC turn it on by adding `-fopenmp` to CFLAGS and LDFLAGS.

Usage
-----

The `-f` command line argument enables the fullscreen mode.

An attractor can be set using the left mouse button and removed using the right
mouse button.

Pressing any key kills the application.

License
-------

See `LICENSE`.
