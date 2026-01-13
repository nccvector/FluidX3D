MAKEFLAGS = -j$(nproc)
CC = g++
CFLAGS = -std=c++17 -pthread -O -Wno-comment

.PHONY: no-target
no-target:
	@echo "\033[91mError\033[0m: Please select one of these targets: make Linux-X11, make Linux, make macOS, make Android"

Linux-X11 Linux macOS Android: LDFLAGS_OPENCL = -I./src/OpenCL/include

Linux-X11 Linux: LDLIBS_OPENCL = -L./src/OpenCL/lib -lOpenCL
macOS: LDLIBS_OPENCL = -framework OpenCL
Android: LDLIBS_OPENCL = -L/system/vendor/lib64 -lOpenCL

Linux-X11: LDFLAGS_X11 = -I./src/X11/include
Linux macOS Android: LDFLAGS_X11 =

Linux-X11: LDLIBS_X11 = -L./src/X11/lib -lX11 -lXrandr
Linux macOS Android: LDLIBS_X11 =

Linux-X11 Linux macOS Android: bin/FluidX3D

bin/FluidX3D: temp/graphics.o temp/info.o temp/kernel.o temp/lbm.o temp/lodepng.o temp/main.o temp/setup.o temp/shapes.o make.sh
	@mkdir -p bin
	$(CC) temp/*.o -o bin/FluidX3D $(CFLAGS) $(LDFLAGS_OPENCL) $(LDLIBS_OPENCL) $(LDFLAGS_X11) $(LDLIBS_X11)

temp/graphics.o: src/graphics.cpp src/defines.hpp src/graphics.hpp src/lodepng.hpp src/utilities.hpp make.sh
	@mkdir -p temp
	$(CC) -c src/graphics.cpp -o temp/graphics.o $(CFLAGS) $(LDFLAGS_X11)

temp/info.o: src/info.cpp src/defines.hpp src/graphics.hpp src/info.hpp src/lbm.hpp src/lodepng.hpp src/opencl.hpp src/units.hpp src/utilities.hpp make.sh
	@mkdir -p temp
	$(CC) -c src/info.cpp -o temp/info.o $(CFLAGS) $(LDFLAGS_OPENCL)

temp/kernel.o: src/kernel.cpp src/kernel.hpp src/lodepng.hpp src/utilities.hpp
	@mkdir -p temp
	$(CC) -c src/kernel.cpp -o temp/kernel.o $(CFLAGS)

temp/lbm.o: src/lbm.cpp src/defines.hpp src/graphics.hpp src/info.hpp src/lbm.hpp src/lodepng.hpp src/opencl.hpp src/units.hpp src/utilities.hpp make.sh
	@mkdir -p temp
	$(CC) -c src/lbm.cpp -o temp/lbm.o $(CFLAGS) $(LDFLAGS_OPENCL)

temp/lodepng.o: src/lodepng.cpp src/lodepng.hpp
	@mkdir -p temp
	$(CC) -c src/lodepng.cpp -o temp/lodepng.o $(CFLAGS)

temp/main.o: src/main.cpp src/defines.hpp src/graphics.hpp src/info.hpp src/lbm.hpp src/lodepng.hpp src/opencl.hpp src/setup.hpp src/shapes.hpp src/units.hpp src/utilities.hpp make.sh
	@mkdir -p temp
	$(CC) -c src/main.cpp -o temp/main.o $(CFLAGS) $(LDFLAGS_OPENCL)

temp/setup.o: src/setup.cpp src/defines.hpp src/graphics.hpp src/info.hpp src/lbm.hpp src/lodepng.hpp src/opencl.hpp src/setup.hpp src/shapes.hpp src/units.hpp src/utilities.hpp make.sh
	@mkdir -p temp
	$(CC) -c src/setup.cpp -o temp/setup.o $(CFLAGS) $(LDFLAGS_OPENCL)

temp/shapes.o: src/shapes.cpp src/shapes.hpp src/utilities.hpp make.sh
	@mkdir -p temp
	$(CC) -c src/shapes.cpp -o temp/shapes.o $(CFLAGS) $(LDFLAGS_OPENCL)

# ============================================================================
# Shared Library Targets (C API)
# ============================================================================

# Library build flag - excludes main() and provides stubs
LIB_FLAGS = -DFX3D_LIBRARY

# Object file for C API wrapper
temp/fluidx3d_c_api.o: src/fluidx3d_c_api.cpp src/fluidx3d.h src/defines.hpp src/lbm.hpp src/shapes.hpp src/units.hpp src/utilities.hpp make.sh
	@mkdir -p temp
	$(CC) -c src/fluidx3d_c_api.cpp -o temp/fluidx3d_c_api.o $(CFLAGS) -fPIC $(LIB_FLAGS) $(LDFLAGS_OPENCL)

# Shared library objects (with -fPIC)
temp/graphics_pic.o: src/graphics.cpp src/defines.hpp src/graphics.hpp src/lodepng.hpp src/utilities.hpp make.sh
	@mkdir -p temp
	$(CC) -c src/graphics.cpp -o temp/graphics_pic.o $(CFLAGS) -fPIC $(LIB_FLAGS) $(LDFLAGS_X11)

temp/info_pic.o: src/info.cpp src/defines.hpp src/graphics.hpp src/info.hpp src/lbm.hpp src/lodepng.hpp src/opencl.hpp src/units.hpp src/utilities.hpp make.sh
	@mkdir -p temp
	$(CC) -c src/info.cpp -o temp/info_pic.o $(CFLAGS) -fPIC $(LIB_FLAGS) $(LDFLAGS_OPENCL)

temp/kernel_pic.o: src/kernel.cpp src/kernel.hpp src/lodepng.hpp src/utilities.hpp
	@mkdir -p temp
	$(CC) -c src/kernel.cpp -o temp/kernel_pic.o $(CFLAGS) -fPIC $(LIB_FLAGS)

temp/lbm_pic.o: src/lbm.cpp src/defines.hpp src/graphics.hpp src/info.hpp src/lbm.hpp src/lodepng.hpp src/opencl.hpp src/units.hpp src/utilities.hpp make.sh
	@mkdir -p temp
	$(CC) -c src/lbm.cpp -o temp/lbm_pic.o $(CFLAGS) -fPIC $(LIB_FLAGS) $(LDFLAGS_OPENCL)

temp/lodepng_pic.o: src/lodepng.cpp src/lodepng.hpp
	@mkdir -p temp
	$(CC) -c src/lodepng.cpp -o temp/lodepng_pic.o $(CFLAGS) -fPIC $(LIB_FLAGS)

temp/shapes_pic.o: src/shapes.cpp src/shapes.hpp src/utilities.hpp make.sh
	@mkdir -p temp
	$(CC) -c src/shapes.cpp -o temp/shapes_pic.o $(CFLAGS) -fPIC $(LIB_FLAGS) $(LDFLAGS_OPENCL)

# Library object files list
LIB_OBJS = temp/fluidx3d_c_api.o temp/graphics_pic.o temp/info_pic.o temp/kernel_pic.o temp/lbm_pic.o temp/lodepng_pic.o temp/shapes_pic.o

# Platform-specific shared library targets
.PHONY: lib-Linux-X11 lib-Linux lib-macOS

lib-Linux-X11: LDFLAGS_OPENCL = -I./src/OpenCL/include
lib-Linux-X11: LDLIBS_OPENCL = -L./src/OpenCL/lib -lOpenCL
lib-Linux-X11: LDFLAGS_X11 = -I./src/X11/include
lib-Linux-X11: LDLIBS_X11 = -L./src/X11/lib -lX11 -lXrandr
lib-Linux-X11: lib/libfluidx3d.so

lib-Linux: LDFLAGS_OPENCL = -I./src/OpenCL/include
lib-Linux: LDLIBS_OPENCL = -L./src/OpenCL/lib -lOpenCL
lib-Linux: LDFLAGS_X11 =
lib-Linux: LDLIBS_X11 =
lib-Linux: lib/libfluidx3d.so

lib-macOS: LDFLAGS_OPENCL = -I./src/OpenCL/include
lib-macOS: LDLIBS_OPENCL = -framework OpenCL
lib-macOS: LDFLAGS_X11 =
lib-macOS: LDLIBS_X11 =
lib-macOS: lib/libfluidx3d.dylib

# Linux shared library
lib/libfluidx3d.so: $(LIB_OBJS)
	@mkdir -p lib
	$(CC) -shared -o lib/libfluidx3d.so $(LIB_OBJS) $(CFLAGS) $(LDFLAGS_OPENCL) $(LDLIBS_OPENCL) $(LDFLAGS_X11) $(LDLIBS_X11)
	@cp src/fluidx3d.h lib/
	@echo "\033[92mLibrary built:\033[0m lib/libfluidx3d.so"
	@echo "\033[92mHeader copied:\033[0m lib/fluidx3d.h"

# macOS shared library
lib/libfluidx3d.dylib: $(LIB_OBJS)
	@mkdir -p lib
	$(CC) -dynamiclib -o lib/libfluidx3d.dylib $(LIB_OBJS) $(CFLAGS) $(LDFLAGS_OPENCL) $(LDLIBS_OPENCL) $(LDFLAGS_X11) $(LDLIBS_X11)
	@cp src/fluidx3d.h lib/
	@echo "\033[92mLibrary built:\033[0m lib/libfluidx3d.dylib"
	@echo "\033[92mHeader copied:\033[0m lib/fluidx3d.h"

# Convenience target: auto-detect platform and build library
.PHONY: lib
lib:
	@echo "\033[91mError\033[0m: Please select one of these targets: make lib-Linux-X11, make lib-Linux, make lib-macOS"

# ============================================================================
# Test/Demo Targets (C API)
# ============================================================================

# SDL2 paths (macOS Homebrew)
SDL2_CFLAGS = -I/opt/homebrew/opt/sdl2/include
SDL2_LIBS = -L/opt/homebrew/opt/sdl2/lib -lSDL2

# Demo executables
DEMO_CFLAGS = -I./lib -L./lib -lfluidx3d -framework OpenCL -lc++ -Wl,-rpath,./lib

.PHONY: demos demo-test demo-vortex demo-gpu demo-viewer run-test run-vortex run-gpu run-viewer

# Build all demos
demos: test/test_c_api test/demo_vortex test/demo_gpu_graphics test/demo_viewer
	@echo "\033[92mAll demos built!\033[0m"

# Individual demo builds (requires lib to be built first with: make lib-macOS)
test/test_c_api: test/test_c_api.c lib/fluidx3d.h
	@test -f lib/libfluidx3d.dylib || (echo "\033[91mError:\033[0m Run 'make lib-macOS' first" && exit 1)
	clang -o $@ $< $(DEMO_CFLAGS)

test/demo_vortex: test/demo_vortex.c lib/fluidx3d.h
	@test -f lib/libfluidx3d.dylib || (echo "\033[91mError:\033[0m Run 'make lib-macOS' first" && exit 1)
	clang -o $@ $< $(DEMO_CFLAGS)

test/demo_gpu_graphics: test/demo_gpu_graphics.c lib/fluidx3d.h
	@test -f lib/libfluidx3d.dylib || (echo "\033[91mError:\033[0m Run 'make lib-macOS' first" && exit 1)
	clang -o $@ $< $(DEMO_CFLAGS)

test/demo_viewer: test/demo_viewer.c lib/fluidx3d.h
	@test -f lib/libfluidx3d.dylib || (echo "\033[91mError:\033[0m Run 'make lib-macOS' first" && exit 1)
	clang -o $@ $< $(DEMO_CFLAGS) $(SDL2_CFLAGS) $(SDL2_LIBS)

# Run targets
run-test: test/test_c_api
	DYLD_LIBRARY_PATH=./lib ./test/test_c_api

run-vortex: test/demo_vortex
	DYLD_LIBRARY_PATH=./lib ./test/demo_vortex

run-gpu: test/demo_gpu_graphics
	@mkdir -p output
	DYLD_LIBRARY_PATH=./lib ./test/demo_gpu_graphics

run-viewer: test/demo_viewer
	DYLD_LIBRARY_PATH=./lib ./test/demo_viewer

# Convenience aliases
demo-test: run-test
demo-vortex: run-vortex
demo-gpu: run-gpu
demo-viewer: run-viewer

# ============================================================================
# Clean
# ============================================================================

.PHONY: clean clean-demos
clean:
	@rm -rf temp bin/FluidX3D lib/ output/

clean-demos:
	@rm -f test/test_c_api test/demo_vortex test/demo_gpu_graphics test/demo_viewer
