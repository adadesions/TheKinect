# TheKinect

## Dependencies Installation
```sh
cd TheKinect/
git clone https://github.com/OpenKinect/libfreenect2
git clone https://github.com/opencv/opencv
git clone https://github.com/OpenNI/OpenNI
git clone https://github.com/avin2/SensorKinect
```
## CMake Build
```sh
cd <dep_dir>
mkdir build && cd build
cmake ..
make
```

only for OpenCV
```sh
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j8
```

## Usage
```sh
cd TheKinect/
mkdir build && cd build
cmake ..
make
```
To run a program
```sh
./TheKinect
```