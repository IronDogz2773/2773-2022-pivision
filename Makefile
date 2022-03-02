CXX=".\arm-raspbian10-linux-gnueabihf-g++.exe"
DEPS_CFLAGS=-Iinclude -Iinclude/opencv -Iinclude
DEPS_LIBS=-Llib -lwpilibc -lwpiHal -lcameraserver -lntcore -lcscore -lopencv_dnn -lopencv_highgui -lopencv_ml -lopencv_objdetect -lopencv_shape -lopencv_stitching -lopencv_superres -lopencv_videostab -lopencv_calib3d -lopencv_videoio -lopencv_imgcodecs -lopencv_features2d -lopencv_video -lopencv_photo -lopencv_imgproc -lopencv_flann -lopencv_core -lwpiutil -latomic
EXE=multiCameraServerExample
DESTDIR?=/home/pi/

.PHONY: clean build install

build: ${EXE}

install: build
	cp ${EXE} runCamera ${DESTDIR}

clean:
	rm ${EXE} *.o

OBJS=main.o

ballpipeline.o: ballpipeline.cpp
	${CXX} -c ballpipeline.cpp ${DEPS_CFLAGS}

retro.o: retro.cpp
	${CXX} -c retro.cpp ${DEPS_CFLAGS}

${EXE}: ${OBJS} ballpipeline.o retro.o
	${CXX} -pthread -g -o $@ $^ ${DEPS_LIBS} -Wl,--unresolved-symbols=ignore-in-shared-libs

.cpp.o:
	${CXX} -pthread -g -Og -c -o $@ -std=c++17 ${CXXFLAGS} ${DEPS_CFLAGS} $<
