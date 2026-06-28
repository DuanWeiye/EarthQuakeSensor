CXX=g++
CXXFLAGS=-std=c++17 -O2 -Wall
jma: test_jma.cpp ../firmware/jma_intensity.cpp
	$(CXX) $(CXXFLAGS) -o build_jma.out $^ && ./build_jma.out
detector: test_detector.cpp ../firmware/detector.cpp ../firmware/jma_intensity.cpp ../firmware/jma_stream.cpp
	$(CXX) $(CXXFLAGS) -o build_det.out $^ && ./build_det.out
jma_stream: test_jma_stream.cpp ../firmware/jma_stream.cpp ../firmware/jma_intensity.cpp
	$(CXX) $(CXXFLAGS) -o build_jma_stream.out $^ && ./build_jma_stream.out
all: jma jma_stream detector
.PHONY: jma detector jma_stream all
