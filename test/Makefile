CXX=g++
CXXFLAGS=-std=c++17 -O2 -Wall
jma: test_jma.cpp ../firmware/jma_intensity.cpp
	$(CXX) $(CXXFLAGS) -o build_jma.out $^ && ./build_jma.out
detector: test_detector.cpp ../firmware/detector.cpp ../firmware/jma_intensity.cpp
	$(CXX) $(CXXFLAGS) -o build_det.out $^ && ./build_det.out
.PHONY: jma detector
