CXX=g++
CXXFLAGS=-std=c++17 -O2 -Wall
jma: test_jma.cpp ../firmware/jma_intensity.cpp
	$(CXX) $(CXXFLAGS) -o build_jma.out $^ && ./build_jma.out
detector: test_detector.cpp ../firmware/detector.cpp ../firmware/jma_intensity.cpp ../firmware/jma_stream.cpp
	$(CXX) $(CXXFLAGS) -o build_det.out $^ && ./build_det.out
jma_stream: test_jma_stream.cpp ../firmware/jma_stream.cpp ../firmware/jma_intensity.cpp
	$(CXX) $(CXXFLAGS) -o build_jma_stream.out $^ && ./build_jma_stream.out
wavering: test_wavering.cpp ../firmware/wavering.cpp
	$(CXX) $(CXXFLAGS) -o build_wavering.out $^ && ./build_wavering.out
wavelog: test_wavelog.cpp ../firmware/wavelog.cpp
	$(CXX) $(CXXFLAGS) -o build_wavelog.out $^ && ./build_wavelog.out
all: jma jma_stream detector wavering wavelog
.PHONY: jma detector jma_stream wavering wavelog all
