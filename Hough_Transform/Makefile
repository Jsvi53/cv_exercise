CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra
OPENCV_INCLUDE = -I D:/program/Opencv470/include
OPENCV_LIBDIR = -L D:/program/Opencv470/x64/mingw/lib
OPENCV_LIBS = -lopencv_world470

SRC = hough_transform.cpp
TARGET = hough_transform

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(OPENCV_INCLUDE) $(OPENCV_LIBDIR) $(SRC) -o $@ $(OPENCV_LIBS)

clean:
	rm -f $(TARGET)
