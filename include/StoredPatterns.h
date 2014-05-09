#ifndef STORED_PATTERNS
#define STORED_PATTERNS

#include "opencv2/calib3d/calib3d.hpp"
#include <stdio.h>
#include <iostream>
#include <cmath>
#include "./findPose.h"
#include "./SquarePattern.h"
#include <map>

#define SquareSideLength 2.9
#define GapLength 0.75
#define gridSize 3
#define gridBorderOffset 2.2

using namespace cv;

struct tagPose {
  Mat r_vec;
  Mat t;
};

const SquarePattern storedPatterns[] = {0x1a2, 0x154, 0x1a4};//{0xa3, 0xc3, 0x145};
const SquarePattern INITIAL_PATTERN = storedPatterns[0];
const int numPatterns = 3;
std::map<SquarePattern, tagPose> patternPose;

void addPattern(SquarePatternHandle& handle, SquarePattern pattern, tagPose pose) {
  handle.add(pattern);
  patternPose[pattern] = pose;
}  
  
#endif
