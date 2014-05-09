#ifndef SQUARE_PATTERN
#define SQUARE_PATTERN

/**************************************
 * SquarePattern.h
 * Matthew Bailey
 *
 * This file describes a lightweight
 * pattern for use in tag detection,
 * matching, and tracking.
 *
 * A SquarePattern is defined as
 * an NxN grid of squares, addressed
 * from bottom to top, left to right.
 *
 *  i\j  0    1    2
 *  2  [   ][   ][   ]
 *  1  [   ][   ][   ]
 *  0  [   ][   ][   ]
 *
 ***************************************/

#include <cmath>
#include <iostream>
#include <iomanip>
#include <map>

// SquarePattern are stored internally as integers.
typedef unsigned int SquarePattern;
const SquarePattern NULL_PATTERN = 0x00;

struct rotation {
  SquarePattern pattern;
  unsigned int angle;
};

class SquarePatternHandle {
public:
    SquarePatternHandle(int n = 3);
    ~SquarePatternHandle();

    rotation nullRotation;
    inline bool get(SquarePattern, int, int);
    inline bool access(SquarePattern, int, int);
    inline void set(SquarePattern&, int, int);
    inline void set(SquarePattern&, int, int, bool);
    inline void clear(SquarePattern&, int, int);
    rotation findMatchingPattern(SquarePattern);
    inline SquarePattern rotate(SquarePattern, int);
    bool areEquivalent(SquarePattern, SquarePattern);
    SquarePattern createPattern(bool**);
    void add(SquarePattern);
    void print(SquarePattern, std::ostream&);
    void print(SquarePattern);

private:
    int N;
    const static int minIndex = 0;
    int maxIndex;
    int maxBitIndex;

    std::map<SquarePattern, rotation> list;

    inline int shift(int, int);
    inline void build(SquarePattern&, SquarePattern, int);
};

SquarePatternHandle::SquarePatternHandle(int n) {
    N = n;
    maxIndex = n - 1;
    maxBitIndex = n*n - 1;
    nullRotation.pattern = NULL_PATTERN;
    nullRotation.angle   = 0;

    list[NULL_PATTERN] = nullRotation;
}

SquarePatternHandle::~SquarePatternHandle() { }


/* bit shift for SquarePatterns interpreted
 * with the following bit-pattern:
 * ___ ___ ___ ___ ___ ___ ___ ___ ___
 * 0,0|0,1|0,2|1,0|1,1|1,2|2,0|2,1|2,2
 *
 * for the 3x3 example above.
 */
inline int SquarePatternHandle::shift(int i, int j) {
    return (maxBitIndex - j - N*i);
}

/* get the i,jth element of the pattern */
inline bool SquarePatternHandle::get(SquarePattern p, int i, int j) {
    return ((p >> shift(i,j)) & 0x01);
}

/* set the i,jth element of the pattern */
inline void SquarePatternHandle::set(SquarePattern& p, int i, int j) {
    p |= ((0x01) << shift(i,j));
}

/* clear the i,jth element of the pattern */
inline void SquarePatternHandle::clear(SquarePattern& p, int i, int j) {
    p &= ~((0x01) << shift(i,j));
}

/* set the i,jth element of the pattern to the given value */
inline void SquarePatternHandle::set(SquarePattern& p, int i, int j, bool val) {
    if(val) set(p, i, j);
    else    clear(p, i, j);
}

/* function used internally to build one pattern from a rotated pattern */
inline void SquarePatternHandle::build(SquarePattern& p1, SquarePattern p2, int o) {

}

/* rotate a given pattern by the given multiple of pi/2 */
inline SquarePattern SquarePatternHandle::rotate(SquarePattern p, int mult) {
    SquarePattern returnPattern = NULL_PATTERN;
    unsigned int angle = ((mult < 0)? (4 - (-mult) % 4) : (mult % 4));

    if(angle == 1)
    for(int i = 0; i < N; ++i) {
        for(int j = 0; j < N; ++j) {
            set(returnPattern, i, j, get(p, j, N-1 - i));
        }
    }
    else if(angle == 2)
    for(int i = 0; i < N; ++i) {
        for(int j = 0; j < N; ++j) {
            set(returnPattern, i, j, get(p, N-1 - i, N-1 - j));
        }
    }
    else if(angle == 3)
    for(int i = 0; i < N; ++i) {
        for(int j = 0; j < N; ++j) {
            set(returnPattern, i, j, get(p, N-1 - j, i));
        }
    }
    else
      returnPattern = p;
    return returnPattern;
}

/* determine if two patterns are equivalent */
bool SquarePatternHandle::areEquivalent(SquarePattern patternA, SquarePattern patternB) {
    if(patternA == patternB) return true;
    else for(int i = 1; i < 4; ++i) {
        if(patternA == rotate(patternB, i)) return true;
    }
    return false;
}

/* finds the matching rotation (SquarePattern and angle)
 * for the given pattern
 */
rotation SquarePatternHandle::findMatchingPattern(SquarePattern p) {
    std::map<SquarePattern, rotation>::iterator i = list.find(p);
    return ((i != list.end())? i->second : nullRotation);
}

/* print a pattern to standard out */
void SquarePatternHandle::print(SquarePattern p, std::ostream& out) {
    for(int i = N; i > 0; --i) {
        for(int j = 0; j < N; ++j) {
            out << get(p, i-1, j) << " ";
        }
        out << "\n";
    }

    out << std::endl;
}

/* print a pattern to standard out */
void SquarePatternHandle::print(SquarePattern p) {
    print(p, std::cout);
}

/* create a pattern from an NxN boolean array */
SquarePattern SquarePatternHandle::createPattern(bool** patternArray) {
    SquarePattern returnPattern = NULL_PATTERN;
    for(int i = 0; i < N; ++i) {
        for(int j = 0; j < N; ++j) {
            if(patternArray[N - i - 1][j]) set(returnPattern, i, j);
        }
    }

    return returnPattern;
}

/* add a pattern to the list of patterns to search */
void SquarePatternHandle::add(SquarePattern p) {
    for(int angle = 0; angle < 4; ++angle) {
        rotation newRotation;
        newRotation.pattern = p;
        newRotation.angle   = angle;
        list[rotate(p, angle)] = newRotation;
    }
}

#endif
