#pragma once
#include <iostream>
#include <cmath>
using std::max;
using std::abs;
class DistanceEuclidian {
protected:
	int calculateDistance(int xCur, int yCur, int xDst, int yDst) {
		static int xd,yd,d;
		xd=xDst-xCur;
		yd=yDst-yCur;
		d=static_cast<int>(sqrt(xd*xd+yd*yd));
		return d;
	}
};

class DistanceManhattan {
protected:	
	int calculateDistance(int xCur, int yCur, int xDst, int yDst) {
		static int xd,yd,d;
		xd=xDst-xCur;
		yd=yDst-yCur;
		d=abs(xd)+abs(yd);
		return d;
	}
};

class DistanceChebyshev {
protected:
	int calculateDistance(int xCur, int yCur, int xDst, int yDst) {
		static int xd,yd,d;
		xd=xDst-xCur;
		yd=yDst-yCur;
		d=max(abs(xd),abs(yd));
		return d;
	}
};
