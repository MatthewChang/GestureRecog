#ifndef GESTURES
#define GESTURES

#include <opencv2/core/core.hpp>

using namespace cv;
using namespace std;

class GestureComponent {
	int frame_duration;
	Point vector;
public:
	GestureComponent(int duration, Point v) {
		frame_duration = duration;
		vector = v;
	}
};

class Gesture {
	string name;
	vector<GestureComponent> components;
public:
	Gesture(string n) {
		name = n;
	}
	void addComponent(int d,float x,float y) {
		Point p(x, y);
		GestureComponent c(d, p);
	}
};

vector<Gesture> gestures;
Gesture circle("Circle");
#endif