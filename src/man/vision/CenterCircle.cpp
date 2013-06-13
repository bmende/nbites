#include "CenterCircle.h"

#include <stdio.h>


using namespace std;

namespace man {
namespace vision {



CenterCircleDetector::CenterCircleDetector(boost::shared_ptr<NaoPose> _pose) :
    cEdges(new EdgeDetector),
    cGradient(new Gradient)
{
    pose = _pose;
}


void CenterCircleDetector::detect(int upperBound,
                                  int* field_edge,
                                  const uint16_t *img)
{

    upperBound -= 10; // just in case horizon is too low

    upperBound = min(max(0, upperBound), IMAGE_HEIGHT - 3);

    cGradient->reset();
    cEdges->detectEdges(upperBound, field_edge, img, *cGradient);

    cout << cGradient->numPeaks << " is the number of gradient peaks\n";

    point<float> a, b, c;
    a.x = 15; a.y = 4;
    b.x = 9; b.y = 10;
    c.x = 3; c.y = 4;

    generateCircle(a, b, c);

}

Circle CenterCircleDetector::generateCircle(point<float> a,
                                            point<float> b,
                                            point<float> c) {

    float yDelta_A = b.y - a.y;
    float xDelta_A = b.x - a.x;
    float yDelta_B = c.y - b.y;
    float xDelta_B = c.x - b.x;

    float aSlope = yDelta_A / xDelta_A;
    float bSlope = yDelta_B / xDelta_B;

    Circle circ;

    circ.x = ((aSlope * bSlope * (a.y - c.y)) + (bSlope * (a.x + b.x))
              - (aSlope * (b.x + c.x))) / (2 * (bSlope - aSlope));

    circ.y = (-1*(circ.x - (a.x + b.x)/2) / aSlope) + ((a.y + b.y) / 2);

    cout << circ.x << ", " << circ.y << endl;

    return circ;
}



}
}
