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

    boost::mt19937 gen(time(0));
    boost::uniform_int<> distro(0, cGradient->numPeaks - 1);

    int points[3];
    for (int i = 0; i < 3; i++) {
        points[i] = distro(gen);
    }

    generateEllipseCenter(points);



}

point<int> CenterCircleDetector::generateEllipseCenter(int points[3])
{

    HoughLine lines[3];
    int x[3], y[3];
    for (int i = 0; i < 3; i++) {
        int x[i] = cGradient->getAnglesXCoord(points[i]);
        int y[i] = cGradient->getAnglesYCoord(points[i]);
        int t = cGradient->getAngle(points[i]);
        int r = getR(x[i], y[i], t);
        HoughLine l(r, t, 0);
        lines[i] = l;
    }

    point<int> t_12;
    if (!lines[0].intersects(lines[1], t_12)) {
        t_12 = NULL;
}

int CenterCircleDetector::getR(int x, int y, int t)
{
    float a = static_cast<float>(t & 0xff) * M_PI_FLOAT / 128.0f;
    return static_cast<int>(floor(static_cast<float>(x) * cos(a) +
                                  static_cast<float>(y) * sin(a)));
}

float CenterCircleDetector::distanceBetweenPoints(point<float> a,
                                                  point<float> b)
{

    float dist = sqrt( pow((a.x - b.x), 2) + pow((a.y - b.y), 2) );

    return dist;
}



/**************************** OUTDATED CIRCLE STUFF ************************/
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

    circ.center.x = ((aSlope * bSlope * (a.y - c.y)) + (bSlope * (a.x + b.x))
              - (aSlope * (b.x + c.x))) / (2 * (bSlope - aSlope));

    circ.center.y = (-1*(circ.center.x - (a.x + b.x)/2) / aSlope) + ((a.y + b.y) / 2);

    circ.radius = distanceBetweenPoints(circ.center, a);

    return circ;
}
}
}
