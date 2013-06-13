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

    point<float> points[3];
    Circle cur, best;

    boost::mt19937 gen(time(0));
    boost::uniform_int<> distro(0, cGradient->numPeaks - 1);

    float bestRANSACVar = 10000;
    int numAttempts = 0.2*cGradient->numPeaks;
    int bestNumPoints = 0;
    for (int j = 0; j < 40; j++) { // this is the ransac thing

        float bestVar = 10000, centerRad = 750;
        for (int n = 0; n < numAttempts; n++) {

            for (int i = 0; i < 3; i++) {
                int edge_number = distro(gen); // some random number between 0 and numPeaks;
                int img_x = cGradient->getAnglesXCoord(edge_number) + IMAGE_WIDTH/2;
                int img_y = cGradient->getAnglesYCoord(edge_number) + IMAGE_HEIGHT/2;
                estimate e = pose->pixEstimate(img_x, img_y, 0.0f);
                points[i].x = e.x;
                points[i].y = e.y;
            }

            cur = generateCircle(points[0], points[1], points[2]);
            if ((fabs(cur.radius - centerRad)) < bestVar) {
                bestVar = fabs(cur.radius - centerRad);
                best = cur;
            }
        }
        int numPoints = 0;
        float var = 0;
        for (int n = 0; n < cGradient->numPeaks; n++) {

            int img_x = cGradient->getAnglesXCoord(n) + IMAGE_WIDTH/2;
            int img_y = cGradient->getAnglesYCoord(n) + IMAGE_HEIGHT/2;
            estimate e = pose->pixEstimate(img_x, img_y, 0.0f);

            point<float> test; test.x = e.x; test.y = e.y;
            float difference = fabs(distanceBetweenPoints(test, best.center) - centerRad);
            if (difference < 15) {
                var += pow(difference, 2);
                numPoints++;
            }
        }
        var = 0.2*var - numPoints;
        if (var < bestRANSACVar) {
            bestRANSACVar = var;
            centerCircleGuess = best;
            bestNumPoints = numPoints;
        }

    }

    best = centerCircleGuess;
    cout << best.center.x << ", " << best.center.y << " radius: " << best.radius;
    cout << " and there are " << bestNumPoints << " in it\n";

    estimate f = pose->pixEstimate(best.center.x, best.center.y, 0);
    cout << "center is distance " << f.dist << " and bearing " << f.bearing << endl;


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

    circ.center.x = ((aSlope * bSlope * (a.y - c.y)) + (bSlope * (a.x + b.x))
              - (aSlope * (b.x + c.x))) / (2 * (bSlope - aSlope));

    circ.center.y = (-1*(circ.center.x - (a.x + b.x)/2) / aSlope) + ((a.y + b.y) / 2);

    circ.radius = distanceBetweenPoints(circ.center, a);
  
    return circ;
}

float CenterCircleDetector::distanceBetweenPoints(point<float> a,
                                                  point<float> b)
{

    float dist = sqrt( pow((a.x - b.x), 2) + pow((a.y - b.y), 2) );

    return dist;
}

}
}
