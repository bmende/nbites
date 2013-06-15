#include "CenterCircle.h"

#include <stdio.h>


using namespace std;
using namespace boost::numeric;

namespace man {
namespace vision {



CenterCircleDetector::CenterCircleDetector(boost::shared_ptr<NaoPose> _pose) :
    cEdges(new EdgeDetector),
    cGradient(new Gradient)
{
    pose = _pose;
    debugEllipse = true;
}


void CenterCircleDetector::reset()
{
    ellipses.clear();
}

void CenterCircleDetector::detect(int upperBound,
                                  int* field_edge,
                                  const uint16_t *img)
{

    upperBound += 10; // just in case horizon is too low
    //   upperBound = min(max(0, upperBound), IMAGE_HEIGHT - 3);


    reset();
    cGradient->reset();
    cEdges->detectEdges(upperBound, field_edge, img, *cGradient);

    if (debugEllipse)
        cout << cGradient->numPeaks << " is the number of gradient peaks\n";

    boost::mt19937 gen(time(0));
    int maxPeaks = cGradient->numPeaks - 1;
    boost::uniform_int<> distro(maxPeaks/4, 3*maxPeaks/4);

    for (int n = 0; n < 2000; n++) {
        point<int> points[3];
        for (int i = 0; i < 3; i++) {
            int temp = distro(gen);
            points[i].x = cGradient->getAnglesXCoord(temp);
            points[i].y = cGradient->getAnglesYCoord(temp);
        }

        Ellipse result;
        if (generateEllipse(points, result)) {
            cout << result << endl;
            ellipses.push_back(result);
        }

    }


}

bool CenterCircleDetector::generateEllipse(point<int> points[3], Ellipse &out) {

    if (!generateEllipseCenter(points, out.center)) return false;

/**
   We are now solving a system of equations to find the parameters
   of the ellipse. First we translate the x's and y's to center
   the ellipse at the origin. Now we have the equation:
           Ax^2 + 2Bxy + Cy^2 = 1 where 4AC > B^2.
    We will solve for A, B, and C. If B^2 > 4AC, then we dont have
    an ellipse. If we do have an ellipse, we will see if the other
    edges lie along this ellipse. If enough do, we will return true.

 **/

    float x[3], y[3], x2[3], y2[3];
    for (int i = 0; i < 3; i++) {
        x[i] = (float)(points[i].x + IMAGE_WIDTH/2);
        y[i] = (float)(points[i].y + IMAGE_HEIGHT/2);
        x2[i] = x[i] * x[i];
        y2[i] = y[i] * y[i];
    }

    ublas::matrix<float> eqSystem(3, 3);
    eqSystem(0,0) = x2[0]; eqSystem(0,1) = 2*x[0]*y[0]; eqSystem(0,2) = y2[0];
    eqSystem(1,0) = x2[1]; eqSystem(1,1) = 2*x[1]*y[1]; eqSystem(1,2) = y2[1];
    eqSystem(2,0) = x2[2]; eqSystem(2,1) = 2*x[2]*y[2]; eqSystem(2,2) = y2[2];

    ublas::vector<float> result(3);
    result(0) = 1;
    result(1) = 1;
    result(2) = 1;
    ublas::permutation_matrix<> P(3);
    int singularRow = lu_factorize(eqSystem, P);
    if (singularRow != 0) {
        if (debugEllipse) {
            cout << "We have a singular matrix. This is bad?\n"; fflush(stdout);}
        return false;
    }
    lu_substitute(eqSystem, P, result);

    if (4*result(0)*result(2) <= result(1)*result(1)) {
        return false;
    }
    out.major = sqrt(1/result(0)); //semimajor = sqrt(1/A)
    out.minor = sqrt(1/result(2)); //semiminor = sqrt(1/C)

    if (out.minor > out.major) return false; // what we know about the center circle

    if (out.major > IMAGE_WIDTH/2 || out.minor > IMAGE_WIDTH/2) // too big
        return false;

    if (out.major < 50 || out.minor < 40) // too small
        return false;

    if (isnan(out.major) || isnan(out.minor)) return false;

    return verifyEllipse(out);

}

bool CenterCircleDetector::generateEllipseCenter(point<int> points[3], point<int>& out)
{
    static const int CENTER_DIFFERENCE_THRESHOLD = 10;

    // find tangent lines to ellipse
    HoughLine lines[3];
    for (int i = 0; i < 3; i++) {
        int t = cGradient->getAngle(
            cGradient->peaks_list_contains(points[i].y, points[i].x - 1));
        int r = getR(points[i].x, points[i].y, t);
        HoughLine l(r, t, 0);
        lines[i] = l;
    }

    // find tangent line intersections.
    point<int> t_01, t_02, t_12;
    lines[0].intersects(lines[1], t_01);
    lines[0].intersects(lines[2], t_02);
    lines[1].intersects(lines[2], t_12);

    // find bisectors of previous intersections;
    point<int> m_01((points[0].x+points[1].x)/2, (points[0].y+points[1].y)/2);
    point<int> m_02((points[0].x+points[2].x)/2, (points[0].y+points[2].y)/2);
    point<int> m_12((points[1].x+points[2].x)/2, (points[1].y+points[2].y)/2);

    float slope_01 = (float)(m_01.y - t_01.y) / (float)(m_01.x - t_01.x);
    float slope_02 = (float)(m_02.y - t_02.y) / (float)(m_02.x - t_02.x);
    float slope_12 = (float)(m_12.y - t_12.y) / (float)(m_12.x - t_12.x);

    float b_01 = m_01.y - slope_01*m_01.x;
    float b_02 = m_02.y - slope_01*m_02.x;
    float b_12 = m_12.y - slope_12*m_12.x;

    // find intersection of bisectors. This should be the center,
    // so they should all be the same if it is an ellipse;
    point<int> center[3];
    center[0].x = (b_02 - b_01) / (slope_02 - slope_01) + IMAGE_WIDTH/2;
    center[0].y = (slope_01 * center[0].x) + b_01 + IMAGE_HEIGHT/2;

    center[1].x = (b_12 - b_01) / (slope_12 - slope_01) + IMAGE_WIDTH/2;
    center[1].y = (slope_01 * center[1].x) + b_01 + IMAGE_HEIGHT/2;

    center[2].x = (b_12 - b_02) / (slope_12 - slope_02) + IMAGE_WIDTH/2;
    center[2].y = (slope_02 * center[2].x) + b_02 + IMAGE_HEIGHT/2;

    //  cout << "the centers are: " << center[0] << " " << center[1] << " " << center[2] << endl;

    int distance[3];
    distance[0] = distanceBetweenPoints(center[0], center[1]);
    distance[2] = distanceBetweenPoints(center[0], center[2]);
    distance[1] = distanceBetweenPoints(center[1], center[2]);
    //  cout << "distance between: ";
    bool close[3] = {false, false, false};
    for (int i = 0; i < 3; i++) {
        if (distance[i] < CENTER_DIFFERENCE_THRESHOLD) {
            close[i] = true;
            out.x = center[i].x;
            out.y = center[i].y;
            //      cout << out << " ";
        }
        //     cout << distance[i] << "\t     ";
    }
//    cout << endl;

    if (out.x >= 0 && out.x < IMAGE_WIDTH
        && out.y >= 0 && out.y < IMAGE_HEIGHT) {
        return (close[0] || close[1] || close[2]);
    }
    else
        return false;
}


bool CenterCircleDetector::verifyEllipse(Ellipse &e)
{

    int x_0 = e.center.x;
    int y_0 = e.center.y;
    float a = e.major;
    float b = e.minor;

    float perimeter = M_PI*(3*(a+b) - sqrt((3*a+b)*(a+3*b))); // Ramanujan's Approx.
    int increment_arc_length = 10; // find about 1 edge per ial units of circumference.
    int box = (increment_arc_length / 2) - 1; // look around the point, but not in another box
    if(box < 0) box = 0;
    float increment = (2*M_PI*increment_arc_length) / perimeter;

    int numEdges = 0;
    for (float t = 0; t < 2 * M_PI; t+=increment) {
        int x = ( x_0 + a*cos(t));
        int y = ( y_0 + b*sin(t));

        if (x > IMAGE_WIDTH - box || x < 0 + box
            || y > IMAGE_HEIGHT - box || y < 0 + box)
            continue; // point outside image, so no edges;

        for (int dx = -box; dx < box; dx++) {
            for (int dy = -box; dy < box; dy++) {
                int i = x + dx - IMAGE_WIDTH/2;
                int j = y + dy - IMAGE_HEIGHT/2;
                if (cGradient->peaks_list_contains(j, i)) numEdges++;
            }
        }
    }

    if (numEdges == 0) return false;
    e.ver = numEdges / (perimeter);
    if (e.ver < 0.25) return false;
    return true;

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

int CenterCircleDetector::distanceBetweenPoints(point<int> a,
                                                  point<int> b)
{

    float dist = sqrt( pow((a.x - b.x), 2) + pow((a.y - b.y), 2) );

    return (int)dist;
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
