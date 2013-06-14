#ifndef _CenterCircleDetector_h
#define _CenterCircleDetector_h


#include <list>
#include <vector>

#include <time.h>

#include <boost/random.hpp>
#include <boost/random/uniform_real.hpp>

#include "FieldLines/EdgeDetector.h"
#include "geom/HoughLine.h"
#include "Vision.h"


namespace man {
namespace vision {

struct Circle {
    point<float> center;
    float radius;
};

struct Ellipse {
    point<int> center;
    float major;
    float minor;
};

class CenterCircleDetector {

public:

    CenterCircleDetector(boost::shared_ptr<NaoPose> _pose);
    virtual ~CenterCircleDetector() {};

    void detect(int upperBound,
                int* field_edge,
                const uint16_t *img);

    boost::shared_ptr<Gradient> getEdges() { return cGradient; };

    Ellipse generateEllipse();
    point<int> generateEllipseCenter(int points[3]);

    Circle generateCircle(point<float> a, point<float> b, point<float> c);
    point<float> getCircleCenter() { return centerCircleGuess.center; };

private:

    float distanceBetweenPoints(point<float> a, point<float> b);
    int getR(int x, int y, int t);

    boost::shared_ptr<EdgeDetector> cEdges;
    boost::shared_ptr<Gradient> cGradient;

    boost::shared_ptr<NaoPose> pose;

    Circle centerCircleGuess;


};

}
}


#endif // _CenterCircleDetector_h 
