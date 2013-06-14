#ifndef _CenterCircleDetector_h
#define _CenterCircleDetector_h


#include <list>
#include <vector>

#include <time.h>

#include <boost/random.hpp>
#include <boost/random/uniform_real.hpp>

struct Ellipse;

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
    float theta; // the angle between major and x axes
    float ver;

    friend std::ostream& operator<< (std::ostream &o, const Ellipse &e)
    {
        return o << "center: " << e.center << ",\tmajor: " << e.major
                 << ",\tminor: " << e.minor << ",\torientation: " << e.theta
                 << ",\tscore: " << e.ver;
    }
};

class CenterCircleDetector {

public:

    CenterCircleDetector(boost::shared_ptr<NaoPose> _pose);
    virtual ~CenterCircleDetector() {};

    void detect(int upperBound,
                int* field_edge,
                const uint16_t *img);

    boost::shared_ptr<Gradient> getEdges() { return cGradient; };

    bool generateEllipse(int points[3], Ellipse &out);
    bool generateEllipseCenter(int points[3], point<int>& out);

    Circle generateCircle(point<float> a, point<float> b, point<float> c);
    point<float> getCircleCenter() { return centerCircleGuess.center; };

    std::vector<Ellipse> getEllipses() { return ellipses; }

private:

    float distanceBetweenPoints(point<float> a, point<float> b);
    int distanceBetweenPoints(point<int> a, point<int> b);
    int getR(int x, int y, int t);

    bool verifyEllipse(Ellipse &out);

    void reset();

    boost::shared_ptr<EdgeDetector> cEdges;
    boost::shared_ptr<Gradient> cGradient;

    boost::shared_ptr<NaoPose> pose;

    Circle centerCircleGuess;
    std::vector<Ellipse> ellipses;

    bool debugEllipse;

};

}
}


#endif // _CenterCircleDetector_h 
