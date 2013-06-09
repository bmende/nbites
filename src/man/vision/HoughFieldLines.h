#pragma once


// System includes
#include <math.h>                       // use fabs, max, min
#include <vector>                      //
#include <list>                        //
#include <sstream>                     //
#include <iomanip> // setprecision for cout

namespace man {
namespace vision {
class HoughFieldLines; // forward reference
}
}


#include "FieldConstants.h"
#include "Common.h" //
#include "VisualFieldObject.h" //
#include "ConcreteFieldObject.h" //
#include "ConcreteCorner.h" //
#include "VisualCorner.h" //
#include "VisualLine.h"
#include "Utility.h" //
#include "NaoPose.h" // Used to estimate distances in the image
#include "Vision.h"



namespace man {
namespace vision {


class HoughFieldLines {


public:


    HoughFieldLines(Vision *visPtr, boost::shared_ptr<NaoPose> posePtr);

    virtual ~HoughFieldLines() {}

    void reset();

    void lineLoop();

    void cornerLoop();

    const std::vector < boost::shared_ptr<VisualLine> >* getLines()
        const { return &linesList; }


private:

    void createLines();
    void checkLineWidth(); // make sure lines aren't too thick
    void checkFieldEdge(); // make sure lines are under the field edge

private:


    Vision *vision;
    boost::shared_ptr<NaoPose> pose;


    std::vector<boost::shared_ptr<VisualLine> > linesList;
};




}
}
