
#include <algorithm>    // for sort() and merge()
#include <boost/shared_ptr.hpp>

#include "HoughFieldLines.h"
#include "FieldLines/CornerDetector.h"
#include "FieldLines/FieldLinesDetector.h"
using namespace std;

namespace man {
namespace vision {


HoughFieldLines::HoughFieldLines(Vision *visPtr, boost::shared_ptr<NaoPose> posePtr )
{

    vision = visPtr;
    pose = posePtr;


}

void HoughFieldLines::reset()
{
    linesList.clear();
}

void HoughFieldLines::lineLoop()
{
    reset();

    createLines(); // this makes the initial lines via the hough transform.
                   // line sanity checks still needed.

    checkLineWidth();

    checkFieldEdge();

}


void HoughFieldLines::cornerLoop()
{


}

void HoughFieldLines::createLines()
{
	vision->linesDetector->detect(vision->thresh->getVisionHorizon(),
								  vision->thresh->field->getTopEdge(),
								  vision->yImg);

	vision->cornerDetector->detect(vision->thresh->getVisionHorizon(),
								   vision->thresh->field->getTopEdge(),
								   vision->linesDetector->getLines());

	const vector<HoughVisualLine>& h_lines = vision->linesDetector->getLines();

	vector<HoughVisualLine>::const_iterator line_iter;
	for (line_iter = h_lines.begin(); line_iter != h_lines.end(); line_iter++) {
		pair<HoughLine, HoughLine> lp = line_iter->getHoughLines();
		boost::shared_ptr<VisualLine> vl (new VisualLine(lp.first, lp.second,
														 *vision->linesDetector->getEdges()));
        if (vl->getStartpoint().x == 0 && vl->getStartpoint().y == 0
            && vl->getEndpoint().x == 0 && vl->getEndpoint().y == 0)
            continue;
		linesList.push_back(vl);
	}

}

void HoughFieldLines::checkLineWidth()
{

//    vector<boost::shared_ptr<VisualLine> >::iterator i;
    for (int i = 0; i < linesList.size(); i++)
    {
        cout << "i is " << i << std::endl;
        estimate topL = pose->pixEstimate(linesList[i].get()->tl.x,
                                          linesList[i].get()->tl.y, 0);
        estimate botL = pose->pixEstimate(linesList[i].get()->bl.x,
                                          linesList[i].get()->bl.y, 0);
        float distL = pose->getDistanceBetweenTwoObjects(topL, botL);

        estimate topR = pose->pixEstimate(linesList[i].get()->tr.x,
                                          linesList[i].get()->tr.y, 0);
        estimate botR = pose->pixEstimate(linesList[i].get()->br.x,
                                          linesList[i].get()->br.y, 0);
        float distR = pose->getDistanceBetweenTwoObjects(topR, botR);

        if ((distL+distR)/2 > 3*LINE_WIDTH) // check average width
            linesList.erase(linesList.begin() + i);
    }

}

void HoughFieldLines::checkFieldEdge()
{

}

}
}
