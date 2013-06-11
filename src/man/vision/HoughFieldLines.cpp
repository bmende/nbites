
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

    PROF_ENTER(P_HOUGH);
	vision->linesDetector->detect(vision->thresh->getVisionHorizon(),
								  vision->thresh->field->getTopEdge(),
								  vision->yImg);

	vision->cornerDetector->detect(vision->thresh->getVisionHorizon(),
								   vision->thresh->field->getTopEdge(),
								   vision->linesDetector->getLines());
    PROF_EXIT(P_HOUGH);
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

    for (int i = 0; i < linesList.size(); i++)
    {
        //trying to check width by pixel rather than reality.
        int topLY = linesList[i].get()->getTopLeftEndpoint().y;
        int topLX = linesList[i].get()->getTopLeftEndpoint().x;
        int botLY = linesList[i].get()->getBottomLeftEndpoint().y;
        int botLX = linesList[i].get()->getBottomLeftEndpoint().x;

        int distL = pow(topLY - botLY, 2) + pow(topLX - botLX, 2);
        distL = sqrt(distL);

        int topRY = linesList[i].get()->getTopRightEndpoint().y;
        int topRX = linesList[i].get()->getTopRightEndpoint().x;
        int botRY = linesList[i].get()->getBottomRightEndpoint().y;
        int botRX = linesList[i].get()->getBottomRightEndpoint().x;

        int distR = pow(topRY - botRY, 2) + pow(topRX - botRX, 2);
        distR = sqrt(distR);

        // estimate topL = pose->pixEstimate(linesList[i].get()->tl.x,
        //                                   linesList[i].get()->tl.y, 0);
        // estimate botL = pose->pixEstimate(linesList[i].get()->bl.x,
        //                                   linesList[i].get()->bl.y, 0);
        // float distL = pose->getDistanceBetweenTwoObjects(topL, botL);

        // estimate topR = pose->pixEstimate(linesList[i].get()->tr.x,
        //                                   linesList[i].get()->tr.y, 0);
        // estimate botR = pose->pixEstimate(linesList[i].get()->br.x,
        //                                   linesList[i].get()->br.y, 0);
        // float distR = pose->getDistanceBetweenTwoObjects(topR, botR);

        if (distL > 10 || distR > 10) { //check average width
            linesList.erase(linesList.begin() + i);
            //           cout << "THROWING OUT THICK-ASS LINE\n";
        }
    }

}

void HoughFieldLines::checkFieldEdge()
{

    int *topEdge = vision->thresh->field->getTopEdge();

    for (int i = 0; i < linesList.size(); i++)
    {
        int startY = linesList[i].get()->getStartpoint().y;
        int startX = linesList[i].get()->getStartpoint().x;
        int endX = linesList[i].get()->getEndpoint().x;
        int endY = linesList[i].get()->getEndpoint().y;

        if (startY < topEdge[startX] && endY < topEdge[endX]) {
            // cout << "\n\nAll of the line is over the edge " << i << std::endl;
            linesList.erase(linesList.begin() + i);
        }
    }
}
}
}
