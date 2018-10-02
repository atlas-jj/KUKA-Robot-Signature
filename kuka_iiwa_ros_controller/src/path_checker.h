/****
-----------------------how to use--------------------
1 contruct the class with msg.
2 getPathPositions().
3 getNormalVects()
****/

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "normal_surface_calc/targetPoints.h"
using namespace std;
using namespace cv;
class path_checker
{
  public:
    path_checker();
    path_checker(const normal_surface_calc::targetPoints::ConstPtr& _msg);
    path_checker(const normal_surface_calc::targetPoints::ConstPtr& _msg, vector< vector <Point2d> > _inputPoints);
    ~path_checker();
    vector<Point3d> getPathPositions();
    vector<Point3d> getNormalVects();
    vector< vector <Point3d> > getStrokesPathPositions();
    vector< vector <Point3d> > getStrokesNormalVects();

  private:
    int detectNAN(int i);
    void processNANs();
    void getFlags();
    int getNearestLeft1(int i);
    int getNearestRight1(int i);
    void chopFirstAndLastZero();
    int getPointOfStrokeIndex(int pointIndex);
    vector<Point> getStrokeIndex();
    //Point3d ProcessSingleNormal(int _index);
    normal_surface_calc::targetPoints::ConstPtr msg;
    vector<int> flags;
    vector<Point3d> path_positions;
    vector<Point3d> path_normals;
    vector<int> validIndex;
    vector< vector <Point2d> > strokes;
    vector< Point> strokeIndex;//Point.x this stroke's start index, Point.y this stroke's end index.
};
