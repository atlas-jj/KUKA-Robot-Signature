#include "../include/transformation2D.h"

/** constuctor
_t: 2d translation
_scale: scale
_rotation: rotation in degrees
1 do translation
2 do rotation.
3 do scale
*/
transformation2D::transformation2D(Point _t, double _scale, double _rotation)
{
  t=_t;
  scale=_scale;
  rotation=_rotation;
}
transformation2D::~transformation2D(){}
Point2d transformation2D::doTransformation(Point2d p1)
{
   //get the 2*3 transformation matrix
   cout<<"p1"<<p1<<endl;
   double r=rotation*CV_PI/180;
   Mat_<double> transformation(2,3);
   transformation(0,0)=cos(r);transformation(0,1)=-sin(r);transformation(0,2)=t.x*cos(r)-t.y*sin(r);
   transformation(1,0)=sin(r);transformation(1,1)=cos(r);transformation(1,2)=t.x*sin(r)+t.y*cos(r);

   std::cout<<"matrix:"<<transformation<<endl;
   cv::Mat_<double>p13(3,1);
   p13(0,0)=p1.x;
   p13(1,0)=p1.y;
   p13(2,0)=1.0;
   cv::Mat_<double> p2=transformation*p13;
   return Point2d(p2(0,0),p2(1,0));
}
