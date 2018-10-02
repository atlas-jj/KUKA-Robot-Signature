#include "path_checker.h"


path_checker::path_checker(const normal_surface_calc::targetPoints::ConstPtr& _msg)
{
   msg=_msg;
   size_t pointNum=msg->path_robot.size();
  //  for(int i=0;i<pointNum;i++)
  //     validIndex.push_back(i);
   processNANs();
}

path_checker::~path_checker(){}
path_checker::path_checker(const normal_surface_calc::targetPoints::ConstPtr& _msg, vector< vector <Point2d> > _inputPoints)
{
  msg=_msg;
  size_t pointNum=msg->path_robot.size();
  strokes=_inputPoints;
  strokeIndex=getStrokeIndex();
 //  for(int i=0;i<pointNum;i++)
 //     validIndex.push_back(i);
  processNANs();
}
vector<Point3d> path_checker::getPathPositions()
{
  //TODO do some processNANs
  size_t validPointNum=validIndex.size();
  for(int i=0;i<validPointNum;i++)
        path_positions.push_back(Point3d(msg->path_robot[validIndex[i]].x, msg->path_robot[validIndex[i]].y, msg->path_robot[validIndex[i]].z));
  return  path_positions;
}

/**get points in strokes structure.
*/
vector< vector <Point3d> > path_checker::getStrokesPathPositions()
{
  vector< vector <Point3d> > rtvects;
  size_t strokesNum=strokes.size();
  //initialize
  rtvects.resize(strokesNum);

  size_t validPointNum=validIndex.size();
  for(int i=0;i<validPointNum;i++)
  {
      Point3d thisPoint(msg->path_robot[validIndex[i]].x, msg->path_robot[validIndex[i]].y, msg->path_robot[validIndex[i]].z);
      int thisStroke=getPointOfStrokeIndex(validIndex[i]);
      rtvects[thisStroke].push_back(thisPoint);
  }

  return  rtvects;
}

/**return the stroke index that this point is in.
default, return 0
*/
int path_checker::getPointOfStrokeIndex(int pointIndex)
{
    //vector< Point> strokeIndex;
    size_t strokesNum=strokeIndex.size();
    for(int i=0;i<strokesNum;i++)
    {
      Point strokeStartEnd=strokeIndex[i];
      if(pointIndex>=strokeStartEnd.x&&pointIndex<=strokeStartEnd.y)
          return i;
    }
    return 0;
}

vector<Point3d> path_checker::getNormalVects()
{
  size_t validPointNum=validIndex.size();
   for(int i=0;i<validPointNum;i++)
   {
     if(flags[validIndex[i]]==1)
         path_normals.push_back(Point3d(msg->normals_robot[validIndex[i]].x, msg->normals_robot[validIndex[i]].y, msg->normals_robot[validIndex[i]].z));
     else
     {
       int left1Index=getNearestLeft1(i);
       int rightIndex=getNearestRight1(i);
       double avrgX=(msg->normals_robot[left1Index].x+msg->normals_robot[rightIndex].x)/2;
       double avrgY=(msg->normals_robot[left1Index].y+msg->normals_robot[rightIndex].y)/2;
       double avrgZ=(msg->normals_robot[left1Index].z+msg->normals_robot[rightIndex].z)/2;
       path_normals.push_back(Point3d(avrgX,avrgY,avrgZ));
     }
   }

  return path_normals;
}

/**get normals in strokes structure.
*/
vector< vector <Point3d> > path_checker::getStrokesNormalVects()
{
  vector< vector <Point3d> > rtvects;
  size_t strokesNum=strokes.size();
  //initialize
  rtvects.resize(strokesNum);

  size_t validPointNum=validIndex.size();
   for(int i=0;i<validPointNum;i++)
   {
     Point3d thisPoint(0,0,0);
     if(flags[validIndex[i]]==1)
         thisPoint=Point3d(msg->normals_robot[validIndex[i]].x, msg->normals_robot[validIndex[i]].y, msg->normals_robot[validIndex[i]].z);
     else
     {
       int left1Index=getNearestLeft1(i);
       int rightIndex=getNearestRight1(i);
       double avrgX=(msg->normals_robot[left1Index].x+msg->normals_robot[rightIndex].x)/2;
       double avrgY=(msg->normals_robot[left1Index].y+msg->normals_robot[rightIndex].y)/2;
       double avrgZ=(msg->normals_robot[left1Index].z+msg->normals_robot[rightIndex].z)/2;
       thisPoint=Point3d(avrgX,avrgY,avrgZ);
     }
     int thisStroke=getPointOfStrokeIndex(validIndex[i]);
     rtvects[thisStroke].push_back(thisPoint);
   }
  return rtvects;
}

vector<Point> path_checker::getStrokeIndex()
{
  vector<Point> vec;
  size_t strokesNum=strokes.size();
  int counter=0;
  for(int i=0;i<strokesNum;i++)
  {
    size_t thisStrokePointNum=strokes[i].size();
    vec.push_back(Point(counter, counter+thisStrokePointNum-1));
    counter+=thisStrokePointNum;
  }
  return vec;
}

/**the overall process
*/
void path_checker::processNANs()
{
  getFlags();//get the flags vector
  //size_t pointNum=flags.size();
  chopFirstAndLastZero();
  //search in the validIndex for nearest left 1, and nearest right1
}

/**
i is index in the validIndex vector
search in the validIndex vector;
return the index in flags, or the original index;
*/
int path_checker::getNearestLeft1(int i)
{
   for(int m=i;m>0;m--)
   {
     int thisFlag=flags[validIndex[m]];
     if(thisFlag==1)
        return validIndex[m];
   }
}

/**
i is index in the validIndex vector
search in the validIndex vector;
return the index in flags, or the original index;
*/
int path_checker::getNearestRight1(int i)
{
  size_t validPointNum=validIndex.size();
   for(int m=i;m<validPointNum;m++)
   {
     int thisFlag=flags[validIndex[m]];
     if(thisFlag==1)
        return validIndex[m];
   }
}

void path_checker::chopFirstAndLastZero()
{
  int flagsNum=flags.size();
  if(flagsNum<2)
  {
    validIndex.push_back(0);
    return;
  }
  int firstOneMark=0;
  int lastOneMark=flagsNum-1;
   //process the first zeros
   if(flags[0]==0)
   {
     for(int i=0;i<flagsNum;i++)
     {
          if(flags[i]==1)
          {
            firstOneMark=i;
            break;
          }
     }
   }

   //process the last zeros
   if(flags[flagsNum-1]==0)
   {
     for(int i=flagsNum-1;i>-1;i--)
     {
          if(flags[i]==1)
          {
            lastOneMark=i;
            break;
          }
     }
   }

  for( int i=firstOneMark;i<=lastOneMark;i++)
     validIndex.push_back(i);
}



/**check NAN, return 0 if NAN detected.
*/
int path_checker::detectNAN(int i)
{
  size_t pointNum=msg->path_robot.size();
  if(i<pointNum)
  {
    bool rtb=1;
    // if(isnan(msg->path_robot[i].x)||isnan(msg->path_robot[i].y)||isnan(msg->path_robot[i].z))
    //     rtb=true;
    if(isnan(msg->normals_robot[i].x)||isnan(msg->normals_robot[i].y)||isnan(msg->normals_robot[i].z))
        rtb=0;
    return rtb;
  }
  else
     return 0;
}

void path_checker::getFlags()
{
  size_t pointNum=msg->path_robot.size();
  for(int i=0;i<pointNum;i++)
     flags.push_back(detectNAN(i));
}
