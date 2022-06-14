// Foil2OpenScad.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>

#include "Bezier.h"

using namespace std;

//struct Point
//{
//Point(float x, float y) : mX(x), mY(y)
//{
//}
//double mX;
//double mY;
//};

//typedef std::vector<Point> TPoints;

int _tmain(int argc, _TCHAR* argv[])
{
if(argc != 2)
   {
   cout<<"Usage: airfoil.txt"<<endl;
   return 1;
   }

ifstream is;
//const wchar_t *inname = argv[1];
const char* inName = argv[1];
is.open(inName);

if(!is.good())
   {
   cout<<"Could not open "<<inName<<endl;
   return 2;
   }
  

char lineBuf[128];
is.getline(lineBuf,sizeof(lineBuf));
string name(lineBuf);
if(name.back()==' ')
   name.pop_back();

string outName = name;
outName += ".scad";

ofstream os;
os.open(outName);

bool reverse = false; //reverse direction of rotation in output file
TPoints points;
TPoints topPoints;
TPoints bottomPoints;

TPoints unalignedPoints;

TPoints allPoints;

bool top = true;


float lastX = 2;
while(true)
   {
   //double x,y;
   float x,y;
   is>>x>>y;
   Point point(x,y);
   //points.push_back(point);
   if(x >= lastX)
      {//turning point
      top = false;
      }
   if(top)
      topPoints.push_back(point);
   else
      bottomPoints.push_back(point);
      
   allPoints.push_back(point);

   lastX = x;

   if(!is.good())
      break;
   }


   double maxThickness = 0;
   double maxChord = 0;
   double maxThickChord = 0;
//if(topPoints.size() != bottomPoints.size())
   {
   bool swappedTopBot = false;
   if(topPoints.size() > bottomPoints.size())
      {
      swappedTopBot = true;
      std::swap(topPoints, bottomPoints);
      }
   //Now topPoints is shorter
   if (topPoints.size() < 50)
      {
      //Interpolate
      BezierSpline interpolator(topPoints);
      interpolator.InterpBy(2, topPoints);

      if (topPoints.size() > bottomPoints.size())
         {
         swappedTopBot = !swappedTopBot;
         std::swap(topPoints, bottomPoints);
         }
      }

   bool topReversed = false;
   bool bottomReversed = false;
   //TPoints topPointsRev = topPoints;
   //Make both top and bottompoints start at x ==0
   if(topPoints.front().mX > topPoints.back().mX)
      {
      topReversed = true;
      std::reverse(topPoints.begin(), topPoints.end());
      }

   if(bottomPoints.front().mX > bottomPoints.back().mX)
      {
      bottomReversed = true;
      std::reverse(bottomPoints.begin(), bottomPoints.end());
      }


   TPoints bottomPointsOut;
   auto it = bottomPoints.begin();
   auto endIt = bottomPoints.end();
   for(auto point: topPoints)
      {
      auto x = point.mX;
      //find the bottom points on either side of the top point
      auto gte = std::find_if(it, endIt, [x](const Point & p) { return p.mX >= x; });
      int gteI = std::distance(it,gte);
      double thickness = 0;
      if(gteI > 0)
         {
         double x1 =  bottomPoints[gteI-1].mX;
         double x2 = bottomPoints[gteI].mX;
         double y1 =  bottomPoints[gteI-1].mY;
         double y2 = bottomPoints[gteI].mY;
         double dxFrac = (x-x1)/(x2-x1);
         double y = y1*(1-dxFrac) + y2*dxFrac;
         bottomPointsOut.push_back(Point(x,y));
         thickness = std::abs(y-point.mY);
         }
      else
         {
         double y = bottomPoints.front().mY;
         bottomPointsOut.push_back(Point(x,y));
         thickness = std::abs(y-point.mY);
         }
      if(thickness > maxThickness)
         {
         maxThickness = thickness;
         maxThickChord = x;
         }
      }

   maxChord = abs(topPoints.front().mX-topPoints.back().mX);

   if(swappedTopBot)
      std::swap(topPoints, bottomPoints);
      
   if(topReversed)
      std::reverse(topPoints.begin(), topPoints.end());

   if (bottomReversed)
      {
      std::reverse(bottomPointsOut.begin(), bottomPointsOut.end());
      std::reverse(bottomPoints.begin(), bottomPoints.end());
      }

   points.insert(points.end(),topPoints.begin(),topPoints.end());
   points.insert(points.end(),bottomPointsOut.begin(),bottomPointsOut.end());

   unalignedPoints.insert(unalignedPoints.end(), topPoints.begin(), topPoints.end());
   unalignedPoints.insert(unalignedPoints.end(), bottomPoints.begin(), bottomPoints.end());

   //Make the points go clockwise
   if (reverse)
      {
      std::reverse(points.begin(), points.end());
      std::reverse(unalignedPoints.begin(), unalignedPoints.end());
      }
   }
//else
//   {
//   points.insert(points.end(),topPoints.begin(),topPoints.end());
//   points.insert(points.end(),bottomPoints.begin(),bottomPoints.end());
//   }

double thickPercent = 100.0*maxThickness/maxChord;
double maxThickChordPercent = 100.0*maxThickChord/maxChord;

os<<name.c_str()<<"ThickPercent ="<<thickPercent<<";"<<endl;
os<<name.c_str()<<"MaxThickChordPercent ="<<maxThickChordPercent<<";"<<endl;

os<<name.c_str()<<"Aligned =[";
bool first = true;
int count = 0;
for(auto point: points)
   {
   if(!first)
      os<<',';
   os<<'['<<point.mX<<','<<point.mY<<']';

   if((++count & 7) == 0)
      os<<endl;
   ////geo::append(lineFoil,TPoint(x,y));
   //inputPoly.outer().push_back(TPoint(1.0-x,y)); //We reflect about y axis to make the winding clockwise as boost::geometry expects.
   first = false;
   }
os<<"];"<<endl<<endl;

first = true;
count = 0;
os<<name.c_str()<<"=[";
for(auto point: unalignedPoints)
   {
   if(!first)
      os<<',';
   os<<'['<<point.mX<<','<<point.mY<<']';

   if((++count & 7) == 0)
      os<<endl;
   ////geo::append(lineFoil,TPoint(x,y));
   //inputPoly.outer().push_back(TPoint(1.0-x,y)); //We reflect about y axis to make the winding clockwise as boost::geometry expects.
   first = false;
   }
os<<"];";


return 0;
}

