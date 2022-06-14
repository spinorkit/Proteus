#pragma once

#include <vector>

struct Point
   {
   Point(double x = 0.0, double y = 0.0) :
      mX(x), mY(y)
      {
      }
   double mX;
   double mY;
   };

typedef std::vector<Point> TPoints;

/// <summary>
/// Bezier Spline methods
/// </summary>
class BezierSpline
   {
   /// <summary>
   /// Get open-ended Bezier Spline Control Points.
   /// </summary>
   /// <param name="knots">Input Knot Bezier spline points.</param>
   /// <param name="firstControlPoints">Output First Control points
   /// array of knots.Length - 1 length.</param>
   /// <param name="secondControlPoints">Output Second Control points
   /// array of knots.Length - 1 length.</param>
   /// <exception cref="ArgumentNullException"><paramref name="knots"/>
   /// parameter must be not null.</exception>
   /// <exception cref="ArgumentException"><paramref name="knots"/>
   /// array must contain at least two points.</exception>

      TPoints mKnots;
      TPoints mFirstCtrl;
      TPoints mSecondCtrl;

   public:

      BezierSpline(const TPoints &knots);

      void InterpBy(int interpBy, TPoints &interpOut)
         {
         interpOut.clear();

         interpOut.reserve(mKnots.size()*interpBy); //close enough
         for (int i(1),iEnd(mKnots.size()-1);i<iEnd;++i)
            {
            const auto &knot = mKnots[i];
            const auto &endKnot = mKnots[i+1];
            interpOut.push_back(knot);
            for (int j(1); j < interpBy; ++j)
               {
               double t = j/double(interpBy);
               auto x = BezierInterp(t, knot.mX, mFirstCtrl[i].mX, mSecondCtrl[i].mX, endKnot.mX);
               auto y = BezierInterp(t, knot.mY, mFirstCtrl[i].mY, mSecondCtrl[i].mY, endKnot.mY);
               interpOut.push_back({x,y});
               }
            }
         interpOut.push_back(mKnots.back());
         }

      static void GetCurveControlPoints(const TPoints &knots,
         TPoints & firstControlPoints, TPoints & secondControlPoints);


      static double BezierInterp(double t, double a, double b, double c, double d)
         {
         // here's an alternative to Michal's bezierInterpolation above.
         // the result is absolutely identical.
         // of course, you could calculate the four 'coefficients' only once for
         // both this and the slope calculation, if desired.
         double C1 = (d - (3.0 * c) + (3.0 * b) - a);
         double C2 = ((3.0 * c) - (6.0 * b) + (3.0 * a));
         double C3 = ((3.0 * b) - (3.0 * a));
         double C4 = (a);

         // it's now easy to calculate the point, using those coefficients:
         return (C1*t*t*t + C2 * t*t + C3 * t + C4);
         }

      static double BezierTangent(double t, double a, double b, double c, double d)
         {
         // note that abcd are aka x0 x1 x2 x3

     /*  the four coefficients ..
         A = x3 - 3 * x2 + 3 * x1 - x0
         B = 3 * x2 - 6 * x1 + 3 * x0
         C = 3 * x1 - 3 * x0
         D = x0

         and then...
         Vx = 3At2 + 2Bt + C         */

         // first calcuate what are usually know as the coeffients,
         // they are trivial based on the four control points:

         double C1 = (d - (3.0 * c) + (3.0 * b) - a);
         double C2 = ((3.0 * c) - (6.0 * b) + (3.0 * a));
         double C3 = ((3.0 * b) - (3.0 * a));
         double C4 = (a);  // (not needed for this calculation)

         // finally it is easy to calculate the slope element,
         // using those coefficients:

         return ((3.0 * C1 * t* t) + (2.0 * C2 * t) + C3);

         // note that this routine works for both the x and y side;
         // simply run this routine twice, once for x once for y
         // note that there are sometimes said to be 8 (not 4) coefficients,
         // these are simply the four for x and four for y,
         // calculated as above in each case.
         }

   /// <summary>
   /// Solves a tridiagonal system for one of coordinates (x or y)
   /// of first Bezier control points.
   /// </summary>
   /// <param name="rhs">Right hand side vector.</param>
   /// <returns>Solution vector.</returns>
   private:
      static void GetFirstControlPoints(const std::vector<double> &rhs, std::vector<double> &x);

   };
