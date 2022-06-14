#include "stdafx.h"

#include "Bezier.h"

//based on: https://www.codeproject.com/Articles/31859/Draw-a-Smooth-Curve-through-a-Set-of-2D-Points-wit


BezierSpline::BezierSpline(const TPoints &knots):
   mKnots(knots)
   {
   GetCurveControlPoints(mKnots, mFirstCtrl, mSecondCtrl);
   }



void BezierSpline::GetCurveControlPoints(const TPoints &knots,
   TPoints & firstControlPoints, TPoints & secondControlPoints)
   {
   int n = knots.size() - 1;
   if (n < 1)
      throw std::exception("At least two knot points required");

   if (n == 1)
      { // Special case: Bezier curve should be a straight line.
      firstControlPoints.resize(1);
      // 3P1 = 2P0 + P3
      firstControlPoints[0].mX = (2 * knots[0].mX + knots[1].mX) / 3;
      firstControlPoints[0].mY = (2 * knots[0].mY + knots[1].mY) / 3;

      secondControlPoints.resize(1);
      // P2 = 2P1 – P0
      secondControlPoints[0].mX = 2 *
         firstControlPoints[0].mX - knots[0].mX;
      secondControlPoints[0].mY = 2 *
         firstControlPoints[0].mY - knots[0].mY;
      return;
      }

   // Calculate first Bezier control points
   // Right hand side vector
   std::vector<double> rhs(n);

   // Set right hand side X values
   for (int i = 1; i < n - 1; ++i)
      rhs[i] = 4 * knots[i].mX + 2 * knots[i + 1].mX;

   rhs[0] = knots[0].mX + 2 * knots[1].mX;
   rhs[n - 1] = (8 * knots[n - 1].mX + knots[n].mX) / 2.0;
   // Get first control points X-values
   std::vector<double> x;
   GetFirstControlPoints(rhs, x);

   // Set right hand side Y values
   for (int i = 1; i < n - 1; ++i)
      rhs[i] = 4 * knots[i].mY + 2 * knots[i + 1].mY;
   rhs[0] = knots[0].mY + 2 * knots[1].mY;
   rhs[n - 1] = (8 * knots[n - 1].mY + knots[n].mY) / 2.0;
   // Get first control points Y-values
   std::vector<double> y;
   GetFirstControlPoints(rhs, y);

   // Fill output arrays.
   firstControlPoints.resize(n);
   secondControlPoints.resize(n);
   for (int i = 0; i < n; ++i)
      {
      // First control point
      firstControlPoints[i].mX = x[i];
      firstControlPoints[i].mY = y[i];
      // Second control point
      if (i < n - 1)
         secondControlPoints[i] = Point(2 * knots
            [i + 1].mX - x[i + 1], 2 *
            knots[i + 1].mY - y[i + 1]);
      else
         secondControlPoints[i] = Point((knots
            [n].mX + x[n - 1]) / 2,
            (knots[n].mY + y[n - 1]) / 2);
      }
   }

void BezierSpline::GetFirstControlPoints(const std::vector<double> &rhs, std::vector<double> &x)
   {
   int n = rhs.size();
   x.resize(n); // Solution vector.
   std::vector<double> tmp(n); // Temp workspace.

   double b = 2.0;
   x[0] = rhs[0] / b;
   for (int i = 1; i < n; i++) // Decomposition and forward substitution.
      {
      tmp[i] = 1 / b;
      b = (i < n - 1 ? 4.0 : 3.5) - tmp[i];
      x[i] = (rhs[i] - x[i - 1]) / b;
      }
   for (int i = 1; i < n; i++)
      x[n - i - 1] -= tmp[n - i] * x[n - i]; // Backsubstitution.
   }
