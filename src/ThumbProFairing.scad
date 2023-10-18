
use <Naca_sweep.scad>
use <tools.scad>
use <naca4.scad>
include <Thumbpro.scad>


//"module airfoil(naca=2412, L = 100, N = 81, h = 1, open = false) - renders airfoil object\n", 
//  "module airfoil(naca=[.2, .4, .12], L = 100, N = 81, h = 1, open = false) - renders airfoil object using percentage for camber,  camber distance and thicknes\n", 
//  "function airfoil_data(naca=12, L = 100, N = 81, open = false)\n",


module Cover()
{
import("Hurricane MkII cover 2 LW-kits.3mf", convexity = 5);
}

profileLen = 100;


camber = 0;
height = thumbH;

frontScale = 0.7;
sideScale = 0.35;
fairingLen = thumbLen+2; //-8;

camAng = 20;
shimH = 15.5;

duoStripWidth = 43;//25.8+0.4;
duoStripLen = 11;
duoStripsThick = 5+0.3;


module Shim(shimH = 24)
{
intersection()
{
rotate([camAng,0,0])
translate([0,8-thumbLen,0])
difference()
{
linear_extrude(shimH)
   projection()
      rotate([0,90,0])
         Fairing();
translate([0,2,0])
   rotate([0,0,-90])
   translate([height/2-thumbLen,height/2,shimH])
   rotate([90,0,0])
   CamPlusDuoStrip();
}
translate([-100,-209,0])
   cube([200,200,100]);
}
}

Shim();
//CamPlusDuoStrip();

//FairingToPrint();
//FairingToSubtract();
//CamToSubtract();

module FairingToSubtract()
{
//difference()
   {
   //Cover();

   rotate([0,0,180])
   translate([-53,6.6,-5])
   rotate([-90,0,0])
   rotate([0,0,-90])
      FairingPlusCam();
   }
}

module CamToSubtract()
{
//difference()
   {
   //Cover();

   rotate([0,0,180])
   translate([-53,6.6,-5])
   rotate([-90,0,0])
   rotate([0,0,-90])
      {
      translate([0,thumbLen-8,-thumbH/2])
      rotate([0,0,-90])
         {
         ThumbPro();

         }
         translate([-duoStripsThick/2,duoStripWidth,0])
            duoStrip();
Cable();
      }
   }
}

module CamPlusDuoStrip()
{
ThumbPro();
rotate([0,0,90])
translate([-duoStripsThick/2,-2*duoStripWidth/3,height/2])
   duoStrip();

}


module duoStrip()
{
cube([duoStripsThick,duoStripWidth,thumbH-6],true);
}

module Cable()
{
translate([0,thumbLen-12,0])
   mirror([0,0,1])
   cylinder(h = 20, d = 7);
}

module FairingToPrint()
{
difference()
{
Fairing();

translate([0,thumbLen-8,-thumbH/2])
rotate([0,0,-90])
ThumbPro();
translate([-duoStripsThick/2,duoStripWidth-26,0])
   duoStrip();
Cable();
}
}

module FairingPlusCam()
{
//difference()
{
Fairing();

translate([0,thumbLen-8,-thumbH/2])
rotate([0,0,-90])
   ThumbPro();
//translate([-duoStripsThick/2,duoStripWidth,0])
//   duoStrip();

Cable();
}
}


module Fairing()
{
profile = airfoil_data(naca=[camber, 0, height/profileLen], L = profileLen, N = 100, open = false);

maxThickChord = 0.3;

module profilePoly()
{
intersection()
   {
   polygon(profile);
   square([2*maxThickChord*profileLen,2*maxThickChord*profileLen],true);
   }
}

scale([frontScale,sideScale,1])
   {
   rotate_extrude(angle=90, convexity=10,$fn = 64)  
      translate([-maxThickChord*profileLen,0,0])
      profilePoly();
   }

scale([frontScale,1,1])
   {
   translate([-maxThickChord*profileLen,0,0])
   rotate([-90,0,0])
   linear_extrude(height = fairingLen, center = false, convexity = 10)
      profilePoly();
   }
}

