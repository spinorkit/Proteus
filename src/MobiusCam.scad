//61mm (L) x 35mm (W) x18mm (H)
camLen = 60;
camWidth = 36;
camH = 19;
lensRimOD = 19;
lensRimLen = 2.5;
lensID = 8+2+4;
cameraDrop = 1.0; //due to weight of camera
lensViewAngle = 100;

extrusionWidth = 0.46;
holderThick = 2*extrusionWidth;
vibSpace = 3.2;
vibSpaceBackExt = 1;
supportRaise = 0.8;
supportThick = extrusionWidth+0.01; //Add 0.01 to prevent S3D doing lots of retractions
supportCamGap = 0.25;
supportH = camH+2*(holderThick+vibSpace);
supportLen = 0.7*camLen;
supportLenFractionRigid = 0.5;
//supportW = 0.75*vibSpace-2*supportThick;
supportW = 0.75*vibSpace-2*supportThick;

duoStripT = 3.75;
duoStripW = 26;
duoStripL = 48;
duoVibSpaceT = 1.2;

coolingD = 23;
coolingFromBack = 19;
coolingFromSide = 22;

buttonD = 7;
buttonH = 25;
buttonsFromSide = 8;
but1FromBack = 30;
but2FromBack = 39;
but3FromBack = 48;

ledFromBack = 38;
ledFromSide = 28.5;

flexiFil = true;

//MobiusCam();
//MobiusHolder();
//%VibSpace(false);
//LeftRightSupport(flexiFil);
//SineSupport(supportLen, supportH, supportW);

testHolder();

module testHolder(flexiFiliament = flexiFil)
{
supportLenToUse = flexiFiliament?supportLen:supportLenFractionRigid*supportLen;

offset = camH/2;
intersection()
   {
   union()
      {
      MobiusHolder();
      LeftRightSupport(flexiFiliament);
      difference()
         {
         translate([0,0,offset])
         scale([1,1.1,1.1])
            translate([0,0,-offset])
            VibSpace(true);
         VibSpace(true);
         }
      }
   translate([(supportLenToUse-camLen)/2,-camWidth,-2*camH])
      #cube([camLen,2*camWidth, 4*camH]);
   }
}

//MobiusCamBody();

module AddCamera(transVec, rotVec, camera = true, holder = true, flexiFiliament = false)
{
if(!camera)
   children();
else
   {
   difference()
      {
      children();
      //Subtract these 2
      translate(transVec)
         rotate(rotVec)
         {
         #VibSpace(!holder); //make not solid if we want the camera holder
         #MobiusCam();
         }
      }
   //Add these 
   if(holder)
   translate(transVec)
      rotate(rotVec)
      {
      MobiusHolder();
      LeftRightSupport(flexiFiliament);
      }
   }
}

module LeftRightSupport(flexiFiliament)
{
RightSupport(flexiFiliament);
mirror([0,1,0])
   RightSupport(flexiFiliament);
}

module RightSupport(flexiFiliament)
{
supportLenToUse = flexiFiliament?supportLen:supportLenFractionRigid*supportLen;
//if(flexiFiliament)
translate([(camLen-supportLenToUse)/2,camWidth/2+holderThick+supportCamGap,supportRaise-(vibSpace+holderThick)])
   //Support(supportLenToUse);
   SineSupport(supportLenToUse, supportH, supportW);
//else
//{
//translate([(camLen-supportLen)/2,camWidth/2+holderThick+supportCamGap,supportRaise-(vibSpace+holderThick)])
//   {
//   Support(supportLenFractionRigid*supportLen);
//   translate([supportLen-supportLenFractionRigid*supportLen,0,0])
//      Support(supportLenFractionRigid*supportLen);
//   }
//}
}

module MobiusCam()
{
translate([0,-camWidth/2,0])
   {
   //cube([camLen,camWidth,camH]);
   MobiusCamBody();
   translate([0,camWidth-13.5,camH/2])
   rotate([0,-90,0])
      {
      cylinder(d=lensRimOD, h = lensRimLen);
      translate([-cameraDrop,0,0])
      scale([9/16,1,1])
      //cylinder(d1=lensID,d2 = lensID + 2*25*tan(lensViewAngle/2), h = 25);
         hull()
         {
            linear_extrude(height=35,scale = (lensID + 2*25*tan(lensViewAngle/2))/lensID)square(lensID,true);
         }
      }
   }
}

module MobiusCamBody()
{
coolingH = 2.5;
difference()
{
union()
{
cube([camLen,camWidth,camH]);
translate([camLen-duoStripL,(camWidth-duoStripW)/2,camH])
   cube([duoStripL,duoStripW,duoStripT]);
//Cooling hole
translate([camLen-coolingFromBack,coolingFromSide,-coolingH])
   scale([1.1,1,1])cylinder(d=coolingD, h = coolingH);
//Button holes
translate([camLen-but1FromBack,buttonsFromSide,-buttonH])
   cylinder(d=buttonD, h = buttonH);
translate([camLen-but2FromBack,buttonsFromSide,-buttonH])
   cylinder(d=buttonD, h = buttonH);
translate([camLen-but3FromBack,buttonsFromSide,-buttonH])
   cylinder(d=buttonD, h = buttonH);
//LED hole
translate([camLen-ledFromBack,ledFromSide,-buttonH])
   cylinder(d=buttonD, h = buttonH);
}
translate([camLen-1,0,0*camH])
   cube([1,camWidth,0.15*camH]);
}
}


module MobiusHolder(hollow = true)
{
thick = holderThick;
width = camWidth+2*thick;
duoWidth = duoStripW+2*thick;
height = camH+2*thick;
len = camLen; 

   {
   difference()
      {
      union()
         {
         translate([0,-width/2,-thick])
            cube([len,width,height]);
         translate([camLen-duoStripL-thick,-(duoWidth)/2,camH])
            cube([duoStripL+thick,duoWidth,duoStripT+thick]);
         }
      if(hollow)
         translate([0,-camWidth/2,0])
            MobiusCamBody();
      }
   }
}

module VibSpace(solid = false)
{
width = camWidth+2*(holderThick+vibSpace);
height = camH+2*(holderThick+vibSpace);
len = camLen+2*vibSpace+vibSpaceBackExt; 
   
duoWidth = duoStripW+2*holderThick+2*duoVibSpaceT;
duoLen = duoStripL+holderThick+duoVibSpaceT+vibSpace+vibSpaceBackExt;
duoT = duoStripT+holderThick+duoVibSpaceT;
   {
   difference()
      {
intersection()
            {
   translate([len/2,0,(height)/2-holderThick-vibSpace])
      scale([1.2,0.7,0.5]) sphere(d = vibSpace+1.125*sqrt(pow(len,2)+pow(width,2)));
      union()
         {
         translate([-vibSpace,-width/2,-holderThick-vibSpace])
            cube([len,width,height]);
         translate([camLen-duoStripL-holderThick-duoVibSpaceT,-(duoWidth)/2,camH])
            cube([duoLen,duoWidth,duoT]);
         }
      }
     // translate([0,-camWidth/2,0])
      if(!solid) 
         {
         MobiusHolder(false);
         MobiusCam();
         }
      }
   }

}

module Shear(ang)
{

multmatrix(m = [ [1, 0, 0, 0],
                 [0, 1, tan(ang), 0],                        
                 [0, 0, 1, 0],
                 [0, 0, 0,  1]
  ])
   children();
}



module ngon(num, r) {
  polygon([for (i=[0:num-1], a=i*360/num) [ r*cos(a), r*sin(a) ]]);
}

//ngon(5, 10);
// More complex list comprehension:
// Similar to ngon(), but uses an inner function to calculate
// the vertices. the let() keyword allows assignment of temporary variables.
module rounded_ngon(num, r, rounding = 0) {
  function v(a) = let (d = 360/num, v = floor((a+d/2)/d)*d) (r-rounding) * [cos(v), sin(v)];
  polygon([for (a=[0:360-1]) v(a) + rounding*[cos(a),sin(a)]]);
}

//SineSupport(supportLen, supportH, supportW);

//Support(supportLen);

module SineSupport(supportLen, height, width, thick = supportThick)
{
nCycles = 9;
waveLen = height/nCycles;
tMax = 2*(nCycles+1)*360;
tRev = tMax/2;
phase = 90;
inThick = thick/50;
amp = (width-thick)/2;

translate([0,0,-waveLen/2])
   {
translate([0,amp+2*thick,0])
   {
mirror([0,1,0])
rotate([0,-90,-180])
linear_extrude(height = supportLen)  
offset(r = thick/2)
  polygon(
   [for (t=[0:12:tMax-1]) [(t<tRev?t:tMax-t)*waveLen/360,(t<tRev?amp*sin(t+phase)+inThick/2:-amp*sin(t-phase)-inThick/2)]]);

//    [for (t=[nCycles*360:-12:1]) [t*waveLen/360,0.5+width*sin(t)]
}
  
translate([0,1*supportThick-supportW,height/2-0.5*supportThick])
      cube([supportLen,supportW+supportThick,supportThick]);
}
   
}


module Support(supportLen)
{
ang = 45;
sections = floor(supportH/(supportW*tan(ang)));
sectionH = supportH/sections;
   
//bottom joiner
//translate([0,supportThick,-supportThick])
//   cube([supportLen,supportThick,supportThick]);

//Zig-zags
translate([0,supportW+supportThick,(-1)*sectionH])
   Shear(-ang)
      cube([supportLen,supportThick/cos(ang),sectionH]);
   
translate([0,supportThick,0])
   {
   for(i=[0:2:sections-1])
      {
      translate([0,0,i*sectionH])
         Shear(ang)
            cube([supportLen,supportThick/cos(ang),sectionH]);
      translate([0,supportW,(i+1)*sectionH])
         Shear(-ang)
            cube([supportLen,supportThick/cos(ang),sectionH]);
      }
//   translate([0,0,supportH-sectionH])
//   Shear(ang)
//   cube([supportLen,supportThick/cos(ang),sectionH]);
      
//   translate([0,supportThick-supportW,0])
//      cube([supportLen,supportW,supportThick]);
//   translate([0,supportW,supportH-supportThick])
//      cube([supportLen,supportW,supportThick]);
      
   translate([0,0*supportThick-supportW,floor((sections-1)/2)*sectionH-supportThick])
      cube([supportLen,supportW+supportThick,supportThick]);
   }
}