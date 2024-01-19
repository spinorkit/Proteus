use <Naca_sweep.scad>
use <tools.scad>
use <naca4.scad>



thick = 3.8;
//tipLen = 26;

rootLen = 68;
flapLen = 378;
midLen = 119;
aileronLen = 485;
aileronTaperLen = 240;
tipLen = 20;//135;

tipRadius = 31+1.5; //I think!

extraWidthForPrinting = 3;
rootWidth = 15+extraWidthForPrinting;
tipWidth = 11+extraWidthForPrinting;

eleLen = flapLen/3;
//eleLen = rootLen;

//width = rootWidth+TEExt;


cutAng = 0;//90-55;

//TELen = eleLen-width*tan(cutAng);

offsetY  = 20;

TEExt = 3; //printing support

$fn = 128;

top = false;

profileRoot = [[0,0],[0, thick], [rootWidth, 0],[0,0]];
profileTip = [[0,0],[0, thick], [tipWidth, 0],[0,0]];

//EleExtX2();

//CurvedTipExt();

//AileronTapered();

AileronExt();



//Cut(){EleExtX2();}

//module EleExtX2()
//{
//translate([width,0,0])
//   EleExt();
//translate([-width,offsetY,0])
//rotate([0,0,180])
//   EleExt();
//cube([0.6,offsetY,TELen]);
//}

module EleExtX2()
{
translate([0,0,0])
   EleExt();
translate([0,offsetY,0])
rotate([0,0,180])
   EleExt();
translate([-0.6,0,0])
   cube([0.6,offsetY/2+0.6,eleLen]);
translate([0,offsetY/2,0])
   cube([0.6,offsetY/2,eleLen]);

   union()
      {
      children();
      translate([-width+TEExt, 0,top?eleLen/2:0])
         cylinder(h=0.6,r=thick);

      translate([0,offsetY,0])
      translate([width-TEExt, 0,top?eleLen/2:0])
         cylinder(h=0.6,r=thick);
      }

}

//TipExt();

//function planShapeFunc(z) = 1;//pow(2-z,-6);
//function planShape(z) = 1;//(planShapeFunc(z)-planShapeFunc(0))/(planShapeFunc(1)-planShapeFunc(0)); //normalise output to [0,1)


module CurvedTipExt()
{
maxAngle = 90;
function planShapeFunc(z) = pow(z,3);
function planShape(z) = (planShapeFunc(z)-planShapeFunc(0))/(planShapeFunc(1)-planShapeFunc(0)); //normalise output to [0,1)

function curveFn(z) = z;//pow(z,3);
function curve(z) = (curveFn(z)-curveFn(0))/(curveFn(1)-curveFn(0)); //normalise output to [0,1)
tipInc = 0.05;
function tipVecs() = 
   [
   for(i = [0: tipInc : 1])
      let (x =  0) //0*planShape(i))
      let (xScale = 1-planShape(i))
      let (yScale = xScale)
      let (angle = maxAngle*curve(i))
      let (y = -tipRadius*(cos(angle))-0*tipRadius)
      let (h = tipRadius*(sin(angle)))
      TransXYZ(x,y,h, Rx_(-angle,vec3D( profileTip*[[xScale,0],[0,yScale]])))
   ];
vecs = tipVecs();
//echo(vecs);
poly3dFromVectors(vecs);

}

module AileronTapered(span = aileronTaperLen)
{
function vecsFunc(span) = 
   let (profileRoot3D = vec3D( profileRoot))
   let (profileTip3D = vec3D( profileTip))
   catPoly3D( 
      profileRoot3D,
      [[for (v = profileTip3D) v+[0,0,span]]]
      //[for (v = profileRoot3D) v+[0,0,span]],
      //TransXYZ(0,0,-span, profileRoot3D),
      //catPoly( TransXYZ(0,0,-span/2, vec3D( profileRoot)),
      //catPoly3D( [for (v = profileRoot3D) v+[0,0,-span/2]],//M*vec3D( profileRoot),
      //translatePoly3D([0,0,0], profileTip3D)
   );
vecs = vecsFunc(span);
echo(vecs);
poly3dFromVectors(vecs);
}

module AileronExt(span = 180)
{
function vecsFunc(span) = 
   let (profileRoot3D = vec3D( profileRoot))
   let (profileTip3D = vec3D( profileRoot))
   catPoly3D( 
      profileRoot3D,
      [[for (v = profileTip3D) v+[0,0,span]]]
      //[for (v = profileRoot3D) v+[0,0,span]],
      //TransXYZ(0,0,-span, profileRoot3D),
      //catPoly( TransXYZ(0,0,-span/2, vec3D( profileRoot)),
      //catPoly3D( [for (v = profileRoot3D) v+[0,0,-span/2]],//M*vec3D( profileRoot),
      //translatePoly3D([0,0,0], profileTip3D)
   );
vecs = vecsFunc(span);
echo(vecs);
poly3dFromVectors(vecs);

}



//EleExtCut();

module Cut()
{
intersection()
   {
   if(top)
      translate([-eleLen/2,-eleLen/2,eleLen/2]) cube([eleLen, eleLen, eleLen/2]);
   else
      translate([-eleLen/2,-eleLen/2,0]) cube([eleLen, eleLen, eleLen/2]);
   union()
      {
      children();
      translate([-width+TEExt, 0,top?eleLen/2:0])
         cylinder(h=0.6,r=thick);

      translate([0,offsetY,0])
      translate([width-TEExt, 0,top?eleLen/2:0])
         cylinder(h=0.6,r=thick);
      }
   }
}

module EleExtCut()
{
rotate([0,0,-90])  
intersection()
   {
   if(top)
      translate([-eleLen/2,-eleLen/2,eleLen/2]) cube([eleLen, eleLen, eleLen/2]);
   else
      translate([-eleLen/2,-eleLen/2,0]) cube([eleLen, eleLen, eleLen/2]);
   union()
      {
      EleExt();
      translate([-width+TEExt, 0,top?eleLen/2:0])
         cylinder(h=0.6,r=thick);
      }
   }
}

module TipExt()
{
rotate([0,0,-90])
   {
   Extension(tipLen);
   translate([TEExt, 0,0])
      cylinder(h=0.6,r=thick);
   }
}

module Extension(tipLen)
{
linear_extrude(height = tipLen)
   polygon(rootProfile);

}

module EleExt()
{
difference()
   {
   translate([-width,0,0])
      Extension(eleLen);
   translate([-width,-2*thick,TELen])
      rotate([0,-cutAng,0])
      cube([2*width, eleLen, eleLen]);
   }

}