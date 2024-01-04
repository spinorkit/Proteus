use <Naca_sweep.scad>
use <tools.scad>
use <naca4.scad>



TEExt = 3;
thick = 3.8;
//tipLen = 26;

rootLen = 68;
flapLen = 378;
midLen = 119;
aileronLen = 485;
tipLen = 20;//135;

rootWidth = 15;
tipWidth = 13;

eleLen = flapLen/3;
//eleLen = rootLen;

width = rootWidth+TEExt;


cutAng = 0;//90-55;

TELen = eleLen-width*tan(cutAng);

offsetY  = 20;

$fn = 128;

top = false;

rootProfile = [[0,0],[width, thick], [width, 0],[0,0]];
profileTip = rootProfile;

//EleExtX2();

CurvedTip();



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

function planShapeFunc(z) = 1;//pow(2-z,-6);
function planShape(z) = 1;//(planShapeFunc(z)-planShapeFunc(0))/(planShapeFunc(1)-planShapeFunc(0)); //normalise output to [0,1)


module CurvedTip()
{
maxAngle = 90;
function curveFn(z) = z;//pow(z,3);
function curve(z) = (curveFn(z)-curveFn(0))/(curveFn(1)-curveFn(0)); //normalise output to [0,1)
tipInc = 0.1;
function tipVecs() = 
   [
   for(i = [0: tipInc : 1])
      let (x =  0*planShape(i))
      let (xScale = 1)//1-planShape(i))
      let (yScale = xScale)
      let (angle = maxAngle*curve(i))
      let (y = width*(cos(angle))-width)
      let (h = tipLen*(sin(angle)))
      TransXYZ(x,y,h, Rx_(angle,vec3D( profileTip*[[xScale,0],[0,yScale]])))
   ];
vecs = tipVecs();
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