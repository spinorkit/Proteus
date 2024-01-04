include <PW51.scad>
include <PWRESmain.scad> //Airfoil
use <Naca_sweep.scad>
use <tools.scad>
use <naca4.scad>

NACACamber = 0;

ASW22OrigSpan = 440;
ASW22OrigChord = 92.5;
ASW22OrigTipChord = 80;
ASW22OrigThickAbs = 16.5;

chordExtraForPrinting = 5;

scaleFromOrig = 1.25;
span = scaleFromOrig*ASW22OrigSpan;
chord = scaleFromOrig*ASW22OrigChord+chordExtraForPrinting;
thickAbs = ASW22OrigThickAbs;


thick = thickAbs/chord;
echo("thick=",thick); 


tipLen = 0.1*span;
halfSpan = span/2-tipLen;

sweep = 0;
sweepExtraForPrinting = 0;

nFoilPoints = 100;

function FoilNACA(chord = 100,N = 120/*len(airfoil)*/, thick = 0.08) = 
   TransXYZ(-chord,0,0,airfoil_data([-NACACamber,0.33,thick], chord, N, false));

//translate([0,0,span/2]) mirror([0,0,1])TailPlane();

difference()
   {
   TailPlane();
   translate([0,0,-100])
      cube([100,100,100],true);
   }

//polygon(100*profileRoot);
  

xScaleTip = 0.8;
yScaleTip = 0.9;

tipScale = [[xScaleTip,0],[0,yScaleTip]];

//profileRoot = 100*PWRESmainAligned;
profileRoot = FoilNACA(chord,nFoilPoints, thick);  
profileTip = profileRoot*tipScale;     

function planShapeFunc(z) = pow(2-z,-6);
function planShape(z) = (planShapeFunc(z)-planShapeFunc(0))/(planShapeFunc(1)-planShapeFunc(0)); //normalise output to [0,1)

//Interpolate between 2 profiles using the shape function
//function interp(hz) = 
//   [
//   for (i=[0: inc : 1+inc])
//      let(f = shape(i))
//      let(h = i>1?-(hz+finHeight*cos(NACAFinAngle)):-i*(hz))
//      let(x = shapex(i)*interpLen-0*h*tan(finSweep)+ (i>1? finBaseChord*(1-finTopChordFrac):0) )
//      //let(x = shapex(i)*interpLen-0*h*tan(finSweep)+ (i>1? 0*finBaseChord*(1-finTopChordFrac):0) )
//      let(y = i>1? finRaise+finHeight*sin(NACAFinAngle):f*finRaise)
//         //We use the last i value (i == i+inc) to generate the main, non-curved section of the fin 
//         TransXYZ(x,y,h, Rx_(i*NACAFinAngle,vec3D(i>1?finTopChordFrac*profile2:vecInterp(profile1,profile2,shapex(i)),0)))
//   ];

tipFrac = tipLen/halfSpan;

tipInc = 0.05;

//function tipVecs(hz) = 
//   [
//   for(i = [0: tipInc : 1 + tipInc])
//      let(h = i > 1 ? hz : i*tipLen)
//      let(x = i > 1 ? 0 : planShape(i))
//      let (xScale = i > 1 ? 1 : planShape(i))
//      let(yScale = xScale)
//      let(y = 0)
//         TransXYZ(x,y,h, vec3D(i > 1 ? profileRoot : profileTip*[[xScale,0],[0,yScale]]))
//   ];

// input : nested list
// output : list with the outer level nesting removed
function flatten(l) = [ for (a = l) for (b = a) b ] ;

function flattenVec3(l) = 
[ 
//for (a = l) for (b = a) b //loses profile []
//for (a = l) each a //loses profile []
for (a = l) [each a] //extra outer []
] ;

function tipVecs(tipLen) = 
   [
   //TransXYZ(0,0,-span/2, vec3D( profileRoot)),
   //TransXYZ(0,0,0, vec3D( profileTip)),
   for(i = [0: tipInc : 1])
      let (h = i*tipLen)
      let (x =  planShape(i))
      let (xScale = 1-planShape(i))
      let (yScale = xScale)
      let (y = 0)
      TransXYZ(x,y,h, vec3D( profileTip*[[xScale,0],[0,yScale]]))
   ];

//function wingVecs(span) = 
//   let (profileRoot3D = vec3D( profileRoot))
//   catPoly3D( [for (v = profileRoot3D) v+[0,0,-span]],
//      //TransXYZ(0,0,-span, profileRoot3D),
//      //catPoly( TransXYZ(0,0,-span/2, vec3D( profileRoot)),
//      catPoly3D( [for (v = profileRoot3D) v+[0,0,-span/2]],//M*vec3D( profileRoot),
//      translatePoly3D([0,0,0], tipVecs(tipLen)))
//   );


function wingVecs(span) = 
   let (profileRoot3D = vec3D( profileRoot))
   catPoly3D( [for (v = profileRoot3D) v+[0,0,-span/2]],
      //TransXYZ(0,0,-span, profileRoot3D),
      //catPoly( TransXYZ(0,0,-span/2, vec3D( profileRoot)),
      //catPoly3D( [for (v = profileRoot3D) v+[0,0,-span/2]],//M*vec3D( profileRoot),
      translatePoly3D([0,0,0], tipVecs(tipLen))
   );

//function wingVecs(span) = 
//   concat(TransXYZ(0,0,-span/2, vec3D( profileRoot)),TransXYZ(0,0,0, vec3D( profileTip)));


module TailPlane()
{

//blendedVecs = [[TransXYZ(0,0,-span/2, vec3D( profileRoot))], [tipVecs(span/2)]];
//blendedVecs = [[TransXYZ(0,0,-span/2, vec3D( profileRoot))],[tipVecs(span/2)]];
//wingVs = rotatePoly3D([0, 0, 0 ], wingVecs(span));

dihedral = 10;

shears = [[1,0,1*tan(sweep+sweepExtraForPrinting)],[0,1,tan(dihedral)],[0,0,1]];

wingVs = multPoly3D(shears, scalePoly3D([1,1,1],wingVecs(span)));
//echo(wingVs);
rotate([0,0,0])
//      multmatrix(m = [ [1,0 , -1*tan(sweep+sweepExtraForPrinting), 0],
//                     [0, 1,0 , 0],
//                     [0, 0, 1, 0],
//                     [0, 0, 0,  1]
//                              ])
         {
         poly3dFromVectors(wingVs);
         }
}
