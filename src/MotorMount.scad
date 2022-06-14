include <motormountparams.scad>

motorMountTol = 0.1;
//
//motorTubeLen = 0.6*rootChord;
//motorTubeX = rootChord-motorTECutout-motorTubeLen;
//taperFracOD = 0.68;
//nacelleLen = motorTubeOD;
//nacellePosAdj = 0.77*nacelleLen;
//motorMountBoltD = 3;
//motorMountBoltZFrac = 0.6;
motorMountBoltZ = motorMountBoltZFrac*nacellePosAdj;
//rimH = 3+0;
motorBearingD = 9;
motorBearingH = 2;
boltHeadD = 6;

od = motorTubeOD-2*motorTubeThick-motorMountTol; //was 42
strutW = 6;//9.9;
innerSquare = 10;//19.5;
id = 5;//8.8;
rimThick = 2.5;
strutThick = 8;//+3.2+0.01;
thick = nacellePosAdj;//12.5;
height = thick+rimH-3;
fillet = 2;//3;

holeDia = 3.2+0.25;//3.18+0.4;
holeDiag = 16;

$fn = 64;

Reinforcer(motorTubeLen);
//cutouts();
//AddMotorMount(motorTubeLen = 0.6*200,motorTubeX);

module Reinforcer(motorTubeLen = 0.6*200)
{
//      translate([0,0,-1*extraRimH])
//         cylinder(d=motorTubeOD,h=extraRimH);


intersection()
   {
   union()
      {
      translate([0,0,rimH])
         cylinder(d2=taperFracOD*motorTubeOD-2*motorTubeThick, d1=motorTubeOD-2*motorTubeThick,h=motorTubeLen);
      cylinder(d=motorTubeOD,h=rimH);
      }
   difference()
      {
      union()
         {
         difference()
            {
            union()
               {
               cylinder(d=motorTubeOD,h=rimH);
               cylinder(d=od,h=height);
               }
            cylinder(d=od-2*rimThick,h=height);
            }
         difference()
            {
            union()
               {
               cylinder(d=od,h=strutThick);
               }
            cutouts();
            //cylinder(d=id,h=strutThick);
            }
         }
         //translate([0,0,motorMountBoltZ+rimH]) rotate([90,0,45])
         //   #cylinder(d=motorMountBoltD,h=motorTubeOD, center=true);
         
         //translate([motorTubeX+motorTubeLen-motorMountBoltZFrac*nacellePosAdj,0,0])
         translate([0,0,motorMountBoltZ+rimH]) rotate([0,90,0])
            {
            rotate([motorBoltAngle,0,0])
            rotate([90,0,0])
            #cylinder(d = motorBoltThreadD, h=0.7*motorTubeOD,center = false);
            rotate([-motorBoltAngle,0,0]) 
            rotate([90,0,0])
            #cylinder(d = motorBoltThreadD, h=0.7*motorTubeOD,center = false);
            }
         //Air and cable gap
         cableGapD = 10;
         translate([0,rimThick-(cableGapD+motorTubeOD)/2,0])
            cylinder(d = cableGapD,h=motorTubeLen);
         //Motor bearing space
            cylinder(d = motorBearingD,h=motorBearingH);
      //Bolt head clearance
      rotate([0,0,-135])
         translate([holeDiag/2,0,strutThick])
            #cylinder(d = boltHeadD,h=strutThick);
      rotate([0,0,45])
         translate([holeDiag/2,0,strutThick])
            #cylinder(d = boltHeadD,h=strutThick);
      rotate([0,0,135])
         translate([holeDiag/2,0,strutThick])
            #cylinder(d = boltHeadD,h=strutThick);
      rotate([0,0,-45])
         translate([holeDiag/2,0,strutThick])
            #cylinder(d = boltHeadD,h=strutThick);
      }
   }
}

module cutouts()
{
linear_extrude(strutThick)
   {
      minkowski()
         {
         difference()
            {
            circle(d=od-2*rimThick-fillet);
            //translate([0,0,strutThick/2-0.001])
            union()
               {
               square(innerSquare+fillet,true);
               rotate([0,0,45])
                  square([strutW+fillet,od+rimThick],true);
               rotate([0,0,-45])
                  square([strutW+fillet,od+rimThick],true);
               }
            }
         circle(d=fillet);
         }
   
   rotate([0,0,-45])
      translate([holeDiag/2,0,0])
         circle(d= holeDia);
   rotate([0,0,45])
      translate([holeDiag/2,0,0])
         circle(d= holeDia);
   rotate([0,0,135])
      translate([holeDiag/2,0,0])
         circle(d= holeDia);
   rotate([0,0,-135])
      translate([holeDiag/2,0,0])
         circle(d= holeDia);
   }
}

//module AddMotorMount(motorTubeLen, motorTubeX, motorTECutout, motor=true)
//{
//difference()
//   {
////   motorTubeLen = 0.6*rootChord;
////   motorTubeX = rootChord-motorTECutout-motorTubeLen;
////   taperFracOD = 0.68;
////   nacelleLen = motorTubeOD;
////   nacellePosAdj = 0.77*nacelleLen;
////   motorBoltD = 3.2;
////   motorBoltHeadD = 5.2;
//   union()
//      {
//      children();
//      if(motor)
//         {
//         translate([motorTubeX,0,0])
//            {
//            translate([motorTubeLen-nacellePosAdj,-0.4*motorTubeOD,0])
//               scale([1,0.2,3])rotate([0,90,-90])
//                  GuardNacelle(nacelleLen);
//            translate([motorTubeLen-nacellePosAdj,0.4*motorTubeOD,0])
//               scale([1,0.2,3])rotate([0,90,-90])
//                  GuardNacelle(nacelleLen);
//         rotate([0,90,0])
//            cylinder(d1=taperFracOD*motorTubeOD, d2=motorTubeOD,h=motorTubeLen);
//            translate([motorTubeLen-escLeadLen+13-1.5*escLen,-escHeight/2,-escWidth/2])
//            cube([1.5*escLen,escHeight,escWidth]);
//            }
//         }
//      }
//   if(motor)
//      {
//      translate([motorTubeX,0,0])
//      rotate([0,90,0])
//         {
//         cylinder(d1=taperFracOD*motorTubeOD-motorTubeThick, d2=motorTubeOD-motorTubeThick,h=motorTubeLen);
//         }
//         translate([motorTubeX+motorTubeLen,0,0])rotate([0,90,0])
//            cylinder(d=motorTubeOD, h=motorTubeLen/2);
//      translate([motorTubeX+motorTubeLen-motorMountBoltZ,0,0])
//            {
//            rotate([90,0,0])
//            cylinder(d = motorBoltD, h=1.5*motorTubeOD,center = true);
//            translate([0,0.625*motorTubeOD,0]) 
//            rotate([90,0,0])
//            #cylinder(d = motorBoltHeadD, h=0.25*motorTubeOD,center = true);
//            translate([0,-0.625*motorTubeOD,0]) 
//            rotate([90,0,0])
//            #cylinder(d = motorBoltHeadD, h=0.25*motorTubeOD,center = true);               
//            }
//      }
//   }
//   
//}
