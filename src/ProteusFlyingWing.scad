//Proteus Flying Wing
//Copyright: Kit Adams, Dunedin, New Zealand
// October 2016
// commercial use prohibited
//Uses:
// Naca4_sweep.scad - sweep library
// Code: Rudolf Huttary, Berlin 
// June 2015
// commercial use prohibited

//To do:
//Increase flap/elevon joiner dia slightly - fixed
//Rotate wing joiners parallel to spar - fixed
//Antenna hole collides with fuse joiner - fixed
//increase length of spar box by 2mm - fixed

use <Naca_sweep.scad>
use <tools.scad>
use <naca4.scad>

//include <MH_45.scad> //Airfoil
include <PW51.scad> //Airfoil
/* [Hidden] */
thickPercent = PW51ThickPercent;//9.85;   //should calculate this from the airfoil!
//thickPercent = MH_45ThickPercent;//9.85;   //should calculate this from the airfoil!

use <mobiuscam.scad>

include <motormountparams.scad> //defines rootChord!

//airfoil = rootChord*MH_45;
airfoil = rootChord*PW51;
airfoilAligned = rootChord*PW51Aligned; //Top and bottom points at have same x positions


motor = false;
skid = true;
camera = false;
forwardSkid = camera || motor;
flexiFilament = true; //Using flexible filament for the Mobius Cam supports (and the nose).
bottomHinges = true;
elevonHingePos = bottomHinges ? -1 : 1; //0 => both, 1 => top, -1 => bottom 

hingeInboardEndGap = 13;
hingeTipEndGap = 12;
hingeW = 15;
hingeLen = 30;

/* [Global] */
//[ShellOnly,FuselageRear,FuselageNose,FuselageHatch,RootWing,MidWing,TipWing,Fin,
//Elevon,Flap,ServoGuard,ServoGuardFlaps,SparJoiner, Hinges]
partToGenerate = "ShellOnly"; //For testing use ""

midSectionIndex = 0; //Set this to 0 for the middle of the 3 piece wing half

/* [Printer] */
printerAbsMaxZ = 200; //Actual printer  height limit
printerMaxZ = scaleFactor*3*230/4; //max wing section height
echo("printerMaxZ",printerMaxZ);
nSections = 4;
//printerMaxZ = 200;
printerMaxX = 200;
printerMaxY = 200;
printerMaxXYLen = 0.85*sqrt(printerMaxX*printerMaxX+printerMaxY*printerMaxY);

fuselage =  partToGenerate == "FuselageRear" ||
            partToGenerate == "FuselageNose" ||
            partToGenerate == "FuselageHatch";

/* [Wing] */
rootThick = (thickPercent*rootChord)/100;
tipChord = 0.65*rootChord;//0.65*rootChord;
tipScale = tipChord/rootChord;
tipThick = tipScale*rootThick;
washout = 1.5; //3.5;
//sweepAtQuarterChord = 24;
sweep = 22;//26;//22; //This is leading edge sweep
dihedral = 0.5;

tubeSpar = false; //false means using a swept spar
rearTubeSpar = true;
maxSparLenO2 = 500+2+2;
maxTubeSparLen = 505;

flaps = true;
maxSparLenO2Prj = tubeSpar?maxSparLenO2:maxSparLenO2*cos(sweep);

/* [Fin] */
flatPlateTip = false;
NACAFinCamber = 0;//0.009;   //+ve to generate inwards and upwards lift
tipSectionCutAngle = flatPlateTip ? 0 : 45; //Make the NACA foil blended into wing tip FDM printable in one piece
NACAFinAngle = 70;  //For NACA profile fins only, from horizontal
NACAFinThick = 0.08;
NACAFinAspectRatio = 1.2;//1.4

finAngle = 4; //For flat plate fins only, from vertical
finSpan = scaleFactor*20;         //Span used to blend tip foil into fin

CGPercent = 18;//20; //percentc
CGDimples = true;
CGDimpleDia = 3;
CGDimpleDepth = 10;


/* [Fuselage] */

bodyWidth = 120;
bodyH = 50+2;
noseLen = scaleFactor * 100+((motor || camera)?5:0);//90 + (camera?10:0) + (motor?10:0) + 0; //ProteusPlusv1: 100
noseFlatness = (motor || camera) ? 1.7+0.2+0.2: 1.7;

cameraPos = [16+8-noseLen,-6,0];
cameraAngle = [-90,0,6];

rootWingOnBodyWidth = 6; //Attach this length of wing to the body to easy using tape to attach wings to body


span = bodyWidth+2*rootWingOnBodyWidth+2*nSections*printerMaxZ; //1000;
echo("span=",span);

sweptSparChordFrac = 0.24;
sparSweep = atan(tan(sweep)-sweptSparChordFrac*rootChord*(1-tipScale)/span);
echo(sparSweep= sparSweep);

wingLen = 0.5*(span-bodyWidth); //length of one wing not including fins

spanIncludingTips = span + 108;
echo("spanIncludingTips=",spanIncludingTips);

motorTECutout = motor?0*15:0;
motorTrayOffset = 0.9*motorTECutout; //Move tray forwards

echo("root chord=",rootChord);
echo("tip chord=",tipChord,"tip start y=",0.5*span,"tip offset=", 0.5*span*tan(sweep));
//echo("fin base chord=",tipChord,"tip start y=",0.5*span,"tip offset=", 0.5*span*tan(sweep));


module TranslateTrayAndHatchForMotor()
{
translate([-motorTrayOffset,0,0]) children();
}


//ESC size (HK 30 Amp)
escLen = 45;
escWidth = 26;
escHeight = 10;
escLeadLen = 80; //Length of the 3 leads to the motor


/* [Fuselage space] */
trayNoseHeight = 27+1;
trayNoseWidth = 36.5;
noseWeightD = 12.5;

duoStripWidth = 25.8+0.4;
duoStripLen = 11;
duoStripsThick = 5+0.3;

trayFloorY = -0.17*bodyH;
trayBodyLen = 90+motorTrayOffset-duoStripLen;
trayHeight = (camera || motor)?32.2:30;
trayWidth = 60+14;
trayMaxLen = 0.7*rootChord;
duoStripSupportThick = 2.0;
trayHeightRear1Pos = 1*trayBodyLen;
trayHeightRear1 = 0.61*trayHeight-0.175*motorTrayOffset-duoStripSupportThick;
trayHeightRear2Pos = 0.85*trayMaxLen;
trayHeightRear2 = 0.56*trayHeight-0.175*motorTrayOffset-duoStripSupportThick;

HatchLen = 0.38*(rootChord+noseLen)-motorTrayOffset;
HatchFrontX = -noseLen + motorTrayOffset+0.3*(rootChord+noseLen)-(motor?5:0);

hatchAngle = (motor || camera)?-1.8:-2.5;
HatchHeight = trayFloorY+trayNoseHeight-HatchFrontX*sin(hatchAngle);
hollowHatch = 1;//not a flat bottom

hatchCutoutRearPos = HatchFrontX+HatchLen-4;


//HatchHeight = 0.33*bodyH;

skinThick = 0.4;//-0.1; //For Cura -0.1
finThick = 1.0;

//Adjust these while rendering ServoAndSparCutouts() to ensure the 
//servo well does not penetrate the top surface of the wing.
//Note that ServoAndSparCutouts draws the wing surface inset by the 
//skinThick parameter above.
elevonServoWellDepthAdjust = 0.4+0.3;//0.5+0.1-0.4; //ProteusPlusv1: 0.0
flapServoWellDepthAdjust = -0.3;

servoWellBackFromSparmm = 0;


/* [Hidden] */
elevonStartTapeJoinHalfW = 10;
elevonEndToTip = 10;//12+5;  //Provide room for taping fin onto tip wing
elevonLen = 0.5*(wingLen-elevonEndToTip); //printerMaxZ;
echo("elevonlen = ",elevonLen);
elevonPieces = ceil(elevonLen/printerAbsMaxZ);
echo("elevonPieces = ",elevonPieces);
elevonPieceLen = elevonLen/elevonPieces;
echo("elevonPieceLen = ",elevonPieceLen);

elevonDifferential = 12.5;
elevonHingeRound = 0;
maxElevonDown = 30;
maxFlapUp = 10;
elevonChord = 0.25; //fraction of chord
elevDAtRoot = 0.048*rootChord;//8.97;
elevDAtTip = elevDAtRoot*tipChord/rootChord;
//elevonEndToTip = 12+0;  //Provide room between servo horn and root section end to allow tip section to be taped to root section.
hingeThick = 0.5;
elevonStartGap = 1.6;
elevonEndGap = 0.8;
hingeGap = 0.4;
flapEndsGap = 0.7;

hornHoleH = bottomHinges?14:17; //from hole to hinge point
hornTipR = 3.4;
hornHoleD = 1.3+0.5;
hornH = hornHoleH+hornTipR;
hornThick = 1.5;

servoTol = 0.8;
servoW = 23+servoTol;
servoH = 23+servoTol;
servoThick = 12+0.7;
servoHornDia = 8;
servoHornCenter = 6;
servHornSlotBottomH = 30; 
servoHornSlotWidth = 3;
servoHHornTop = 34+servoTol;
servoHHornBottom = 27.4+servoTol;
servoFlangeW = 32.6+servoTol;
servoFlangeThick =2.5+0.5;
servoFlangeHBottom =16.0;
servoTopW = 15+3;
servoHornCenterFomLeadSide = 6;
servoAngleRange = bottomHinges?80:110;
servoHornLen = 18;
servoHornR = bottomHinges?4:5;
servoLeadH = 9;
servoLeadSpace = 27+servoTol-servoW;
servoLeadW = 11;
servoLeadThick = 3.5;
servoPlugThick = 4+1;
servoPlugW = 10+1+0.75;

fingerGrips = false;
gripD = 20;
gripDepth = 20;

antennaD = 2+0.4;
antennaLen = 130;

//sparD = 5.82+(fuselage?0.55:0.35);
sparD = 5.08+0.6; //(fuselage?0.55:0.45);
sparHeightAdj = 0;//motor?-1:0;

//12x2x1000 carbon fiber strip (36g/m)
//10x1x1000 carbon fiber strip (15g/m)
sparThick = 1+0.4; //2+0.4;
sparH = 10+0.4; //12+0.5;
sparRaiseFrac = -0.07; //Fraction of rootThick to raise spar so it is centered in the airfoil

sparJoinerD = 5.08+(fuselage?0.55:0.45);

sweptSparFlexSpace = 3;

LEWingJoinerFrac = tubeSpar ? 0.2 : 0.1;
tipJoinerD = 3+(fuselage?0.45:0.3)-0.1;
tipJoinerLen = 70;
rootJoinerLen = 40;
rootRearJoinerWidth = 0.5*rootJoinerLen*sin(sweep)+tipJoinerD+1; //slot to allow wing to rotate forward on nose impact
echo("rootRearJoinerWidth",rootRearJoinerWidth);
fuseJoinerD = 2+0.3-0.1;
fuseJoinerLen = 16;
fuseBottomJoinerHAdj = motor? 1:0;
elevonJoinerLen = 60;
elevonJoinerD = 2+0.4; //these are longer than the fuselage joiners and need to be glued in

alignersD = 2;

$fn = 48;

thickFrac = thickPercent/100;

//sweep = atan(tan(sweepAtQuarterChord)+0.25*rootChord/wingLen*(1-tipScale));
//echo("sweep at leading edge=", sweep);

sparLenLeadingEdge = 0.542*wingLen;

elevonStartSpan = wingLen-elevonEndToTip-elevonLen;//wingLen/2;  //distance from root of wing to inner end of elevon (measured perpendicular to the root of the wing)
elevonStartFraction = elevonStartSpan/wingLen;

flapEndSpan = elevonStartSpan;//-flapEndsGap;
flapStartSpan = servoHHornTop+rootWingOnBodyWidth;//servoH;
flapLen = flapEndSpan-flapStartSpan;
flapStartFraction = flapStartSpan/wingLen;
echo("flapLen = ",flapLen);
flapPieces = ceil(flapLen/printerMaxZ);
echo("flapPieces = ",flapPieces);
flapPieceLen = flapLen/flapPieces;
echo("elevonPieceLen = ",flapPieceLen);

wingChordAtElevonStart =elevonStartFraction*tipChord + (1-elevonStartFraction)*rootChord;
elevonRootPos = (1-elevonChord)*rootChord;
elevonRootLen = elevonChord*rootChord;
elevD = (1-elevonStartFraction)*elevDAtRoot + elevonStartFraction*elevDAtTip;//Diameter of front of elevon at elevonStartFraction along wing
echo(elevD=elevD, elevDAtRoot=elevDAtRoot,elevDAtTip=elevDAtTip);

wingChordAtFlapStart =flapStartFraction*tipChord + (1-flapStartFraction)*rootChord;
flapRootPos = (1-elevonChord)*rootChord;
flapRootLen = elevonChord*rootChord;
flapD = (1-flapStartFraction)*elevDAtRoot + flapStartFraction*elevDAtTip;//Diameter of front of elevon at elevonStartFraction along wing

bodyLen = noseLen+rootChord;


sparChordPos = (tubeSpar?1.45*tan(sweep):sweptSparChordFrac)*rootChord;
rearSparChordPos = 1.82*tan(sweep)*rootChord;
rearTubeSparHAdj = -1;

//If spar is swept, the sparJoiner needs to be in the nose 4mm in front of the join to the Fuselage rear 
//noseCutX = tubeSpar ? 0.1*rootChord: (sparChordPos-bodyWidth/2*tan(sparSweep))+4;
noseCutX = tubeSpar ? 0.1*rootChord: (sparChordPos-bodyWidth/2*tan(sparSweep))-sparJoinerD/2-6;

//+35;//-(printerMaxXYLen-rootChord);

fuseInnerShellOffset = 6;

//HatchFrontX = -0.35*noseLen;
//hatchAngle = -1.5;
guardHeight = 1.25*hornH;//+((thickPercent/100)*(rootChord+tipChord)/2)/4; //assumes the horn is halfway along the wing
guardThick = 0.5;
guardWidth = 12;
defaultGuardNacelleWidthOverFullLen = 0.08;

motorSkidH = 12;
motorSkidW =18;
motorBoltFairingH = 6;
motorBoltFairingW = 9;


nMidSections = nSections-2;
rootSectionStart = bodyWidth/2+rootWingOnBodyWidth;
rootSectionEnd = rootSectionStart+printerMaxZ;
midSectionsEnd = rootSectionEnd+nMidSections*printerMaxZ;
tipSectionStart = midSectionsEnd;

elevonStart = bodyWidth/2+elevonStartSpan;
elevonEnd = elevonStart+elevonLen;

elevonSectionStart = elevonStart+midSectionIndex*(elevonPieceLen+hingeW);
elevonSectionEnd = min(elevonSectionStart+elevonPieceLen+hingeW,elevonEnd);

flapStart = bodyWidth/2+flapStartSpan;
flapEnd = flapStart+flapLen;

flapSectionStart = flapStart+midSectionIndex*(flapPieceLen+hingeW);
flapSectionEnd = min(flapSectionStart+flapPieceLen+hingeW,flapEnd);

midSectionStartIdeal = rootSectionEnd+midSectionIndex*printerMaxZ;
//midSectionEnd = midSectionStart+printerMaxZ;
midSectionEndIdeal = midSectionStartIdeal+printerMaxZ;

needElevonStartTapeJoinAdd = (midSectionEndIdeal > elevonStart - printerMaxZ/2) && (midSectionStartIdeal < elevonStart - printerMaxZ/2);
needElevonStartTapeJoinSubtract = (midSectionEndIdeal > elevonStart + printerMaxZ/2) && (midSectionStartIdeal < elevonStart + printerMaxZ/2);

//Subtract means the length of the section will be less (i.e adding to the start).
midSectionStart = midSectionStartIdeal+(needElevonStartTapeJoinSubtract?elevonStartTapeJoinHalfW:0);
midSectionEnd = midSectionEndIdeal+(needElevonStartTapeJoinAdd?elevonStartTapeJoinHalfW:0);

tipSectionEnd = midSectionsEnd+printerMaxZ-(flatPlateTip?elevonEndToTip:0.1*(thickPercent/100)*tipChord*sin(tipSectionCutAngle)+1.0*elevonEndToTip);

finSectionEnd = tipSectionEnd+printerMaxZ;

echo(rootSectionEnd=rootSectionEnd,midSectionStart=midSectionStart,midSectionEnd=midSectionEnd);

//Section(tipSectionEnd,3*printerMaxZ); //fin  
//Section(rootSectionEnd,tipSectionEnd); //tip section

 //  Section(midSectionStart,midSectionEnd,elevonStart>midSectionStart); //mid
//   Section(tipSectionStart,tipSectionEnd); //tip section
//SparJoiner();
if(partToGenerate == "FuselageRear")
   FuseRear();
else if(partToGenerate == "FuselageNose")
   Nose();
else if(partToGenerate == "FuselageHatch")
   Hatch();
else if(partToGenerate == "RootWing")
   Section(rootSectionStart,rootSectionEnd,(flaps?flapStart:elevonStart)>rootSectionStart); //root
else if(partToGenerate == "MidWing")
   {
   Section(midSectionStart,midSectionEnd,flaps?false:elevonStart>midSectionStart); //mid
   //Elevon(1);
   }
else if(partToGenerate == "TipWing")
   Section(tipSectionStart,tipSectionEnd); //tip section
else if(partToGenerate == "Fin")
   Section(tipSectionEnd,finSectionEnd); //fin   
else if(partToGenerate == "ServoGuard")
   {
   WingServoGuard();
   //WingServoGuard(assemble = false, flap = true);
   }
else if(partToGenerate == "ServoGuardFlaps")
   WingServoGuard(assemble = false, flap = true);
else if(partToGenerate == "Elevon")
   //Elevon();
   ElevonSection(elevonSectionStart,elevonSectionEnd,elevonSectionStart>elevonStart);
else if(partToGenerate == "Flap")
   //Elevon();
   ElevonSection(flapSectionStart,flapSectionEnd,flapSectionStart>flapStart,true);
else if(partToGenerate == "ShellOnly")
	{
   rotate([90,0,0])
      FlyingWingShell();
	}
else if(partToGenerate == "SparJoiner")
   {
   SparJoiner();
//FlyingWingHalf();
//FuseRear();
//ServoAndSparCutouts();
   }
else if(partToGenerate == "Hinges")
   {
   Hinges();
   //%WingBlankHalf(0);
   %FlyingWingShell();
   }
else
   {
   //FlyingWing()
   
   //Servo();
   //ServoAndSparCutouts();
   
   //example1(); 
   //rotate([80, 180, 130])
   //example2(); 
   //foil();
   //guard();
   
   //WingBlank();
   
   //BlendedFinTip();
   //Fin(tipChord);
   //Elevon(0,true); //flap
   //Elevon(0,false); //elevon
   //translate([50,0,0])
   //mirror([0,1,0])
   // rotate([0,0,170])
   //  Elevon();
   
   //rotate([90,0,0]) FlyingWing();
   //WingTip();
   //Wing();
   //WingShell();
   
   //TestSection();
   
   //Hatch();
   //HatchCutout(1);
   //HatchCutout(0);
   
   //HatchHalf();
   //%Fuselage();
   
   //FlyingWingHalf();
   //FlyingWingHalf(1);
   
   //Test nose
   //Nose();
   // intersection()
   //    {
   //    FlyingWingShell();
   //    //Nose();
   //    translate([-160,0,0])
   //    cube([200,200,200],true);
   //    }
   
   //WingServoGuard();
   //GuardNacelle(rootChord);

//tipSectionEnd = rootSectionEnd+printerMaxZ-(flatPlateTip?elevonEndToTip:0.5*(thickPercent/100)*tipChord*sin(tipSectionCutAngle)+0.55*elevonEndToTip);
   
   //Fuselage();
   
   //FuseLeftRear();
   //FuseLeft();
   
   //EquipmentTray(0);
   
//   Joiner(40,tipJoinerD,10);
//#Joiners(60);
//TestFuse();
   
   //WingBlankHalf();  //includes root section of length bodyWidth/2
   //WingBlankInner();
//   }
   
//   difference()
//      {
//      rotate([90,0,0])
//         FlyingWingShell();
//      translate([3-noseLen,0,-7])
//         rotate([0,-6,0])
//         MobiusCam();
//      }
      
//   AddCamera(cameraPos,cameraAngle,camera) 
//      %FlyingWingShell();

//Check servo channels and spars
    //ServoAndSparCutouts();
    //%MainSpar(true);
    //%WingBlankHalf();
    //%WingBlankInner();
    //%FuselageHalfShell(); 
   //%WingBlankSkin();
   //Servo();

     //FlyingWingShell();
      
// AddCamera(cameraPos,cameraAngle,camera,false)
//       TranslateTrayAndHatchForMotor()
//          EquipmentTray(1); //cutOutHatch = 1
      
//   FuselageRearSkid();
//      HatchHalf();
      
//      AddCamera(cameraPos,cameraAngle,camera)
//         Fuselage(0);
      
//     HatchCutout();
//      Hatch();
//      AddMotorMount();
//         %blendFoils();
//      FlyingWingHalf();
   }
   
module FuselageRearSkid()
{
if(skid)
   {
   skidLen = 0.8*rootChord;
   skidAngle = forwardSkid?0:-60;
   translate([forwardSkid?-0.95*noseLen:rootChord-skidLen-motorTECutout-3,0,0])
      multmatrix(m = [ [1, tan(skidAngle), 0, 0],
                 [0, 1, 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0,  1]
                 ])

         intersection() 
            {
            rotate([0,90,-90])
               GuardNacelle(skidLen,guardHeight);
            mirror([0,1,0])
               cube([skidLen, skidLen/4, skidLen/2]);
            }
   } 
}

module MotorSkid(skidLen)
{
skidAngle = 50;
scaleyz = 1.3;

multmatrix(m = [ [1, tan(skidAngle), 0, 0],
  [0, 1, 0, 0],
  [0, 0, 1, 0],
  [0, 0, 0,  1]
  ])
//scale([1,scaleyz*0.35,scaleyz*2])
rotate([0,90,-90])
   GuardNacelle(skidLen, motorSkidH, motorSkidW);
}
   
module AddMotorMount()
{
difference()
   {
//   motorTubeLen = 0.6*rootChord;
   motorTubeX = rootChord-motorTECutout-motorTubeLen;
   //taperFracOD = 0.68;
//   nacelleLen = motorTubeOD;
//   nacellePosAdj = 0.77*nacelleLen;
//   motorBoltD = 3.2;
//   motorBoltHeadD = 5.4;
//   motorBoltAngle = 75;
   skidLen = 90;
   skidOverhang = 30;
   skidThick = 2;
   union()
      {
      children();
      if(motor)
         {
         translate([motorTubeX,0,0])
            {
            //motor wires shroud
            mirror([0,1,0])
            translate([motorTubeLen-skidLen+skidOverhang,0.45*motorTubeOD,0])
            MotorSkid(skidLen);  
            //motor bolt shrouds
            rotate([motorBoltAngle,0,0])
            translate([motorTubeLen-nacellePosAdj,-0.4*motorTubeOD,0])
               //scale([1,0.2,3])
               rotate([0,90,-90])
                  GuardNacelle(nacelleLen,motorBoltFairingH,motorBoltFairingW);
            rotate([180-motorBoltAngle,0,0])
            translate([motorTubeLen-nacellePosAdj,0.4*motorTubeOD,0])
               //scale([1,0.2,3])
               rotate([0,90,-90])
                  GuardNacelle(nacelleLen,motorBoltFairingH,motorBoltFairingW);
            rotate([0,90,0])
               cylinder(d1=taperFracOD*motorTubeOD, d2=motorTubeOD,h=motorTubeLen);
            }
         }
      }
   if(motor)
      {
      translate([motorTubeX,0,0])
         {
         rotate([0,90,0])
            {
            cylinder(d1=taperFracOD*motorTubeOD-2*motorTubeThick, d2=motorTubeOD-2*motorTubeThick,h=motorTubeLen);
            }
          //space for ESC when connecting motor wires
          translate([motorTubeLen-escLeadLen+13-1.5*escLen,-escHeight/2+2,-escWidth/2])
            #cube([1.5*escLen,escHeight,escWidth]);
          mirror([0,1,0])
          translate([motorTubeLen-skidLen+skidOverhang,0.45*motorTubeOD - skidThick,0])
            scale([0.95,1,0.8]) #MotorSkid(skidLen);  

         }
         translate([motorTubeX+motorTubeLen,0,0])rotate([0,90,0])
            cylinder(d=motorTubeOD, h=motorTubeLen/2);
      translate([motorTubeX+motorTubeLen-motorMountBoltZFrac*nacellePosAdj,0,0])
            {
            rotate([motorBoltAngle,0,0])
            rotate([90,0,0])
            #cylinder(d = motorBoltD, h=0.7*motorTubeOD,center = false);
            rotate([-motorBoltAngle,0,0]) 
            rotate([90,0,0])
            #cylinder(d = motorBoltD, h=0.7*motorTubeOD,center = false);
            rotate([motorBoltAngle,0,0])
            translate([0,-0.625*motorTubeOD,0]) 
            rotate([90,0,0])
            #cylinder(d = motorBoltHeadD, h=0.25*motorTubeOD,center = true);
            rotate([-motorBoltAngle,0,0])
            translate([0,-0.625*motorTubeOD,0]) 
            rotate([90,0,0])
            #cylinder(d = motorBoltHeadD, h=0.25*motorTubeOD,center = true);               
            }
      }
   }
   
}

module FuselageHalfShell()
{
AddMotorMount()
   {   
   FuselageShell();
   //mirror([0,0,1])
   //   FuselageShell();
   }

}

module FlyingWingShell()
{
WingShell();
//As of 2017.01.20, need to mirror WingShell separately from FuselageShell to avoid: 
//"ERROR: CGAL error in CGALUtils::applyBinaryOperator union: CGAL ERROR: assertion violation! Expr: itl != it->second.end() File: /opt/mxe/usr/x86_64-w64-mingw32.static/include/CGAL/Nef_3/SNC_external_structure.h Line: 1102 "
mirror([0,0,1])  WingShell();
   
AddMotorMount()
   {   
   FuselageShell();
   mirror([0,0,1])
      FuselageShell();
   }
}


//CG calculation
//N.B. this assumes the fuselage (or at least nose of the fuselage) does not generate
//significant lift forward of the wing's Mean Aerodynamic Chord, which is probably not
//true. Still, it seems to work reasonably well in practice.
//Copied from http://rcwingcog.a0001.net/V3_testing/?i=1
// rcwingcog@gmail.com 
cgInfo = CalcCGPos(CGPercent, sweep,span, bodyWidth, rootChord, tipChord);

echo("cgInfo =",cgInfo);
cg_dist = cgInfo[0][0];
wing_area = cgInfo[0][1];
echo("cg_dist=",cg_dist);
echo("Wing area[dm^2]=",wing_area);
echo("Wing area[square feet]=",0.107639*wing_area);


module ServoAndSparCutouts()
{
   translate([0,0,bodyWidth/2])
      rotate([0,0,-90])
         {
         ServoCutout();
         if(flaps)
            FlapServoCutout();
         }
   MainSpar(true);
   //%WingBlankInner();
   //%WingBlankSkin();
}

//Test(bodyWidth/2,printerMaxZ);

//TestFuse();

module TestFuse()
{
intersection()
   {
   difference()
      {
      FuseLeft();
      translate([0.75*rootChord,0,bodyWidth/2])
         rotate([0,0,-90])
         Joiner(tipJoinerLen,tipJoinerD);
      }
   translate([0.55*rootChord,-2*bodyH,0.2*bodyWidth])
      cube([2*rootChord,4*bodyH,bodyWidth]);
   }
}

module Test(h1,h2)
{
      intersection()
         {
         translate([0,0,bodyWidth/2])
            rotate([0,0,-90])
               WingBlank();
         //();
         union()
            {
            Joiners(h1);
            if(h2 < span/2)
               Joiners(h2);
            }
         }
      }

//servoChordPos = (tubeSpar?0.32:0.36-0.02)*wingChordAtElevonStart+elevonStartSpan*tan(sweep);
//flapServoChordPos = (tubeSpar?0.32:0.38)*wingChordAtFlapStart+flapStartSpan*tan(sweep);
servoChordPos = (tubeSpar?0.32:sweptSparChordFrac)*wingChordAtElevonStart+elevonStartSpan*tan(sweep)+servoWellBackFromSparmm+servoFlangeW/2;
flapServoChordPos = bottomHinges?rootChord-108:(tubeSpar?0.32:0.38)*wingChordAtFlapStart+flapStartSpan*tan(sweep);


module WingServoGuardPos(flap = false) //in Wing coordinate system
{
startSpan = flap?flapStartSpan:elevonStartSpan;
translate([startSpan*tan(-dihedral),startSpan*tan(sweep),startSpan])
      rotate([0,-dihedral-0,washout*(flap?flapStartFraction:elevonStartFraction)+0])
      rotate([0,90,0])
         children();
}

module WingServoGuard(assemble = false, flap = false) //in Wing coordinate system
{
guardFullLen = flap?wingChordAtFlapStart:wingChordAtElevonStart;
guardFullH = guardHeight+(1-(flap?flapStartFraction:elevonStartFraction))*rootThick;
   //Servo cutout
//#translate([(servoThick+0.5)/2-0.007*(rootChord+tipChord)/2+elevonStartSpan*sin(-dihedral),
difference()
   {
//   translate([elevonStartSpan*tan(-dihedral),elevonStartSpan*tan(sweep),elevonStartSpan])
//         rotate([0,-dihedral-0,washout*elevonStartFraction+2])
//         rotate([0,90,0])
   WingServoGuardPos(flap)
            intersection()
               {
               GuardNacelle(guardFullLen,guardHeight);
               translate([-rootChord/2,0,0])
                  //cube([rootChord,(tubeSpar?0.28:0.3)*guardFullLen,guardFullH]);
               if(flap)
                  cube([rootChord,flapServoChordPos-flapStartSpan*tan(sweep)-10,guardFullH]);
               else
                  cube([rootChord,servoChordPos-elevonStartSpan*tan(sweep)-10,guardFullH]);
               }
   //WingServoGuardPos()
      if(!assemble)
         {
         #WingBlank();
         intersection()
            {
            translate([2*skinThick,0,0])
            WingBlank();            
            #WingServoGuardPos(flap)
               GuardNacelleHoles(guardFullLen);
            }
         }
    }
}

module GuardNacelleHoles(fullLen) //Glue holes
{
translate([0,0.1*fullLen,0])
   #cylinder(d = 2,h = 10);
translate([0,0.05*fullLen,0])
   #cylinder(d = 2,h = 10);
translate([0,0.03*fullLen,0])
   {
   //sideOffset = guardWidthOverFullLen*fullLen;
   sideOffset = guardWidth;
   translate([0.25*sideOffset,0.14*fullLen,0])
      #cylinder(d = 2,h = 10);
   translate([0.25*sideOffset,0.2*fullLen,0])
      #cylinder(d = 2,h = 10);
   translate([-0.25*sideOffset,0.165*fullLen,0])
      #cylinder(d = 2,h = 10);
   translate([-0.25*sideOffset,0.21*fullLen,0])
      #cylinder(d = 2,h = 10);
   }
}

module GuardNacelle(fullLen, height, widthIn = -1)  //height below TE of wing
{
width = (widthIn == -1) ?  fullLen*defaultGuardNacelleWidthOverFullLen : widthIn;
difference()
   {
   union()
   {
   difference()
      {
      difference()
         {
         nacelle(0,width/*OverFullLen*/,height,fullLen);
         //nacelle(guardThick,guardWidthOverFullLen,guardHeight,fullLen);
         }
//      extrudeBy = foilWidthRat*nacelleLen*2;
//      translate([extrudeBy/2,0,flatBottomY])
//      rotate([0,-90,0])
//      rotate([0,0,90])
//      linear_extrude(extrudeBy) polygon(innerRoot);
      }
   }
   }
//translate([-pressureSensorW/2,45,duolockThick+mountThick+1])
//   cube([pressureSensorW,pressureSensorLen,pressureSensorH]);
}


module nacelle(inset = 0,width = 0.25,guardHeight = 25, L = 100, N = 100)
{
t = width/L;
outerPts = airfoil_data([0.0,0.0,t], L, N, false);
      
//scale([1,1,2*(guardHeight/L)/t])
scale([1,1,guardHeight/(width/2)])
rotate([90,0,0])
rotate_extrude(convexity=10,$fn=100) 
rotate([0,0,-90])
   {
   intersection()
      {
//[Camber, Camber pos, thickness]
      //polygon(points = airfoil_data([0,0,t], L, N, open));
      offset(r=-inset)
         polygon(outerPts);
      square([L,t*L/2]);
      }
   }
}


module Nose()
{
AddCamera(cameraPos,cameraAngle, camera, true, flexiFilament) 
   {
   //rotate([90,0,0])
   difference()
      {
      union()
         {
         //FlyingWingHalf(0);
         FuseLeft();
         mirror([0,0,1])
            //FlyingWingHalf();
            FuseLeft();
         }

      //Section(0,bodyWidth/2);
      translate([noseCutX,-2*bodyH,-span])
         cube([2*rootChord,4*bodyH,span*2]);
      }
   }

}

module FuseLeftRear()
{
intersection()
   {
   FuseLeft();
   translate([noseCutX,-2*bodyH,0])
      cube([2*rootChord,4*bodyH,bodyWidth]);
   }
}

module FuseRear()
{
FuseLeftRear();
mirror([0,0,1])
   FuseLeftRear();
}

module FuseLeft()
{
difference()
   {
   Section(0,bodyWidth/2+rootWingOnBodyWidth);
   #translate([noseCutX,sparJoinerD+antennaD/2+(fuseJoinerD/2+fuseJoinerD/2)+0.5+3,0.7*bodyWidth/2])
      rotate([60,0,0])
         rotate([0,90,0])
            FuseJoiner();  //cylinder(d=fuseJoinerD,h=fuseJoinerLen);
    #translate([noseCutX,trayFloorY-1.5*fuseJoinerD+fuseBottomJoinerHAdj,0])
      rotate([0,0,0])
         rotate([0,90,0])
            FuseJoiner();  //cylinder(d=fuseJoinerD,h=fuseJoinerLen);
  if(fingerGrips)
      #translate([0.33*rootChord,0,trayWidth/2+0.6])
         rotate([90,0,0])
            {
            intersection()
               {
               cylinder(d=gripD,h=gripDepth);
               translate([-gripD/2,0,0])
               cube([gripD,gripD/2,gripDepth]);
               }
            }
   if(CGDimples)
      difference()
         {
      #translate([cg_dist,0,bodyWidth/2-CGDimpleDia])
//      #translate([rootChord,0,bodyWidth/2]) testing!
         rotate([90,0,0])
            cylinder(d=CGDimpleDia,h=CGDimpleDepth);
         //FuseInner();
         blendFoils(offsetR = -1.5);
         }
   }
//anti-warp TE hold-down if printing on side
//#translate([rootChord-6,0,0]) scale([1.5,1,1]) cylinder(r=6,h=1);
}

module SparJoiner()
{
SparJoinerHalf();
mirror([0,0,1])
   SparJoinerHalf();
}

module SparJoinerHalf()
{
zipTieW = 5.2;
endZipTieW = 3;
zipTieSpacing = 3;
zipTieGrooveDepth = 1.3;
endZipTieGrooveDepth = 1.5;

centralBodyWidth = 30;
width = 0.5*trayWidth-3;
yScale = 1.75;
xScale = 1.4/cos(sparSweep);//1.4;
endXScale = 1.1;
endZipTieSpacingO2 = width - 3;
endZipTieDist = (endZipTieSpacingO2-(endXScale+endZipTieGrooveDepth/sparH)*sparH/2*sin(sparSweep))/cos(sparSweep);//
//In Section(), x increases to TE, y increases to top, z increases to left tip.
//trayWidth,trayHeight
difference()
{
translate([sparChordPos-bodyWidth/2*tan(sparSweep),sparRaiseFrac*rootThick-bodyWidth/2*tan(dihedral),0])
intersection()
   {
difference()
   {
   union()
      {
      translate([sparThick/2,sparH/2,0])
         {
         //arms
         multmatrix(m = [ [1, 0, tan(sparSweep), 0],
                           [0, 1, tan(1*dihedral), 0],                       [0, 0, 1, 0],
                           [0, 0, 0,  1]
                     ])
         scale([endXScale+endZipTieGrooveDepth/sparH,yScale+endZipTieGrooveDepth/sparH,1])
         cylinder(d=sparH, h=width);

         //center
         //scale([xScale,yScale,1])
         //cylinder(d=sparH+zipTieGrooveDepth/yScale, h=zipTieSpacing/2);

         //#translate([0,0,zipTieSpacing/2])
         //scale([xScale,yScale,1])
         //cylinder(d=sparH, h=zipTieW);

         //#translate([0,0,zipTieSpacing/2+zipTieW])

         scale([xScale,yScale,1])
            cylinder(d=sparH+zipTieGrooveDepth/yScale, h=centralBodyWidth/2);

         }
      }
   //zipTieGrooves
   translate([0,0,zipTieSpacing/2])
   translate([sparThick/2,sparH/2,0])
   difference()
      {
      cylinder(d=10*sparH, h=zipTieW);
      scale([xScale,yScale,1])
         cylinder(d=sparH, h=zipTieW);
      }

   //endZipTieGrooves
   rotate([0,sparSweep,0])
   rotate([-dihedral,0,0])
   translate([0,0,endZipTieDist])
   translate([sparThick/2,sparH/2,0])
   difference()
      {
      cylinder(d=10*sparH, h=endZipTieW);
      scale([endXScale-endZipTieGrooveDepth*tan(sweep)/sparH,yScale,1])
         cylinder(d=sparH, h=endZipTieW);
      }
  //subtract the spar
//  rotate([-dihedral,0,0])
//      rotate([0,sweep,0])
////         translate([0,0,-sparLenInFuseO2])
//         {
//         cube([sparThick,sparH,span/2]);
//         }

//bevel entrance
      translate([sparThick/2,sparH/2,0])
         {
         multmatrix(m = [ [1, 0, tan(sparSweep), 0],
                           [0, 1, tan(1*dihedral), 0],                       [0, 0, 1, 0],
                           [0, 0, 0,  1]
                     ])
         {
         yTaper = 1.2;
         xTaper = 2.4;
         taperLen = 3;
//         translate([sparThick/2,sparH/2,width])
         translate([0,0,width-taperLen])
         linear_extrude(height = taperLen,scale=[xTaper,yTaper],convexity = 5,$fn=100)
            square([sparThick,sparH],true);
         }
         }
//Fuselage connector spar
//   translate([-sparThick-1.0,0,0])
//      cube([sparThick,sparH,span/2]);
//   #translate([-sparJoinerD/2-0.2,sparJoinerD/2+0.5,0])
//      cylinder(d=sparJoinerD, h=bodyWidth/2);

   }
//Flatten top for printing
   translate([0,0,0])
      cube([20*sparH,1.5*yScale*sparH,span/2],true);
   
   }
//Fuselage connector spar
#MainSpar();
}
}


module TestSectionMidJoin()
{
intersection()
   {
   //FlyingWingHalf();
   WingTip();
   //translate([])  cube([]);
   translate([-rootChord,-2*rootChord,wingLen/2+bodyWidth/2])
      cube([4*rootChord,4*rootChord,10]);
   }
}

module TestSectionFinJoin()
{
intersection()
   {
   //FlyingWingHalf();
//tipSectionStart?
   Section(rootSectionEnd,tipSectionEnd); //tip section
   //translate([])  cube([]);
   translate([-rootChord,-2*rootChord,wingLen+bodyWidth/2-21])  cube([4*rootChord,4*rootChord,17]);
   }
}


//TestSectionMidJoin();
//TestSectionFinJoin();


module WingBlankInner()
{
WingBlankHalf(-2*skinThick);
}

module WingBlankSkin()
{
WingBlankHalf(0);
}

module ElevonSection(h1 , h2, TEHoldDown = 0,flap = false)
{
intersection()
   {
   translate([0,0,bodyWidth/2])
   rotate([0,0,-90])
      Elevon(0,flap);
   if((flap?flapLen:elevonLen) > printerMaxZ)
      difference()
         {
         translate([-rootChord,-2*rootChord,h1])       
            cube([4*rootChord,4*rootChord,h2-h1]);
         
         intersection()
            {
            WingBlankInner();
            union()
               {
               if(h1 > (flap?flapStart:elevonStart))
                  #ElevonJoiners(h1,flap);
   //            if(!flatPlateTip && h1wing > 0.75*wingLen)
   //               {
   //               //Aligners(
   //               }
               if(h2 < (flap?flapEnd:elevonEnd))
                  #ElevonJoiners(h2,flap);
               }
            }
         }
   }
if(TEHoldDown)
   {
   h1wing = h1-bodyWidth/2;
   h2wing = h2-bodyWidth/2;
   h1wingfrac = h1wing/(wingLen);
   h2wingfrac = h2wing/(wingLen);
   
   h2Chord = h2wingfrac*tipChord+(1-h2wingfrac)*rootChord;
   h1Chord = h1wingfrac*tipChord+(1-h1wingfrac)*rootChord;
//   translate([h1wing*sin(sweep),h1wing*sin(dihedral)+0.01*(rootChord+tipChord),h1])
   translate([h1wing*tan(sweep),h1wingfrac*h1Chord*sin(washout)+h1wing*tan(dihedral),h1])
      {
      #translate([1*h1Chord-6,0,0])
         scale([2,1,1])
         cylinder(r=6,h=0.6);
      }
   }
}

module Section(h1 , h2, TEHoldDown = 0)
{
h1wing = h1-bodyWidth/2;
h2wing = h2-bodyWidth/2;
h1wingfrac = h1wing/(wingLen);
h2wingfrac = h2wing/(wingLen);

h2Chord = h2wingfrac*tipChord+(1-h2wingfrac)*rootChord;
h1Chord = h1wingfrac*tipChord+(1-h1wingfrac)*rootChord;

fin = h1 >= tipSectionEnd;

intersection()
   {
   //difference()
   difference()
      {
      FlyingWingHalf(0);
      Hinges();
      }
   difference()
      {
      if(tipSectionCutAngle==0 || h2 < tipSectionEnd)
         translate([-rootChord,-2*rootChord,h1])       
            cube([4*rootChord,4*rootChord,h2-h1]);
   
      else
         {
         if(fin)
            {
            hSheared = (h2-h1);
            translate([0,h1*tan(dihedral),h1])
            multmatrix(m = [ [1, 0, 0, 0],
                             [0, 1, 0, 0],                        
                             [0, tan(-tipSectionCutAngle), 1, 0],
                             [0, 0, 0,  1]
              ])
            //rotate([-35,0,0])
            translate([0,-2*rootChord,0])
            cube([4*rootChord,4*rootChord,4*hSheared]);
            }
         else
            union()
               {
               hSheared = 0.5*(h2-h1);
               translate([rootChord,h2*tan(dihedral),h1+1.5*hSheared])
               multmatrix(m = [ [1, 0, 0, 0],
                                [0, 1, 0, 0],                        
                                [0, tan(-tipSectionCutAngle), 1, 0],
                                [0, 0, 0,  1]
                 ])
               //rotate([-35,0,0])
               cube([4*rootChord,4*rootChord,hSheared],true);
               translate([-rootChord,-2*rootChord,h1])       
                  cube([4*rootChord,4*rootChord,0.75*(h2-h1)]);
      
               }
         }
      //if(h2 > 0.75*wingLen)
      intersection()
         {
         WingBlankInner();
         union()
            {
				MainSpar(true,h1,h2);
            if(h1 > 0 && h1 < tipSectionEnd)  //not needed unless printing fuselage on side
               Joiners(h1);
//            if(!flatPlateTip && h1wing > 0.75*wingLen)
//               {
//               //Aligners(
//               }
            if(h2 < tipSectionEnd)
               Joiners(h2);
            }
         }
      if(fin)
         {
         Joiners(h1,-1,2); //bottom dimples
         echo("Fin Joiners");
         }
      }
   }
if(!fin && h2 >= tipSectionEnd)
   Joiners(h2,1,2); //top Bumps

if(TEHoldDown)
   {
//   translate([h1wing*sin(sweep),h1wing*sin(dihedral)+0.01*(rootChord+tipChord),h1])
   translate([h1wing*tan(sweep),h1wingfrac*h1Chord*sin(washout)+h1wing*tan(dihedral),h1])
      {
      #translate([1*h1Chord-6,0,0])
         scale([2,1,1])
         cylinder(r=6,h=0.6);
      }
   }
}

module Aligner(h, l = 0.5*thickFrac*tipChord,female = false)
{
//   linear_extrude(height = wingLen+0.07*tipChord*tan(finAngle), convexity = 5,twist = -washout, scale=[tipScale,tipScale], $fn=100)
tol = female?0.25:0;
hTol = female?0.75:-0.0;
taper = 0.8;
translate([0,0,(0.1)*tipChord*thickFrac*tan(-tipSectionCutAngle)])
            multmatrix(m = [ [1, 0, 0, 0],
                             [0, 1, (1-taper)/2+tan(0.75*tipSectionCutAngle), 0],                        
                             [0, tan(-tipSectionCutAngle), 1, 0],
                             [0, 0, 0,  1]
              ])
linear_extrude(height = h+hTol,scale=[taper,taper],convexity = 5,$fn=100)
   square([3.5+tol,l+tol],true);
}

module Joiner(length , dia, width = 0)
{
cylinder(d=dia,h=length,center=true);
if(width > 0)
   {
   translate([width/2,0,0])
      cube([width,dia,length],true);
   }
difference()
   {
   cube([0.1,30,length+3],true);
   cube([30,30,1.2],true);
   }
}

module FuseJoiner(length=fuseJoinerLen , dia=fuseJoinerD)
{
cylinder(d=dia,h=length,center=true);
difference()
   {
   cube([11,0.1,length+3],true);
   cube([30,30,1.2],true);
   }
}

module ElevonJoiners(h,flap = false)
{
hwing = h-bodyWidth/2;
//hRootJoin = bodyWidth/2+rootWingOnBodyWidth;
hwingfrac = hwing/(wingLen);
hChord = hwingfrac*tipChord+(1-hwingfrac)*rootChord;

function yCenterScaling(chordFrac) = chordFrac*hChord*hwingfrac*sin(washout)-0.41*elevD;
   
function zOffset(chordFrac) = tan(-tipSectionCutAngle)*chordFrac*hChord*hwingfrac*sin(washout);

elevonJoinerChordFrac = 0.79;
translate([hwing*tan(sweep),hwing*tan(dihedral)+0.01*(rootChord+tipChord),h])
   {
   if(h >= (flap?flapStart:elevonStart))
      {
echo("flap=",flap);
      TEAngle = atan((rootChord-tipChord)/wingLen);
      echo("TEAngle=",TEAngle);
      #translate([elevonJoinerChordFrac*hChord,yCenterScaling(elevonJoinerChordFrac),0])
         rotate([-dihedral,0,0])
         rotate([0,sweep-TEAngle,0])
         Joiner(elevonJoinerLen,elevonJoinerD);
      }
   }
}

module Joiners(h,topBumps=0,aligners = 0)
{
alignersR = topBumps>0?alignersD/2:alignersD/2+0.15;
hwing = h-bodyWidth/2;
hRootJoin = bodyWidth/2+rootWingOnBodyWidth;
hwingfrac = hwing/(wingLen);
hChord = hwingfrac*tipChord+(1-hwingfrac)*rootChord;
//Temporary for joining Proteus v2 prototype 1 outer wings to fixed fuselage
//angle = (tubeSpar || (h > hRootJoin))?0:sweep; 
angle = (h > hRootJoin) ? (tubeSpar?0:sweep) : 0; //Now we have slot in fuselage don't angle the root joiner (makes fuselage easier to print)

function yCenterScaling(chordFrac) = chordFrac*hChord*hwingfrac*sin(washout)-0.25*elevD;
   
function zOffset(chordFrac) = tan(-tipSectionCutAngle)*chordFrac*hChord*hwingfrac*sin(washout);

translate([hwing*tan(sweep),hwing*tan(dihedral)+0.01*(rootChord+tipChord),h])
   {
   if(!topBumps)
      {
      joinerLen = (h > hRootJoin && tubeSpar) ? tipJoinerLen : rootJoinerLen;
      joinerWidth = (h > hRootJoin || !fuselage)? 0 : rootRearJoinerWidth;
      if(h >= hRootJoin && !rearTubeSpar || h > hRootJoin+10)
         #translate([0.62*hChord,yCenterScaling(0.62),0])
            rotate([0,angle,0])
               Joiner(joinerLen,tipJoinerD,joinerWidth);
      if(h > hRootJoin+10) //h > maxSparLenO2Prj) //Need front joiner if beyond end of main spar
         #translate([LEWingJoinerFrac*hChord,yCenterScaling(LEWingJoinerFrac),0])
            rotate([0,angle,0])
               Joiner(joinerLen,tipJoinerD);
      }
   if(topBumps!=0)
      {
      if(h > 0 && aligners==1)
         {
         translate([0.1*hChord,yCenterScaling(0.1),0]) sphere(alignersR);
         translate([0.32*hChord,yCenterScaling(0.32),0]) sphere(alignersR);
         translate([0.72*hChord,yCenterScaling(0.70),0]) sphere(alignersR);
         }
      if(h > 0 && aligners==2)
         {
         //translate([0.1*hChord,yCenterScaling(0.1),0]) sphere(alignersR);
         posFrac1 = 0.2;
         #translate([posFrac1*hChord,yCenterScaling(posFrac1),zOffset(posFrac1)]) Aligner(h=2.5,l=0.5*thickFrac*tipChord,female = topBumps<0);
         posFrac2 = 0.5;
         #translate([posFrac2*hChord,yCenterScaling(posFrac2),zOffset(posFrac2)]) Aligner(h=2.5,l=0.4*thickFrac*tipChord,female = topBumps<0);
         }
      }
   }
//Only needed if printing on side
//if(h <= 0)
//   {//Fuselage join
//   //translate([-0.85*noseLen,0,0]) FuseJoiner();//cylinder(d=fuseJoinerD,h=fuseJoinerLen);
//   translate([0.8*rootChord,0,0]) FuseJoiner();//cylinder(d=fuseJoinerD,h=fuseJoinerLen);
//   translate([0.8*noseCutX-1,-0.25*bodyH,0]) FuseJoiner();//cylinder(d=fuseJoinerD,h=fuseJoinerLen);
//   translate([0.8*noseCutX-1,0.52*bodyH,0]) FuseJoiner();;//cylinder(d=fuseJoinerD,h=fuseJoinerLen);
//   }  

}

module Hatch()
{
rotate([90,0,0])
rotate([0,0,-1*hatchAngle])
   {
   //AddCamera(cameraPos,cameraAngle,camera)
      union()
         {
         HatchHalf();
         mirror([0,0,1]) HatchHalf();
         }
   }
}

module HatchHalf()
{
intersection()
   {
   AddCamera(cameraPos,cameraAngle,camera,false)
   if(hollowHatch)
         Fuselage(0);
   else
      blendFoils();
   TranslateTrayAndHatchForMotor()
      HatchCutout(0);
   }
}

module HatchCutout(cutout = 1)
{
//cutout = 0; testing!
tol = cutout?0.15:0; //make sure the hatch is not too tight


//lenFrac = 0.4;
widthFrac = 0.6;
//backChamferFrac =1.1;
//linear_extrude(height = 0.3*bodyWidth/2, convexity = 5, $fn=100)
//   polygon([[0-0.1*noseLen,0],[lenFrac*(rootChord+noseLen),-0.05*bodyH],[backChamferFrac*lenFrac*(rootChord+noseLen),0.35*bodyH],[-0.04*(noseLen),0.35*bodyH]]);

//Shear 
length = HatchLen;
//Ensure the angle of the "V" in the fuselage is larger than in the hatch so that the
//front and tongue of the hatch make contact before the tip of the "V"
lenAngle = cutout?-45:-49;//-46;//-50:-53; //-25;//-52;
catchAngle = cutout?45:45;;//52;//-lenAngle; from vertical
widthAngle = 40;
hatchDepth = 0.35*bodyH;
//Testing
translate([-0*tol,0,0])
multmatrix(m = [ [1, 0, 0, 0],
                 [tan(hatchAngle), 1, 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0,  1]
                 ])
translate([HatchFrontX-tol,HatchHeight-tol,0])
   {
   difference()
      {
      union()
         {
         multmatrix(m = [ [1, tan(lenAngle), 0, 0],
                          [0, 1, 0, 0],
                          [0, tan(widthAngle), 1, 0],
                          [0, 0, 0,  1]
                          ])
            {
            translate([0,0,-(widthFrac*bodyWidth/2)/2])
               cube([1.4*length,0.35*bodyH,widthFrac*bodyWidth/2]);
            //translate([length/2,0,-bodyWidth/4])
            //   cube([length,0.35*bodyH,bodyWidth/2]);
            }
         //Projecting hatch tongue
         multmatrix(m = [ [1, tan(catchAngle), 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, 0],
                          [0, 0, 0,  1]
                          ])
            {
            //translate([-(0.2*bodyH)-(camera?10:0),-0.35*bodyH*tan(lenAngle)*tan(hatchAngle),0])
            translate([-hatchDepth,-hatchDepth*tan(lenAngle)*tan(hatchAngle),0])
            //translate([0.25*bodyH*sin(lenAngle)+0.25,-0.35*bodyH*sin(lenAngle)*sin(hatchAngle),0]) //hack hatch for first nose printed without 2*tol above!
               cube([length,hatchDepth,0.2*widthFrac*bodyWidth/2+tol]);
            //translate([length/2,0,-bodyWidth/4])
            //   cube([length,0.35*bodyH,bodyWidth/2]);
            }
         if(cutout)
            {
            translate([-HatchFrontX,-duoStripsThick,0])
            multmatrix(m = [ [1, tan(-lenAngle), 0, 0],
                             [0, 1, 0, 0],
                             [0, 0, 1, 0],
                             [0, 0, 0,  1]
                             ])
               cube([length+HatchFrontX+duoStripLen-tan(-lenAngle)*duoStripsThick,duoStripsThick,duoStripWidth/2]);
            //#translate([-HatchFrontX,-trayHeight/2,0])              cube([length+HatchFrontX,trayHeight/2,1.0*trayWidth/2]);
            }
         }
//      if(!cutout)
//         {
//         #translate([-5,-3.0*duoStripsThick/2-0.5,0])
//            cube([length+duoStripLen+duoStripsThick/2,4*duoStripsThick/2,duoStripWidth/2]);
//         }

//Uncomment this for Hatch without projecting tongue
//         #multmatrix(m = [ [1, tan(-catchAngle), 0, 0],
//                          [0, 1, 0, 0],
//                          [0, 0, 1, 0],
//                          [0, 0, 0,  1]
//                          ])
//            {
//            translate([0.25*bodyH*sin(lenAngle),0.06*bodyH+2*tol,0])
//               cube([0.25*bodyH*cos(lenAngle),0.25*bodyH,0.2*widthFrac*bodyWidth/2+tol]);
//            //translate([length/2,0,-bodyWidth/4])
//            //   cube([length,0.35*bodyH,bodyWidth/2]);
//            }

      }
   }
}

module Fuselage(cutoutHatch = 1)
{
//trayLen = 0.7*bodyLen;
//trayHeight = 0.6*bodyH;
//trayWidth = 0.4*bodyWidth;

difference()
   {
   AddMotorMount()
      FuselageShell();
   TranslateTrayAndHatchForMotor()
      EquipmentTray(cutoutHatch);
   }

}

module FuseInner()
{
translate([0,0.2*fuseInnerShellOffset,0])
   blendFoils(offsetR = -fuseInnerShellOffset);
}

module EquipmentTray(cutoutHatch)
{
//translate([-0.9*motorTECutout,0,0])
   {
   hatchLen = HatchLen;
   noseJoinThick = 0.6;
   airductDiaFraction = 0.4;
   airductDia = airductDiaFraction*trayNoseHeight;
   if(motor && !camera)
      {//air intakes
         translate([-0.75*noseLen,trayFloorY,0])
            {
            airInTakeLen = 60;
            airInTakeDiaFraction = 0.25;
            airIntakeDia = airInTakeDiaFraction*trayNoseHeight;
            translate([airIntakeDia/2,trayNoseHeight/2,trayNoseWidth/2])
               rotate([0,0,-135])rotate([0,90,0]) scale([1,1,1])
               {
               scale([1,1.5,1])
               cylinder(d=airIntakeDia, h=airInTakeLen);
               }
            }
         
      }
   
   intersection()
      {
      FuseInner();
      union()
         {
            hull()
               {//battery and nose weight space
               noseFrontPos = (camera?12-0.9*noseLen:-0.75*noseLen);
               translate([noseFrontPos,trayFloorY,0])
                  {
                  translate([airductDia/2,trayNoseHeight/2,trayNoseWidth/2])
                     rotate([0,90,0]) scale([1,1/airductDiaFraction,1])
                     {
                     cylinder(d=airductDia, h=trayBodyLen);
                     sphere(d=airductDiaFraction*trayNoseHeight);
                     }
                     
               cube([0.1,trayNoseHeight,trayNoseWidth/2]);
               if(!camera)
               translate([-noseWeightD/2,0.4*trayNoseHeight,0])
                    {
                    sphere(d=noseWeightD);
                    //rotate([0,90,0]) cylinder(d=noseWeightD,h= noseWeightD/2+0.5);
                    }
           weightO2Dia = pow(0.5,1/3)*noseWeightD; //diameter goes as cube root of volume (weight)
           translate([-weightO2Dia/2,0.4*trayNoseHeight,noseWeightD/2+weightO2Dia/2])
              sphere(d = weightO2Dia);
                  }
                translate([-0.33*0.75*noseLen,trayFloorY,0])
                   cube([0.1,trayHeight,trayNoseWidth/2]);
            }
         //intersection()
            {
            union()
               {
               hull()
                  {
echo("noseCutX=",noseCutX);
               trayRearPos = 27.7584+noseJoinThick+noseLen/2;
                  translate([-0.33*0.75*noseLen,trayFloorY,0])
                      cube([0.1,trayHeight,trayNoseWidth/2]);
                  translate([noseCutX+noseJoinThick,trayFloorY,0])
                     cube([0.1,trayHeight,trayNoseWidth/2]);
                  //translate([-0.6*noseLen,-0.17*bodyH,0])
                  translate([-0.0*noseLen,trayFloorY,0])
                     cube([1*trayBodyLen,trayHeight,trayWidth/2]);
                  translate([-0.0*noseLen,trayFloorY,0])
                     cube([hatchCutoutRearPos,trayHeight,duoStripWidth/2]);
                  }
               hull()
                  {
                  translate([trayHeightRear1Pos,trayFloorY,0])
                     #cube([0.1,trayHeightRear1,trayWidth/2]);

                  translate([0.85*trayMaxLen,trayFloorY,0])
                     #cube([0.1,trayHeightRear2,0.8*trayWidth/2]);
                  translate([trayMaxLen,0,0])
                     cylinder(d=servoLeadThick,h=0.8*trayWidth/2);
                  }
               //#translate([HatchFrontX,trayFloorY,0]) cube([hatchLen,1.0*trayHeight,1.0*trayWidth/2]);
               }
            //translate([noseCutX+noseJoinThick,-bodyH,-bodyWidth]) cube([bodyLen,2*bodyH,2*bodyWidth]);
            }
         }
      }
   if(cutoutHatch)
      {
      HatchCutout(cutoutHatch);
      }
   }
}


module FlyingWing()
{
FlyingWingHalf();
mirror([0,0,1])
   FlyingWingHalf();
}

module WingTip() //testing only
{
intersection()
   {
   difference()
      {
      union()
         {
         translate([0,0,bodyWidth/2])
         rotate([0,0,-90])
            {
            Wing();
   //         Elevon();
            }

   //      blendFoils();
         }
      MainSpar(true);
      }

   difference()
      {
      translate([-rootChord,-2*rootChord,wingLen/2+bodyWidth/2])
         cube([4*rootChord,4*rootChord,wingLen/2+finThick+2*tipChord*tan(finAngle)]);
  // translate([wingLen*sin(-dihedral),wingLen*sin(sweep),wingLen])
      translate([wingLen/2*tan(sweep),wingLen/2*tan(dihedral)+0.01*(rootChord+tipChord),wingLen/2+bodyWidth/2])
         {
         translate([0.1*(rootChord+tipChord)/2,0,0]) sphere(alignersD/2);
         translate([0.32*(rootChord+tipChord)/2,0,0]) sphere(alignersD/2);
         translate([0.66*(rootChord+tipChord)/2,0,0]) sphere(alignersD/2);
         #translate([0.56*(rootChord+tipChord)/2,0.28*(rootChord+tipChord)/2*sin(washout)-0.18*elevD,-tipJoinerLen/2])
            cylinder(d=tipJoinerD,h=tipJoinerLen);
         #translate([0.2*(rootChord+tipChord)/2,0.2*(rootChord+tipChord)/2*sin(washout)-0.18*elevD,-tipJoinerLen/2])
            cylinder(d=tipJoinerD,h=tipJoinerLen);
         }
      }
   }
}

module WingShell()
{
translate([0,0,bodyWidth/2])
rotate([0,0,-90])
	{
   Wing();
   Elevon();
   if(flaps)
      Elevon(0,1);
   if(!bottomHinges)
      {
      WingServoGuard(true);
      if(flaps)
         WingServoGuard(true,flap = true);
      }
	}
}

module FlyingWingHalf(assemble = 1)
{
difference()
   {
   union()
      {
      translate([0,0,bodyWidth/2])
      rotate([0,0,-90])
         {
         Wing();
         if(assemble)
            {
            Elevon();
            if(flaps)
               Elevon(0,1);
            WingServoGuard(true);
            if(flaps)
               WingServoGuard(true,flap = true);
           }
         }

      Fuselage();
      }
   translate([0,0,bodyWidth/2])
      rotate([0,0,-90])
         {
         ServoCutout();
         if(flaps)
            FlapServoCutout();
         }
   //MainSpar(true);
   //Antenna tubes
//   translate([0.07*rootChord,0,bodyWidth/2])
   antennaX =  27.7584;
   translate([antennaX+antennaD,sparJoinerD+antennaD/2,trayWidth/2-antennaD/2])
      rotate([0,0.7*sweep,0])
      #cylinder(d=antennaD,h=antennaLen,center=false);
   translate([0,-0.5*antennaD-0.8,0.3*bodyWidth/2])
      rotate([0,82,0])
      #cylinder(d=antennaD,h=rootChord);
   }
}

module WingBlankHalf(offsetr = 0)
{
rotate([0,0,-90])
   {
   RootWingBlank(offsetr);
   translate([0,0,bodyWidth/2])
         WingBlank(offsetr);
   }
}

echo("sparChordPos=",sparChordPos);
module MainSpar(webbing = false,sectionWebbing1 = -1, sectionWebbing2 = -1)
{
if(tubeSpar || rearTubeSpar)
   {
   chordPos = rearTubeSpar ? rearSparChordPos : sparChordPos;
   sparLenInWing = sparLenLeadingEdge*cos(sweep);
   sparLenO2 = min(sparLenInWing+bodyWidth/2,maxTubeSparLen/2);
   echo("Tube SparLen = ",2*sparLenO2);
   webbingGapAtSectionJoin = 1.2; //3 layers each side
   webbingIntoFuse = 0.1*bodyWidth;
   webbingBeyondSparEnd = 4;
   webbingLen = sparLenO2+webbingBeyondSparEnd;
   sparCenterHeight = tubeSpar?0.5*tan(dihedral)*sparLenInWing+0.0072*rootChord+sparHeightAdj-18*sin(sweep-22):
      0.5*tan(dihedral)*sparLenInWing+2*(rootChord-chordPos)/rootChord+rearTubeSparHAdj;
//sparCenterHeight = 0.5*tan(dihedral)*sparLenInWing+0.0072*rootChord+sparHeightAdj-18*sin(sweep-22);
echo("sparCenterHeight=",sparCenterHeight);
   //intersection()
      {
      //#WingBlankInner();
	#translate([chordPos,sparCenterHeight,0])
         {
         cylinder(d=sparD, h= sparLenO2);
         if(webbing)
            difference()
               {
               translate([0,0,webbingLen/2])
                  cube([0.1,30,webbingLen],true);
               union()
                  {
                  cube([30,30,bodyWidth-webbingIntoFuse],true);
                  if(sectionWebbing1 >= 0)
                     translate([0,0,sectionWebbing1])
                        cube([30,30,webbingGapAtSectionJoin],true);
                  if(sectionWebbing2 >= 0)
                     translate([0,0,sectionWebbing2])
                        cube([30,30,webbingGapAtSectionJoin],true);
                  }
               }
            }
      }
   }
if(!tubeSpar)
   {
   sparLenO2Prj = bodyWidth/2+printerMaxZ*(nSections-1)+0.2*printerMaxZ; //0.5*0.8*span;
   //sparLenO2 = min(maxSparLenO2,sparLenO2Prj/cos(sweep));
   sparLenO2 = sparLenO2Prj/cos(sparSweep);
   sparLenO2LimitedPrj = sparLenO2*cos(sparSweep);
   echo("Swept spar half Len=",sparLenO2);
   sparLenInFuseO2 = bodyWidth/2/cos(sparSweep);
   webbingGapAtSectionJoin = 1.2; //3 layers each side
   webbingIntoFuse = 0.1*bodyWidth; //projected len
   webbingBeyondSparEnd = 4;
   webbingLen = sparLenO2LimitedPrj+webbingBeyondSparEnd;
   translate([sparChordPos,sparRaiseFrac*rootThick,bodyWidth/2])
      {
      //cylinder(d=sparD, h= sparLenO2);
      rotate([0,sparSweep,0])
      rotate([-dihedral+atan(0.25*(rootThick-tipThick)/(wingLen/cos(sparSweep))),0,0])
         translate([0,0,-sparLenInFuseO2])
         union()
            {
            cube([sparThick,sparH,sparLenO2]);
            //flex space to protect fuse in crashes
            if(fuselage)
               {
               lenZ = 0.75*(bodyWidth/2-0.5*trayWidth)/cos(sparSweep);
               startZ =  (bodyWidth/2+rootWingOnBodyWidth)/cos(sparSweep)-lenZ;
               rCut = 0.5*lenZ*lenZ/sweptSparFlexSpace + sweptSparFlexSpace;
               translate([-(sweptSparFlexSpace),0,startZ])
                  {
                  difference()
                     {
                     cube([sweptSparFlexSpace,sparH,lenZ]);
                     translate([sweptSparFlexSpace-rCut,sparH,0])
                        rotate([90,0,0])
                        #cylinder(r = rCut, h=sparH,$fn=128);//$fs=0.5);
                     }
                  }
               }
            }
      if(!fuselage && webbing)
         multmatrix(m = [ [1, 0, tan(sparSweep), 0],
                          [0, 1, tan(dihedral), 0],
                          [0, 0, 1, 0],
                          [0, 0, 0,  1]
                          ])
         translate([sparThick/2,0,-bodyWidth/2])
         difference()
            {
            translate([0,0,webbingLen/2])
               cube([0.1,30,webbingLen],true);
            union()
               {
               cube([30,30,bodyWidth-2*webbingIntoFuse],true);
               if(sectionWebbing1 >= 0)
                  translate([0,0,sectionWebbing1])
                     cube([30,30,webbingGapAtSectionJoin],true);
               if(sectionWebbing2 >= 0)
                  translate([0,0,sectionWebbing2])
                     cube([30,30,webbingGapAtSectionJoin],true);
               }
            }
         //Fuselage connector spar
         translate([-bodyWidth/2*tan(sparSweep),-bodyWidth/2*tan(dihedral),-bodyWidth/2])
         #translate([-sparJoinerD/2,-1,0])
            //cylinder(d=sparJoinerD, h=bodyWidth/2+rootWingOnBodyWidth+0.01);
            cube([sparThick,sparH, bodyWidth/2+rootWingOnBodyWidth+0.01]);
         }
   }
}

module FuselageShell()
{
blendFoils();
FuselageRearSkid();
}

module blendFoils(H = bodyWidth/2, yMax = bodyH, offsetR = 0)
{
//airfoil = rootChord*MH_45;

//v1 = S_(2,3,1,airfoil);
//v1 = 2*airfoil;
xScale = (noseLen+rootChord)/rootChord;
yScale = yMax/rootThick;//2.2;
//echo(yScale);
ms = [[xScale,0],[0,yScale]];

profile1 = airfoil*ms;  //Profile at centre of fuselage
profile2 = airfoil;     //Profile at wing root

//This scale is a hack because I don't know how to achieve the effect of using offset() on a vector!
scale([1+4*offsetR/(noseLen+rootChord),1+2*offsetR/yMax,1+offsetR/H])
   translate([-noseLen-motorTECutout,0,0])
      poly3dFromVectors(interp());

//function shape(z) = -pow(z,noseFlatness)*(pow(z,noseFlatness)-2);
function shapeFunc(z) = pow((sin((z-0.5)*(180-sweep/noseFlatness))+1)/2,noseFlatness);
function shape(z) = (shapeFunc(z)-shapeFunc(0))/(shapeFunc(1)-shapeFunc(0)); //normalise output to [0,1)
//function shape(z) = z; 

//Interpolate between 2 profiles using the shape function
function interp() = 
   [
   for (i=[0: 0.02 : 1+0.001])
      let(h = i*(H))
      let(f = shape(i))
      let(x = f*(noseLen)+f*motorTECutout)
      TransXYZ(x,0,h, vec3D(vecInterp(profile1,profile2,f),0))
   ];
}

function FinNACA(chord = 100,N = len(airfoil), thick = NACAFinThick) = 
   airfoil_data([-NACAFinCamber,0.33,thick], chord, N, false);


finTopChordFrac = 0.65;  //fraction of chord at fin base
finFracTipChord = 0.63; //increasing this above 0.63 can cause CGAL errors
finBaseChord = finFracTipChord*tipChord;
finHeightFrac =  NACAFinAspectRatio;
finHeight = finBaseChord*finHeightFrac;
finSweep = 22;
finSweepExtraForPrinting = 10; //S3D does not print the full TE. Stops when it gets too thin. This compensates somewhat.
finTotalSweep = finSweep + finSweepExtraForPrinting;
finLen = finHeight/sin(NACAFinAngle);
finLEFlatness = 1.8;    //control the shape of the blend from the wing tip into the fin

echo("fin base y=",span/2+finSpan,"finBaseChord=",finBaseChord,"finBaseOffset=",(span/2+finSpan)*tan(sweep)+tipChord-finBaseChord,"finDihedral=",NACAFinAngle);
echo("fin tip y=",span/2+finSpan+finLen,"finTipChord=",finTopChordFrac*finBaseChord,"finBaseOffset=",(span/2+finSpan)*tan(sweep)+tipChord-finBaseChord+finLen*sin(finTotalSweep));


module BlendedFinTip(H = finSpan, NACAfin = true, endFlattener = false)
{
//xScale = tipChord/rootChord;
//yScale = xScale;//yMax/rootThick;//2.2;
//echo(yScale);
//ms = [[xScale,0],[0,yScale]];
   
finRaise = 0.1*tipChord;
finBlendAngle = 48;
finLEAngle = 60;
   

   
ms = [[1,0],[0,-1]]; //make it go anti-clockwise like airfoil
profile1 = (tipChord/rootChord)*airfoilAligned;  //Profile at tip of wing
//profile2a = finFracTipChord*profile1;     //Profile at base of fin
//profile2 = profile2a*ms;
profile2 = FinNACA(finBaseChord, len(profile1))*ms;
echo("len(airfoil)=",len(airfoil));
echo("len(profile1)=",len(profile1));
echo("len(profile2)=",len(profile2));
//profile200 = FinNACA(100,50)*ms;
//echo("profile200 =",profile200);
interpLen = tipChord-finFracTipChord*tipChord;

//This scale is a hack because I don't know how to achieve the effect of using offset() on a vector!
//scale([1+4*offsetR/(noseLen+rootChord),1+2*offsetR/yMax,1+offsetR/H])
//   translate([-noseLen,0,0])
      //rotate([NACAFinAngle,0,0])
      //hull()
      //{
      //poly3dFromVectors(interp(H));
      //poly3dFromVectors(interp(H+1.25*finThick));
      //}
//function shapeFuncx(z) = -pow(z,finLEFlatness)*(pow(z,finLEFlatness)-2);
//function shapeFunc(z) = pow(1-sqrt(1-pow(z,2)),1);
function shapeFunc(z) = pow(z,3);
function shapeFuncx(z) = pow((sin((z-0.5)*(180-z*finSweep/finLEFlatness))+1)/2,finLEFlatness);
function shape(z) = (shapeFunc(z)-shapeFunc(0))/(shapeFunc(1)-shapeFunc(0)); //normalise output to [0,1)
function shapex(z) = (shapeFuncx(z)-shapeFuncx(0))/(shapeFuncx(1)-shapeFuncx(0)); //normalise output to [0,1)
//function shape(z) = z; 

finSweepWithTaper = atan(tan(finSweep)+ tipChord*(1-finTopChordFrac)/finHeight);

//Interpolate between 2 profiles using the shape function
function interp(hz) = 
   [
   for (i=[0: 1/20 : 1+0.00001])
      let(f = shape(i))
      let(h = i>1?-(hz+finHeight*cos(NACAFinAngle)):-i*(hz))
      let(x = shapex(i)*interpLen-0*h*tan(finSweep)+ (i>1? finBaseChord*(1-finTopChordFrac):0) )
      //let(x = shapex(i)*interpLen-0*h*tan(finSweep)+ (i>1? 0*finBaseChord*(1-finTopChordFrac):0) )
      let(y = i>1? finRaise+finHeight*sin(NACAFinAngle):f*finRaise)
      TransXYZ(x,y,h, Rx_(i*NACAFinAngle,vec3D(i>1?finTopChordFrac*profile2:vecInterp(profile1,profile2,shapex(i)),0)))
//      T_(x,y,h, vec3D(vecInterp(profile1,profile2,f),0))
   ];

finVec = [[0,0],[1,0],[finHeightFrac*tan(finSweep)+finTopChordFrac,finHeightFrac],[finHeightFrac*tan(finSweep),finHeightFrac]];
//intersection()
   {
if(NACAfin)
   {
   mirror([0,0,1])
      {
      //rotate([NACAFinAngle,0,0])
      blendedVecs = interp(H);
         multmatrix(m = [ [1,0 , -1*tan(finSweep+finSweepExtraForPrinting), 0],
                     [0, 1,0 , 0],
                     [0, 0, 1, 0],
                     [0, 0, 0,  1]
                              ])
         {
         poly3dFromVectors(blendedVecs);
         //translate([0*(interpLen+H*tan(finSweep)),1*finRaise,-H])

         translate([1*finBaseChord*(1-finTopChordFrac)+interpLen,1*finRaise,-H])
         rotate([180+NACAFinAngle,0,0])
            translate([0,0,finHeight])
               multmatrix(m = [ [1,0 , tipChord*(1-finTopChordFrac)/finHeight, 0],
                     [0, 1,0 , 0],
                     [0, 0, 1, 0],
                     [0, 0, 0,  1]
                              ])

            FinTipUncambered(profile = profile2*[[finTopChordFrac,0],[0,finTopChordFrac]],chord = finBaseChord*finTopChordFrac);
         }

      //NACA fin foil
//      translate([interpLen+H*tan(finSweep),1*finRaise,-H])      
//      rotate([180+NACAFinAngle,0,0])
//      //Shear to get sweep
//         multmatrix(m = [ [1,0 , tan(finSweep+finSweepExtraForPrinting), 0],
//                     [0, 1,0 , 0],
//                     [0, 0, 1, 0],
//                     [0, 0, 0,  1]
//                              ])
//         {
//         linear_extrude(height = finHeight, convexity = 5, scale=[finTopChordFrac,finTopChordFrac],$fn=len(profile2)-1)
//            polygon(profile2);
//            
//         //tip of fin
//         //N.B. using rotate_extrude() here only works properly if
//         #translate([0,0,finHeight])
//         if(NACAFinCamber==0)
//            {
//            //if(partToGenerate == "Fin") //prevents CGAL assertion in pre-2017 versions of OpenSCAD when rendering TipWing
//               FinTipUncambered(profile = profile2*[[finTopChordFrac,0],[0,finTopChordFrac]],chord = finBaseChord*finTopChordFrac);
//            }
//         else
//            FinTipCambered(profile = profile2*[[finTopChordFrac,0],[0,finTopChordFrac]],chord = finBaseChord*finTopChordFrac);
//         }
      }
   }
else //Semi-blended flat plate finelse //Semi-blended flat plate fin
   {   
   finVec = [[0,0],[1,0],[finHeightFrac*tan(finSweep)+finTopChordFrac,finHeightFrac],[finHeightFrac*tan(finSweep),finHeightFrac]];
   ro = 0.042*tipChord;
   //translate([interpLen+H*tan(finSweep),0.6*finRaise,H-finThick-tan(NACAFinAngle)*finBaseChord*thickPercent/100*0.6])
   //rotate([NACAFinAngle,0,0])
   translate([interpLen+H*tan(finSweep),1*finRaise,H])
   multmatrix(m = [ [1, 0, tan(finLEAngle), 0],
                    [0, 1, tan(finBlendAngle), 0],
                    [0, tan(NACAFinAngle), 1, 0],
                    [0, 0, 0,  1]
                    ])
   linear_extrude(height = endFlattener?10*finThick:finThick, convexity = 5,$fn=100)
      hull()
         {
         offset(ro)
            offset(-ro)
               polygon(finBaseChord*finVec);
         polygon(profile2);
         }
//   cube([finBaseChord,finHeightFrac*finBaseChord,finThick]);
      }
   }
}

module FinTipCambered(profile, chord)
{
maxTipHeight = 1.5*NACAFinThick*chord/2;
xTipFrac = 0.1;
function shapeFunc(z) = 0.5*tan(88*z)+10*pow(z,3);//pow(z,5);
//function shapeFunc(z) = pow((sin((z-0.5)*(180-sweep/noseFlatness))+1)/2,noseFlatness);
function shape(z) = (shapeFunc(z)-shapeFunc(0))/(shapeFunc(1)-shapeFunc(0)); //normalise output to [0,1)
//function shape(z) = z; 
maxXShift = 0.67*chord*(1-xTipFrac)/2;
   
profileChordAtMaxTipHeight = chord+(maxTipHeight)*(chord-(chord/finTopChordFrac))/finHeight;
echo("profileChordAtMaxTipHeight=",profileChordAtMaxTipHeight);
echo("chord=",chord);
//Interpolate between 2 profiles using the shape function
function interp(hz,profile1,profile2) = 
   [
   for (i=[0: 1/50 : 1+0.001])
      let(f = shape(i))
      let(h = i*(hz))
      let(x = f*maxXShift)
      TransXYZ(x,0,h, vec3D(vecInterp(profile1,profile2,f)*[[(i*profileChordAtMaxTipHeight+(1-i)*chord)/chord,0],[0,1]],0))
   ];
profile2 = profile*[[xTipFrac,0],[0,xTipFrac/1]];
scale([1,1,2])
   poly3dFromVectors(interp(maxTipHeight,profile,profile2));
}

module FinTipUncambered(profile, chord)
{
scale([1,1,4])
rotate([90,0,-90])
rotate_extrude(angle=180, convexity=4,$fn=64)
rotate([0,0,-90])
intersection()
   {
   polygon(profile);
   square([chord,3*NACAFinThick*chord/2]);
   }
}

//   for (i=[0: 1/20 : 1+0.001])
//      echo("i=",i);
//BlendedFinTip();

//BlendedFinTip(NACAfin=false,fin = false);
//translate([0,0,-0.4]) NACAfin(NACAfin=false,tip=false);

//polygon(FinNACA());
//polygon(airfoil);

//BlendedFinTip();

module Wing()
{
if(flatPlateTip)
   translate([wingLen*tan(-dihedral),wingLen*tan(sweep),wingLen-finThick])
      Fin(tipChord);
else
   translate([wingLen*tan(-dihedral),wingLen*tan(sweep),wingLen])
      rotate([0,0,90+washout])
         BlendedFinTip();

difference()
   {
   WingBlank();
//   #translate([-0.0072*rootChord,0.65*rootChord,])
//      cylinder(d=sparD, h= 0.60*wingLen);
   Elevon(1);
   if(flaps)
      Elevon(1,1);

   if(flatPlateTip)
      translate([wingLen*tan(-dihedral),wingLen*tan(sweep),wingLen])
      //Flatten the tip of the wing for printing purposes
         Fin(tipChord,1);
   //ServoCutout();
   }
}


module ServoCutout() //in Wing coordinate system
{
//Servo cutout

#translate([servoThick/2-0.5*servoChordPos/wingChordAtElevonStart+elevonStartSpan*tan(-dihedral)-elevonServoWellDepthAdjust,
         servoChordPos,
         elevonStartSpan-servoHHornTop+(servoHHornTop-servoHHornBottom)/2])
      rotate([0,-dihedral,washout/2+1.5-0.8]) //+2
      rotate([0,0,90])
         Servo();

}

module FlapServoCutout() //in Wing coordinate system
{
   //Servo cutout
#translate([servoThick/2-0.5*flapServoChordPos/wingChordAtFlapStart+flapStartSpan*tan(-dihedral)-flapServoWellDepthAdjust,
         flapServoChordPos,
         flapStartSpan-servoHHornTop+(servoHHornTop-servoHHornBottom)/2])
      rotate([0,-dihedral,washout/2+1])
      rotate([0,0,90])
         Servo(flap = true);
}

module Fin(tipChord,endFlattener = 0)
{
ro = 0.042*tipChord;
inner = 0.65*tipChord - ro;

rotate([0,0,90])
translate([tipChord-inner,0.04*tipChord,0])
rotate([0,0,washout])
//Shear to get sweep
multmatrix(m = [ [1, 0,0 , 0],
                 [0, 1, 0, 0],
                 [0, tan(finAngle), 1, 0],
                 [0, 0, 0,  1]
                 ])

linear_extrude(height = endFlattener?10*finThick:finThick, convexity = 5,$fn=100)
   if(endFlattener)
      {
      polygon([[-tipChord,-tipChord],[-tipChord,tipChord],[tipChord,tipChord],[tipChord,-tipChord]]);
      }
   else
      {
intersection()
   {
      offset(r=-ro)
      {
      offset(r=ro)
      offset(r = ro)
      polygon([[-0.3*inner,-0.00*inner],[0.57*inner,0.00*inner],[0.9*inner,0.0*inner],[0.94*inner,0.0*inner],[1.3*inner,(0.8+0.15)*inner],
         [0.6*inner,(0.7+0.15)*inner],[0.1*inner,0.015*inner]]);
      }
   //polygon([[-inner,0],[2*inner,0],[2*inner,3*inner],[-inner,3*inner]]);
   //WingBlank();
   }

   }
}

module Elevon(cutOut = 0, flap = 0)
{
thick = flap ? flapD: elevD;
startSpan = flap ? flapStartSpan : elevonStartSpan;
length = flap ? flapLen : elevonLen;
bevel = (flap && bottomHinges) ? maxFlapUp : maxElevonDown;
difference()
   {
   intersection()
      {
      //Shear to get sweep
      multmatrix(m = [ [1, 0, tan(-dihedral), 0],
                       [0, 1, tan(sweep), 0],
                       [0, 0, 1, 0],
                       [0, 0, 0,  1]
                       ])
      linear_extrude(height = wingLen, convexity = 5,twist = -washout, scale=[tipScale,tipScale], $fn=100)
         intersection()
         {
         if(!cutOut)
            {
            rotate([0,0,90])
               polygon(airfoil);
            }

         //translate([-0.15*thick,elevonRootPos]) //MH_45
         translate([-0.1*thick,elevonRootPos]) //PW51
         offset(r=cutOut?hingeGap:0)
         union()
            {
            if(elevonHingeRound)
               {
               circle(d=thick);
               translate([-thick/2,0])
               square([thick,2*elevonRootLen]);
               }
            else
               {
               translate([-thick/2,0])
                  if(bottomHinges)
                   polygon([[0,cutOut?0:thick*sin(bevel)],[thick,0],[thick,2*elevonRootLen],[0,2*elevonRootLen]]);
                  else
                   polygon([[0,0],[thick,cutOut?0:thick*sin(bevel)],[thick,2*elevonRootLen],[0,2*elevonRootLen]]);
               }
            }
         }
      translate([-rootChord/2,0,startSpan + (cutOut?0:elevonStartGap)])
         cube([rootChord,8*rootChord,length-(cutOut?0:elevonStartGap+elevonEndGap)]);
      }
   if(!cutOut)
      translate([wingLen*tan(-dihedral),wingLen*tan(sweep),wingLen-1.5*hingeGap-0.01*rootChord*tan(finAngle)-finThick])
         Fin(tipChord,1);
   if(!cutOut)
      translate([0,0,-bodyWidth/2])rotate([0,0,90])Hinges();
   }
//horn
if(!cutOut)
   {
   //linear_extrude below is going to reduce hornH more the further out on the wing the horn is.
   //We want hornH fixed, so this compensates.
   wingPosScaleFactor = (1+(startSpan/wingLen)*(1-tipScale));

   hornHScaled = hornH*wingPosScaleFactor;
   hornTipRScaled = hornTipR*wingPosScaleFactor;
   hornHoleDScaled = hornHoleD*wingPosScaleFactor;
   intersection()
      {
      multmatrix(m = [ [1, 0, tan(-dihedral), 0],
                       [0, 1, tan(sweep), 0],
                       [0, 0, 1, 0],
                       [0, 0, 0,  1]
                       ])
      linear_extrude(height = wingLen, convexity = 5,twist = -washout, scale=[tipScale,tipScale], $fn=100)
      union()
         {
         difference()
            {
            offset(r=0)
            union()
               {
               if(bottomHinges)
                  polygon([[0,elevonRootPos+(elevonHingeRound?0:1.1*thick*sin(bevel))],[0.0,0.98*rootChord],
                     [-0.55*hornHScaled,0.84*rootChord],
                     [-(hornHScaled-hornTipRScaled),elevonRootPos+2*hornTipRScaled],
                     [-(hornHScaled-hornTipRScaled),elevonRootPos]]
                     );
               else
                  polygon([[0,elevonRootPos+(elevonHingeRound?0:1.1*thick*sin(bevel))],[0.0,0.98*rootChord],
                     [0.55*hornHScaled,0.84*rootChord],
                     [hornHScaled-hornTipRScaled,elevonRootPos+2*hornTipRScaled],
                     [hornHScaled-hornTipRScaled,elevonRootPos]]
                     );
               translate([(bottomHinges?-1:1)*(hornHScaled-hornTipRScaled),elevonRootPos+hornTipRScaled])
                     circle(r = hornTipRScaled);
               }
            translate([(bottomHinges?-1:1)*(hornHScaled-hornTipRScaled),elevonRootPos+hornTipRScaled])
               circle(d = hornHoleDScaled);               
            }
         }
      translate([-rootChord/2,0,startSpan + (elevonStartGap)])
         cube([rootChord,4*rootChord,hornThick]);
     }
   }
   
}

module RootWingBlank(offsetr = 0)
{
linear_extrude(height = bodyWidth/2, convexity = 5,twist = 0, $fn=100)
   rotate([0,0,90])
      offset(r= offsetr)
         polygon(airfoil);
}


module WingBlank(offsetr = 0)
{
//Shear to get sweep
multmatrix(m = [ [1, 0, tan(-dihedral), 0],
                 		  [0, 1, tan(sweep), 0],
                        [0, 0, 1, 0],
                        [0, 0, 0,  1]
                        ])
linear_extrude(height = wingLen+0.07*tipChord*tan(finAngle), convexity = 5,twist = -washout, scale=[tipScale,tipScale], $fn=100)
   rotate([0,0,90])
      offset(r= offsetr)
         polygon(airfoil);
}


module Servo(leadLen = elevonStartSpan+bodyWidth/2-servoHHornTop, flap = false)
{
connectorSpaceLen = 25; 
servoLeadDepth = servoThick-servoLeadThick-2;
servoToSpar = servoChordPos-(rearTubeSpar? rearSparChordPos : sparChordPos);
bendPosFromServo = servoToSpar/tan(sweep);
bendFrac = bendPosFromServo/leadLen;
echo("bendPosFromServo=",bendPosFromServo);
//echo("bendFrac=",bendFrac);
//overSparAngle = atan((servoLeadDepth-sparD+bendPosFromServo*tan(dihedral)+1.5)/bendPosFromServo);
overSparAngle = atan(((0.5-0.2)*sparD+bendPosFromServo*tan(dihedral))/bendPosFromServo); //Improve spar/lead guide spacing
innerOverSparAngle = overSparAngle;
forwardSparAngle = 0;//-sweep/4;
servoCutLen = servoHornLen-servoHornR;
cube([servoW,servoThick,servoH]);

translate([-servoLeadSpace,0,0])
   cube([servoLeadSpace,servoThick,servoLeadH]);

//space for inserting connector
plugSpaceAngle = atan(servoPlugW/4/connectorSpaceLen);
translate([-servoLeadSpace+connectorSpaceLen*(tan(sweep+plugSpaceAngle)-tan(sweep)),servoLeadDepth,0])
         multmatrix(m = [ [1, 0, tan(sweep+plugSpaceAngle), 0],
                          [0, 1, tan(0*dihedral-overSparAngle), 0], 
                          [0, 0, 1, 0],
                          [0, 0, 0,  1]
                       ])
   translate([0,0,-connectorSpaceLen])
    cube([servoPlugW/cos(sweep),servoPlugThick/cos(dihedral),connectorSpaceLen]);

translate([-(servoFlangeW-servoW)/2,0,servoFlangeHBottom])
   cube([servoFlangeW,servoThick,servoFlangeThick]);

translate([0,0,servoH])
   cube([servoTopW,servoThick,(bottomHinges?servoHornSlotWidth*0:0)+servoHHornBottom-servoH]);

if(bottomHinges)
   translate([servoHornDia-servoHornCenter,0,servoH])
      cube([servoHornDia,servoThick,servHornSlotBottomH+servoHornSlotWidth-servoH]);
   

translate([servoHornCenterFomLeadSide,servoThick/2,(bottomHinges?servHornSlotBottomH:servoHHornBottom)-servoTol])
   mirror([0,bottomHinges?1:0,0])
   rotate([0,0,bottomHinges?8:-elevonDifferential])
      {
      linear_extrude(height = (bottomHinges?servoHornSlotWidth:(servoHHornTop-servoHHornBottom))+servoTol, convexity = 5,$fn=100)
      offset(r = servoHornR)
      polygon([[0,0],
         [-servoCutLen*sin(servoAngleRange/2),-servoCutLen*cos(servoAngleRange/2)],
         [servoCutLen*sin(servoAngleRange/2),-servoCutLen*cos(servoAngleRange/2)]]);
      }
//-(0.05*rootChord-((servoThick-servoLeadThick)-2))/leadLen

//echo(servoCutLen*sin(servoAngleRange/2));

//if(!flap)
//if(0)
      multmatrix(m = [ [1, 0, tan(sweep), 0],
                     [0, 1, tan(0*dihedral-overSparAngle), 0],                       
                     [0, 0, 1, 0],
                     [0, 0, 0,  1]
                    ])
   {
   translate([-servoLeadSpace,servoLeadDepth,-bendFrac*leadLen])
//       cube([servoLeadW/cos(sweep),servoLeadThick/cos(dihedral),bendFrac*leadLen]);
       cube([servoPlugW/cos(sweep),servoPlugThick/cos(dihedral),bendFrac*leadLen]);
   translate([-servoLeadSpace-(1-bendFrac)*leadLen*tan(-forwardSparAngle),(1-bendFrac)*leadLen*tan(-innerOverSparAngle)+servoLeadDepth,-1*leadLen])
   multmatrix(m = [ [1, 0,tan(-forwardSparAngle), 0],
                    [0, 1, tan(innerOverSparAngle), 0], //bring lead conduit to center of profile at wing root
                    [0, 0, 1, 0],
                    [0, 0, 0,  1]
                 ])
//       cube([servoLeadW/cos(sweep),servoLeadThick/cos(dihedral),(1-bendFrac)*leadLen]);
       cube([servoPlugW/cos(sweep),servoPlugThick/cos(dihedral),(1-bendFrac)*leadLen]);
   }
}

module WingTransform(spanPos, rootChordPos)
{
//chordScale = 1*(1-(spanPos/wingLen))+tipScale*(spanPos/wingLen);
LEChordPos = spanPos*tan(sweep);
TEChordPos = LEChordPos+rootChord-(spanPos/wingLen)*(rootChord-tipChord);
chordPos = TEChordPos*(rootChordPos/rootChord)+LEChordPos*(1-(rootChordPos/rootChord));
LEMinusPos = LEChordPos-chordPos;
translate([0,0,bodyWidth/2])
   translate([chordPos+LEMinusPos,spanPos*tan(dihedral),spanPos])
      rotate([0,0,washout*spanPos/wingLen])
         translate([-LEMinusPos,0,0])
            children();


//translate([0,0,bodyWidth/2])
//   {
//   //Shear to get sweep
//   multmatrix(m = [ [1, 0, tan(-dihedral), 0],
//                          [0, 1, tan(sweep), 0],
//                           [0, 0, 1, 0],
//                           [0, 0, 0,  1]
//                           ])
//      children();
//   }
}

hingeChordPos = elevonRootPos;//-hingeLen/2;
TESweepAngle = atan(tan(sweep)-(rootChord-tipChord)/wingLen);


module HingeBox(hingePos)
{
WingTransform(hingePos, hingeChordPos)
   {
   translate([0,elevonHingePos*elevD,0])
   rotate([0,TESweepAngle,0])
      cube([hingeLen,2*elevD,hingeW],center = true);
   }
}

module Hinges()
{
//elevonEndToTip;

hingePos1 = elevonStartSpan+elevonStartGap+hingeInboardEndGap+hingeW/2+elevonStartTapeJoinHalfW;
hingePosEnd = elevonStartSpan-hingeW/2+elevonLen-hingeTipEndGap-elevonEndGap;
hinglePosMid = (hingePos1-elevonStartTapeJoinHalfW-hingeW/2+hingePosEnd)/2;

hingePosFlap = flapStartSpan+elevonStartGap+hingeInboardEndGap+hingeW/2;
hingePosFlapEnd = flapStartSpan-hingeW/2+flapLen-hingeTipEndGap-elevonEndGap;
hinglePosFlapMid = (hingePosFlap+hingePosFlapEnd)/2;

//elevonRootPos
//wingLen
//elevonStartSpan

center = true;

intersection()
   {
   render()
   difference()
      {
      WingBlankHalf(-2*skinThick);
      translate([0,-elevonHingePos*hingeThick,0])
         WingBlankHalf(-2*skinThick);
      //WingBlankHalf(-2*skinThick-hingeThick);
      }
   union()
      {
      HingeBox(hingePos1);
      HingeBox(hinglePosMid);
      HingeBox(hingePosEnd);
      if(flaps)
         {
         HingeBox(hingePosFlap);
         HingeBox(hinglePosFlapMid);
         HingeBox(hingePosFlapEnd);
         }
      }
   }
}

 
 