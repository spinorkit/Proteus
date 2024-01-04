TEExt = 3;
width = 20+TEExt;
thick = 3.8;
tipLen = 26;
eleLen = 172;
cutAng = 90-55;

TELen = eleLen-width*tan(cutAng);

offsetY  = 20;

$fn = 128;

//EleExtX2();

Cut(){EleExtX2();}

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
}

//TipExt();

top = false;

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
   polygon([[0,0],[width, thick/2], [width, -thick/2]]);

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