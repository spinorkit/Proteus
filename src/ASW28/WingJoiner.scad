dihedral = 6;

length = 184;
centerLen = 81;
endLen = (length-centerLen)/2;

width = 16;
thick = 2.5;
holeFromEnd = 10;
holeSep = 28;
holeDia = 2.5;

$fn = 64;

Joiner2();

//JoinerHalf();

//JoinerEnd();

module JoinerEnd()
{
translate([-endLen/2,0,0])
rotate([0,dihedral/2,0])
translate([endLen/2,0,0])
difference()
   {
      cube([endLen, width, thick],true);
      translate([endLen/2-holeFromEnd, 0, -thick])
         cylinder(h = 2*thick, d = holeDia);
      translate([endLen/2-holeFromEnd-holeSep, 0, -thick])
         cylinder(h = 2*thick, d = holeDia);
   }
}

module Joiner2()
{

translate([centerLen/2+endLen/2,0,0])
   rotate([0,0,0])
      JoinerEnd();

cube([centerLen, width, thick],true);

translate([centerLen/2,width/2,0])
   rotate([90,0,0]) cylinder(h = width, r = thick/2);
translate([-centerLen/2,width/2,0])
   rotate([90,0,0]) cylinder(h = width, r = thick/2);


translate([-(centerLen/2+endLen/2),0,0])
      rotate([0,0,180])
      JoinerEnd();

}


module JoinerHalf()
{
difference()
   {
   cube([length/2, width, thick],true);
   translate([length/4-holeFromEnd, 0, -thick])
      cylinder(h = 2*thick, d = holeDia);
   }
}

module Joiner()
{

translate([length/4,0,0])
   rotate([0,0,0])
      JoinerHalf();

difference()
   {
   //rotate([90,0,0]) cylinder(h = width, r = thick/2+(0*thick+length/2)*sin(dihedral));
   //rotate([90,0,0]) cylinder(h = width, r = -thick/2+(0*thick+length/2)*sin(dihedral));
   }

translate([0,width/2,0])
   rotate([90,0,0]) cylinder(h = width, r = thick/2);

rotate([0,-dihedral,0])
translate([-length/4,0,0])
      rotate([0,0,180])
      JoinerHalf();

}