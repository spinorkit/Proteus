dihedral = 6;

length = 184;
width = 15;
thick = 2.5;
holeFromEnd = 10;
holeDia = 2.5;

$fn = 64;

Joiner();

//JoinerHalf();

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