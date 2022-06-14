width = 25;
height = 10;
length = 25;

r = 2;
thick = 1;

$fn = 64;

linear_extrude(height=width)
difference()
{
square([length,height]);
translate([thick,thick])
offset(r)
  offset(-r)
   square(100);
}