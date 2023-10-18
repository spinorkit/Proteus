thumbLen = 54.2;
thumbH = 21;
thumbR = 7.0;
thumbT = 13; //body not lens
thumbW = 26;  //including filter
thumbLensW = 25.3;
thumbConnectorXEdge = 7.8;
thumbConnectorY = 7.36;



//ThumbPro();

module ThumbPro(longLens = 0)
{
$fn = 64;
//cube([thumbLen, thumbW,thumbH]);
//cylinder(h = thumbW, )
translate([thumbR,0,thumbR])
mirror([0,1,0])
rotate([90,0,0])
linear_extrude(height = thumbT) 
   {
   offset(r = thumbR)
   square([thumbLen-2*thumbR,thumbH-2*thumbR]);
   }

//lens
translate([thumbR+thumbLen-thumbLensW,0,thumbR])
mirror([0,1,0])
rotate([90,0,0])
linear_extrude(height = longLens?2*thumbW:thumbW) 
   {
   offset(r = thumbR)
   square([thumbLensW-2*thumbR,thumbH-2*thumbR]);
   }
//connector
translate([thumbConnectorXEdge/2,0,thumbH/2])
   cube([thumbConnectorXEdge,10,thumbConnectorY],true);
}
