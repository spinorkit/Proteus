//Uses:
// Naca4_sweep.scad - sweep library
// Code: Rudolf Huttary, Berlin 
// June 2015
// commercial use prohibited

use <Naca_sweep.scad>

// generate polyhedron from multiple airfoil_datasets
//Just a name change for Rudolf Huttary's sweep() to avoid name clashes
module poly3dFromVectors(dat, convexity = 5) // dat - vec of vec, with vec = airfoil_data
{
sweep(dat = dat, convexity = convexity);
}

//Interpolate between two polygons specified as equal length vectors v1 and v2
//using the factor f
function vecInterp(v1, v2, f) = 
   [
   for(i=[0:len(v1)-1]) (1-f)*v1[i]+f*v2[i]
   ];


//CG calculation
//N.B. this assumes the fuselage (or at least nose of the fuselage) does not generate
//significant lift forward of the wing's Mean Aerodynamic Chord, which is probably not
//true. Still, it seems to work reasonably well in practise.
//Copied from http://rcwingcog.a0001.net/V3_testing/?i=1
// rcwingcog@gmail.com 
function CalcCGPos(CGPercent, LESweep, span, bodyWidth, rootChord, tipChord) = 
[
let(sweep_dist = tan(LESweep)*(span-bodyWidth) / 2)
let(wing_area1 = rootChord * bodyWidth * 0.5)
let(wing_area2 = (rootChord + tipChord)/2 * ((span-bodyWidth) / 2))
let(wing_area = (wing_area2 * 2 + wing_area1 * 2 )/10000)  //convert from mm^2 to dm^2
// Find LE and TE line formula for swept wing
let(le_b = 0)
let(le_a = (sweep_dist - le_b) / ((span - bodyWidth) * 0.5))
let(te_b = rootChord)
let(te_a = ((sweep_dist + tipChord) - te_b) / ((span - bodyWidth) * 0.5))
// Find helper line formula
let(mac_b0 = -tipChord)
let(mac_a0 = ((sweep_dist + tipChord + rootChord) - mac_b0) / ((span - bodyWidth) * 0.5))
let(mac_b1 = rootChord + tipChord)
let(mac_a1 = ((sweep_dist - rootChord) - mac_b1) / ((span - bodyWidth) * 0.5))

// Determine MAC using intersection of helper lines
let(mac_x = (mac_b1 - mac_b0) / (mac_a0 - mac_a1))

// Compute MAC intersection with LE and TE
let(le_mac_y = le_a * mac_x + le_b)
let(te_mac_y = te_a * mac_x + te_b)

// Compute CG
let(cg_dist_wing = le_mac_y + (te_mac_y - le_mac_y) * CGPercent / 100)
let(cg_dist_fuse = rootChord * CGPercent / 100)
let(d = cg_dist_wing - cg_dist_fuse)
let(x1 = d*(wing_area2/(wing_area2 + wing_area1)))
let(cg_dist_from_rootLE = cg_dist_fuse + x1)
[cg_dist_from_rootLE,wing_area]
];
