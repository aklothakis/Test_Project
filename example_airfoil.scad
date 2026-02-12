// ============================================================================
// NACA Airfoil Profile Preview
// ============================================================================
// Use this file to visualize and compare airfoil profiles before generating
// a propeller. Open in OpenSCAD and press F5.
// ============================================================================

use <src/naca_airfoil.scad>
$fn = 100;

// --- NACA 2412 with blunted trailing edge ---
color("SteelBlue")
    naca_airfoil_2d(naca = 2412, chord = 100, blunt = 0.05, n = 80);

// --- Same airfoil without blunting (for comparison) ---
color("Tomato", 0.3)
    naca_airfoil_2d(naca = 2412, chord = 100, blunt = 0.0, n = 80);

// --- NACA 0012 symmetric (offset for comparison) ---
translate([0, -30, 0]) {
    color("SeaGreen")
        naca_airfoil_2d(naca = 12, chord = 100, blunt = 0.05, n = 80);

    color("Orange", 0.3)
        naca_airfoil_2d(naca = 12, chord = 100, blunt = 0.0, n = 80);
}

// --- NACA 4415 high-lift (offset for comparison) ---
translate([0, 30, 0]) {
    color("MediumPurple")
        naca_airfoil_2d(naca = 4415, chord = 100, blunt = 0.05, n = 80);

    color("Gold", 0.3)
        naca_airfoil_2d(naca = 4415, chord = 100, blunt = 0.0, n = 80);
}
