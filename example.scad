// ============================================================================
// Example: Toroidal Propeller with NACA Airfoil Cross-Section
// ============================================================================
// This example shows how to generate toroidal propellers using NACA airfoil
// profiles with blunted trailing edges.
//
// Open this file in OpenSCAD and press F5 to preview or F6 to render.
// ============================================================================

use <src/toroidal_propeller.scad>
$fn = 100;

// --- Example 1: NACA 2412 airfoil propeller (default) ---
// A cambered airfoil with 2% camber at 40% chord, 12% thickness.
// 5% trailing edge blunting for propeller suitability.
toroidal_propeller(
    blades = 3,
    height = 6,
    blade_length = 68,
    blade_width = 42,
    blade_thickness = 4,
    blade_hole_offset = 1.4,
    blade_attack_angle = 35,
    blade_offset = -6,
    blade_safe_direction = "PREV",
    hub_height = 6,
    hub_d = 16,
    hub_screw_d = 5.5,
    // NACA airfoil settings
    naca_profile = 2412,            // NACA 2412 cambered airfoil
    naca_blunt = 0.05,              // 5% chord trailing edge blunting
    naca_points = 80                // airfoil point resolution
);


// --- Example 2: Symmetric NACA 0012 propeller (uncomment to try) ---
// A symmetric (zero-camber) airfoil, 12% thickness.
// translate([0, 0, 20])
// toroidal_propeller(
//     blades = 4,
//     height = 8,
//     blade_length = 72,
//     blade_width = 44,
//     blade_thickness = 5,
//     blade_hole_offset = 1.4,
//     blade_attack_angle = 30,
//     blade_offset = -6,
//     blade_safe_direction = "PREV",
//     naca_profile = 0012,
//     naca_blunt = 0.03,
//     naca_points = 100
// );


// --- Example 3: Original elliptical blade (backward compatible) ---
// Set naca_profile = 0 to fall back to the original elliptical blade shape.
// translate([0, 0, 40])
// toroidal_propeller(
//     blades = 3,
//     naca_profile = 0               // 0 = use original elliptical blades
// );
