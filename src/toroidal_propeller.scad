// ============================================================================
// Ultimate Toroidal Propeller Generator — with NACA Airfoil Support
// ============================================================================
// Based on: https://github.com/RaulBejarano/Ultimate-Toroidal-Propeller-Generator
// Extended with NACA 4-digit airfoil cross-sections and trailing edge blunting.
// ============================================================================

use <naca_airfoil.scad>

eps = 1/128;
$fn = 100;

// ============================================================================
// Main module: toroidal_propeller
//
// All original parameters are preserved. New parameters control the airfoil:
//   naca_profile      : 4-digit NACA number (e.g. 2412). Set to 0 to use
//                        the original elliptical blade shape.
//   naca_blunt        : trailing edge blunt fraction (0.0 = sharp, 0.05 = 5%)
//   naca_points       : number of points per surface for airfoil resolution
// ============================================================================
module toroidal_propeller(
    blades = 3,                     // number of blades
    height = 6,                     // height (extrusion height)
    blade_length = 68,              // blade length in mm (chord-wise span)
    blade_width = 42,               // blade width in mm (radial span)
    blade_thickness = 4,            // blade thickness in mm
    blade_hole_offset = 1.4,        // blade hole offset
    blade_attack_angle = 35,        // blade attack angle
    blade_offset = -6,              // blade distance from propeller axis
    blade_safe_direction = "PREV",  // PREV or NEXT — collision avoidance
    hub_height = 6,                 // Hub height
    hub_d = 16,                     // hub diameter
    hub_screw_d = 5.5,              // hub screw diameter
    hub_notch_height = 0,           // height for the notch
    hub_notch_d = 0,                // diameter for the notch
    // --- NACA airfoil parameters ---
    naca_profile = 2412,            // NACA 4-digit (0 = original elliptical)
    naca_blunt = 0.05,              // trailing edge blunt fraction
    naca_points = 80                // airfoil resolution
){
    l = height / tan(blade_attack_angle);
    p = 2 * PI * blade_length / 2;

    difference(){
        union(){
            linear_extrude(height = height, twist = l / p * 360, convexity = 2){
                if (naca_profile > 0) {
                    naca_blades2D(
                        n = blades,
                        height = height,
                        length = blade_length,
                        width = blade_width,
                        thickness = blade_thickness,
                        hole_offset = blade_hole_offset,
                        blade_direction = blade_attack_angle > 0 ? 1 : -1,
                        offset = blade_offset,
                        blade_safe_direction =
                            blade_safe_direction == "PREV" ? 1 :
                            blade_safe_direction == "NEXT" ? -1 :
                            0,
                        naca = naca_profile,
                        blunt = naca_blunt,
                        naca_n = naca_points
                    );
                } else {
                    blades2D(
                        n = blades,
                        height = height,
                        length = blade_length,
                        width = blade_width,
                        thickness = blade_thickness,
                        hole_offset = blade_hole_offset,
                        blade_direction = blade_attack_angle > 0 ? 1 : -1,
                        offset = blade_offset,
                        blade_safe_direction =
                            blade_safe_direction == "PREV" ? 1 :
                            blade_safe_direction == "NEXT" ? -1 :
                            0
                    );
                }
            }
            cylinder(d = hub_d, h = hub_height);
        }
        translate([0, 0, -eps]){
            cylinder(d = hub_screw_d, h = hub_height + 2 * eps);
            cylinder(d = hub_notch_d, h = hub_notch_height + eps);
        }
    }
}

// ============================================================================
// NACA-based blade arrangement
//
// Each blade is a NACA airfoil cross-section oriented radially, with the
// same toroidal ring-cut logic as the original.
// ============================================================================
module naca_blades2D(n, height, length, width, thickness, hole_offset,
                     blade_direction, offset, blade_safe_direction,
                     naca, blunt, naca_n) {
    // Pre-compute inner NACA shape for collision avoidance cuts
    inner_chord = length - thickness;
    inner_raw = naca_airfoil_points(naca, inner_chord, naca_n);
    inner_pts = (blunt > 0) ?
        blunt_trailing_edge(inner_raw, blunt, inner_chord) : inner_raw;
    inner_ys = [for (p = inner_pts) p[1]];
    inner_y_center = (max(inner_ys) + min(inner_ys)) / 2;
    inner_y_height = max(inner_ys) - min(inner_ys);
    inner_y_scale = (inner_y_height > 0) ?
        (width - thickness) / inner_y_height : 1;

    for (a = [0 : n - 1]) {
        difference() {
            rotate([0, 0, a * (360 / n)]) {
                translate([offset, 0, 0])
                    naca_blade2D(
                        length = length,
                        width = width,
                        thickness = thickness,
                        hole_offset = hole_offset,
                        naca = naca,
                        blunt = blunt,
                        naca_n = naca_n
                    );
            }

            // Collision avoidance: NACA-shaped cut matching adjacent blade
            cw_ccw_mult = blade_direction * blade_safe_direction;
            rotate([0, 0, (a + cw_ccw_mult) * (360 / n)])
                translate([thickness / 2 + hole_offset + offset,
                           -inner_y_center * inner_y_scale, 0])
                    scale([1, inner_y_scale])
                        polygon(inner_pts);
        }
    }
}

// ============================================================================
// Single NACA blade cross-section
//
// The airfoil is generated with its chord along the X axis, then scaled
// so that:
//   - The chord spans blade_length
//   - The maximum width (in Y) matches blade_width
//
// The "hole" (inner cutout that makes the ring shape) is created by
// subtracting a smaller, offset copy — same approach as the original
// elliptical blade, but now using a NACA shape for the outer boundary.
// ============================================================================
module naca_blade2D(length, width, thickness, hole_offset, naca, blunt, naca_n) {
    // --- Outer airfoil ---
    raw_pts = naca_airfoil_points(naca, length, naca_n);
    pts = (blunt > 0) ? blunt_trailing_edge(raw_pts, blunt, length) : raw_pts;

    // Center vertically: cambered airfoils are asymmetric about y=0,
    // but the ring cutout must be centered at y=0 for both sides to
    // have material.
    ys = [for (p = pts) p[1]];
    y_center = (max(ys) + min(ys)) / 2;
    y_height = max(ys) - min(ys);
    y_scale = (y_height > 0) ? width / y_height : 1;

    // --- Inner airfoil (ring cutout) ---
    // Must follow the same NACA contour as the outer shape, just
    // slightly smaller, so the ring band is uniform. An elliptical
    // cutout would leave exposed tips at the leading/trailing edges.
    inner_chord = length - thickness;
    inner_raw = naca_airfoil_points(naca, inner_chord, naca_n);
    inner_pts = (blunt > 0) ?
        blunt_trailing_edge(inner_raw, blunt, inner_chord) : inner_raw;
    inner_ys = [for (p = inner_pts) p[1]];
    inner_y_center = (max(inner_ys) + min(inner_ys)) / 2;
    inner_y_height = max(inner_ys) - min(inner_ys);
    inner_y_scale = (inner_y_height > 0) ?
        (width - thickness) / inner_y_height : 1;

    difference() {
        // Outer airfoil — centered at y=0, scaled to blade_width
        translate([0, -y_center * y_scale, 0])
            scale([1, y_scale])
                polygon(pts);

        // Inner cutout — same NACA shape, reduced by thickness,
        // shifted by thickness/2 + hole_offset to match original offset
        translate([thickness / 2 + hole_offset,
                   -inner_y_center * inner_y_scale, 0])
            scale([1, inner_y_scale])
                polygon(inner_pts);
    }
}

// ============================================================================
// Original elliptical blade modules (preserved for backward compatibility)
// ============================================================================
module blades2D(n, height, length, width, thickness, hole_offset,
                blade_direction, offset, blade_safe_direction) {
    for (a = [0 : n - 1]) {
        difference() {
            rotate([0, 0, a * (360 / n)]) {
                translate([offset, 0, 0]) blade2D(
                    height = height,
                    length = length,
                    width = width,
                    thickness = thickness,
                    hole_offset = hole_offset
                );
            }

            cw_ccw_mult = blade_direction * blade_safe_direction;
            rotate([0, 0, (a + cw_ccw_mult) * (360 / n)])
                translate([length / 2 + hole_offset + offset, 0, 0])
                    scale([1, (width - thickness) / (length - thickness)])
                        circle(d = length - thickness);
        }
    }
}

module blade2D(height, length, width, thickness, hole_offset) {
    difference() {
        translate([length / 2, 0, 0])
            scale([1, width / length]) circle(d = length);

        translate([length / 2 + hole_offset, 0, 0])
            scale([1, (width - thickness) / (length - thickness)])
                circle(d = length - thickness);
    }
}

// Default render
toroidal_propeller();
