// ============================================================================
// NACA 4-Digit Airfoil Generator for OpenSCAD
// ============================================================================
// Generates NACA 4-digit airfoil profiles (e.g. NACA 2412) with optional
// trailing edge blunting suitable for propeller blades.
//
// NACA 4-digit encoding: MPXX
//   M = maximum camber (% of chord)          e.g. 2 -> 0.02
//   P = position of max camber (tenths)      e.g. 4 -> 0.4
//   XX = maximum thickness (% of chord)      e.g. 12 -> 0.12
// ============================================================================

// --- NACA thickness distribution at position x/c ---
// Standard NACA formula for half-thickness
// t = max thickness as fraction of chord (e.g. 0.12)
// xc = x/c position along chord [0..1]
function naca_yt(t, xc) =
    5 * t * (
        0.2969 * sqrt(xc)
      - 0.1260 * xc
      - 0.3516 * pow(xc, 2)
      + 0.2843 * pow(xc, 3)
      - 0.1015 * pow(xc, 4)   // closed trailing edge coefficient
    );

// --- NACA mean camber line ---
// m = max camber as fraction of chord
// p = position of max camber as fraction of chord
// xc = x/c position along chord [0..1]
function naca_yc(m, p, xc) =
    (p == 0 || m == 0) ? 0 :
    (xc <= p) ?
        (m / pow(p, 2)) * (2 * p * xc - pow(xc, 2)) :
        (m / pow(1 - p, 2)) * ((1 - 2 * p) + 2 * p * xc - pow(xc, 2));

// --- Camber line gradient (dy_c/dx) ---
function naca_dyc(m, p, xc) =
    (p == 0 || m == 0) ? 0 :
    (xc <= p) ?
        (2 * m / pow(p, 2)) * (p - xc) :
        (2 * m / pow(1 - p, 2)) * (p - xc);

// --- Camber line angle theta ---
function naca_theta(m, p, xc) = atan(naca_dyc(m, p, xc));

// --- Upper surface point ---
function naca_upper(m, p, t, xc) = [
    xc - naca_yt(t, xc) * sin(naca_theta(m, p, xc)),
    naca_yc(m, p, xc) + naca_yt(t, xc) * cos(naca_theta(m, p, xc))
];

// --- Lower surface point ---
function naca_lower(m, p, t, xc) = [
    xc + naca_yt(t, xc) * sin(naca_theta(m, p, xc)),
    naca_yc(m, p, xc) - naca_yt(t, xc) * cos(naca_theta(m, p, xc))
];

// ============================================================================
// Cosine-spaced x/c distribution for better leading edge resolution
// n = number of points along one surface
// Returns list of x/c values from 0 to 1
// ============================================================================
function cosine_spacing(n) = [
    for (i = [0 : n])
        (1 - cos(i * 180 / n)) / 2
];

// ============================================================================
// Generate raw NACA airfoil points (upper + lower, forming a closed polygon)
//
// naca_digits : 4-digit NACA number (e.g. 2412)
// chord       : chord length in mm
// n           : number of points per surface (more = smoother)
//
// Returns a polygon point list tracing: TE -> upper -> LE -> lower -> TE
// ============================================================================
function naca_airfoil_points(naca_digits, chord, n = 80) =
    let(
        // Parse the 4 digits
        d1 = floor(naca_digits / 1000),
        d2 = floor((naca_digits % 1000) / 100),
        d34 = naca_digits % 100,
        m = d1 / 100,           // max camber fraction
        p = d2 / 10,            // max camber position fraction
        t = d34 / 100,          // max thickness fraction
        xs = cosine_spacing(n)
    )
    // Upper surface from TE (x=1) to LE (x=0)
    concat(
        [ for (i = [n : -1 : 0]) naca_upper(m, p, t, xs[i]) * chord ],
        // Lower surface from LE (x=0) to TE (x=1), skip LE duplicate
        [ for (i = [1 : n]) naca_lower(m, p, t, xs[i]) * chord ]
    );

// ============================================================================
// Trailing Edge Blunting
//
// Truncates the airfoil at a given fraction of chord from the trailing edge,
// creating a blunt (flat) trailing edge. This is common for propeller blades
// to improve structural integrity and manufacturability.
//
// points       : airfoil polygon points
// cut_fraction : fraction of chord to remove from TE (e.g. 0.05 = cut last 5%)
// chord        : chord length in mm
// ============================================================================
function blunt_trailing_edge(points, cut_fraction, chord) =
    let(
        cut_x = (1 - cut_fraction) * chord,
        n = len(points),
        // Filter points that are ahead of the cut line
        kept = [ for (i = [0 : n-1]) if (points[i][0] <= cut_x) points[i] ],
        // Find the upper and lower y-values at the cut line by interpolation
        upper_y = te_interp_y(points, cut_x, 0, floor(n/2), chord),
        lower_y = te_interp_y(points, cut_x, floor(n/2), n-1, chord)
    )
    concat(
        [[cut_x, upper_y]],
        kept,
        [[cut_x, lower_y]]
    );

// Interpolate y-value at x=cut_x by scanning point indices from idx_start to idx_end
// Finds the segment that crosses cut_x and linearly interpolates
function te_interp_y(points, cut_x, idx_start, idx_end, chord) =
    let(
        candidates = [
            for (i = [idx_start : idx_end - 1])
                if ((points[i][0] <= cut_x && points[i+1][0] >= cut_x) ||
                    (points[i][0] >= cut_x && points[i+1][0] <= cut_x))
                    let(
                        x0 = points[i][0], y0 = points[i][1],
                        x1 = points[i+1][0], y1 = points[i+1][1],
                        frac = (x1 == x0) ? 0.5 : (cut_x - x0) / (x1 - x0)
                    )
                    y0 + frac * (y1 - y0)
        ]
    )
    (len(candidates) > 0) ? candidates[0] : 0;

// ============================================================================
// NACA Airfoil 2D Module
//
// Generates a 2D polygon of a NACA 4-digit airfoil, optionally with a
// blunted trailing edge.
//
// naca        : 4-digit NACA number (e.g. 2412, 0012, 4415)
// chord       : chord length in mm
// blunt       : fraction of chord to cut for blunting (0 = sharp TE)
// n           : number of surface points (higher = smoother)
// ============================================================================
module naca_airfoil_2d(naca = 2412, chord = 100, blunt = 0.0, n = 80) {
    raw_points = naca_airfoil_points(naca, chord, n);
    points = (blunt > 0) ?
        blunt_trailing_edge(raw_points, blunt, chord) :
        raw_points;
    polygon(points);
}
