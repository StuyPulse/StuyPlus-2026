package com.stuypulse.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Small helper to draw a polygon-like ring using multiple MechanismLigament2d segments.
 *
 * This class intentionally keeps a minimal, safe API (no assumptions about WPILib internals).
 * It creates N short ligaments positioned around the provided parent object so the result
 * visually approximates a polygonal rim. You can rotate the rim or set a base angle.
 */
public final class MechanismPolygon2d {

    private final MechanismLigament2d[] segments;
    private final int sides;
    private final double radius;

    /**
     * Create a polygon ring attached to a MechanismObject2d (root or ligament).
     *
     * @param parent the MechanismObject2d to attach the polygon to
     * @param name base name for each segment ("name 0", "name 1", ...)
     * @param sides number of polygon sides (must be >= 3)
     * @param radius distance from parent to polygon vertices (visual radius)
     * @param thickness visual thickness of each ligament
     * @param initialAngleDeg starting rotation in degrees for the first segment
     * @param color segment color
     */
    public MechanismPolygon2d(MechanismObject2d parent, String name, int sides, double radius, double thickness, double initialAngleDeg, Color8Bit color) {
        if (sides < 3) {
            throw new IllegalArgumentException("sides must be >= 3");
        }

        this.sides = sides;
        this.radius = radius;
        this.segments = new MechanismLigament2d[sides];

        // Compute chord length for each side so ligaments lie approximately along the rim.
        // chord = 2 * r * sin(pi / sides)
        double chord = 2.0 * radius * Math.sin(Math.PI / sides);
        double step = 360.0 / sides;

        for (int i = 0; i < sides; i++) {
            double angle = initialAngleDeg + i * step;
            // Attach each short ligament at the appropriate angle; length ~= chord
            segments[i] = parent.append(new MechanismLigament2d(name + " " + i, chord, angle, thickness, color));
        }
    }

    /** Rotate every segment by delta degrees (adds to their current angle). */
    public void rotateBy(double deltaDeg) {
        for (MechanismLigament2d s : segments) {
            s.setAngle(s.getAngle() + deltaDeg);
        }
    }

    /** Set a base rotation for the polygon so segment i is placed at base + i*(360/sides). */
    public void setBaseAngle(double baseAngleDeg) {
        double step = 360.0 / sides;
        for (int i = 0; i < sides; i++) {
            segments[i].setAngle(baseAngleDeg + i * step);
        }
    }

    /** Return the underlying segment array for low-level manipulation if needed. */
    public MechanismLigament2d[] getSegments() {
        return segments;
    }
}
