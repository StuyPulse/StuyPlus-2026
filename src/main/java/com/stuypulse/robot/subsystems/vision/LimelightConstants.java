package com.stuypulse.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;

public interface LimelightConstants {
    public interface VisionSettings {

        // TODO: These numbers are temporary, may need testing
        public final Vector<N3> MT1_STDEVS = VecBuilder.fill(0.5, 0.5, 1.0);

        public final Vector<N3> MT2_STDEVS = VecBuilder.fill(0.7, 0.7, 694694.0);

        public final Pose2d INVALID_POSITION = Pose2d.kZero;

        public final double MAX_ANGULAR_VELOCITY_RAD_SEC = 2 * Math.PI;
    }

}