package com.stuypulse.robot.util.vision;

import edu.wpi.first.math.geometry.Pose3d;;

public class AprilTag {
    
    private final int id;
    private final Pose3d location;

    public AprilTag(int id, Pose3d location) {
        this.id = id;
        this.location = location;
    }

    public int getID() {
        return id;
    }

    public Pose3d getLocation() {
        return location;
    }

}