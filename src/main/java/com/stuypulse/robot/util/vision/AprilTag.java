package com.stuypulse.robot.util.vision;

import edu.wpi.first.math.geometry.Pose3d;;

/**
 * <h2>Wrapper class for April Tags. Stores ID and Pose3d of the tag.</h2>
 */
public class AprilTag {
    
    private final int id;
    private final Pose3d location;

    /**
     * Instantiates an AprilTag object with the given ID and location.
     * @param id The ID of the April Tag.
     * @param location The Pose3d of the April Tag. 
    */
    public AprilTag(int id, Pose3d location) {
        this.id = id;
        this.location = location;
    }

    /**
     * @return The ID of the April Tag.
     */
    public int getID() {
        return id;
    }

    /**
     * @return The Pose3d of the April Tag.
     */
    public Pose3d getLocation() {
        return location;
    }

}