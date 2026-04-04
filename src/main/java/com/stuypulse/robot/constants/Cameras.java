package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

import com.stuypulse.stuylib.network.SmartBoolean;


public class Cameras {
    public static final Camera[] LimelightCameras = new Camera[] {
        new Camera("limelight", 
            new Pose3d(Units.inchesToMeters(-12.109), Units.inchesToMeters(-7.129), Units.inchesToMeters(8.375), 
            new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(28), Units.degreesToRadians(180))), 
            new SmartBoolean("Enabled", true))}; // TODO: get actual camera info

    public static class Camera {
        private String name;
        private Pose3d location;
        private SmartBoolean isActive;

        public Camera(String name, Pose3d location, SmartBoolean isActive) {
            this.name = name;
            this.location = location;
            this.isActive = isActive;
        }

        public String getName(){
            return name;
        }

        public Pose3d getLocation() {
            return location;
        }

        public SmartBoolean isEnabled() {
            return isActive;
        }

        public void setEnabled(boolean enabled) {
            this.isActive.set(enabled);
        }
    }
}
