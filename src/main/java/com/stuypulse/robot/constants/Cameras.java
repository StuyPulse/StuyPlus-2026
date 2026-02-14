package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import com.stuypulse.stuylib.network.SmartBoolean;


public class Cameras {
    public static final Camera[] LimelightCameras = new Camera[] {new Camera("Camera", new Pose3d(5, 5, 5, new Rotation3d(0, 0, 0)), new SmartBoolean("Enabled", true))}; // TODO: get actual camera info

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
