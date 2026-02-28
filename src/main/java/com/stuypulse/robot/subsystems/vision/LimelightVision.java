package com.stuypulse.robot.subsystems.vision;

import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.robot.util.vision.LimelightHelpers;
import com.stuypulse.robot.util.vision.LimelightHelpers.PoseEstimate;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.stuypulse.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightVision extends SubsystemBase {
    private static LimelightVision instance;

    static {
        instance = new LimelightVision();
    }

    public static LimelightVision getInstance() {
        return instance;
    }

    private String[] names;
    private SmartBoolean enabled;
    private SmartBoolean[] camerasEnabled;

    public LimelightVision() {
        names = new String[Cameras.LimelightCameras.length];
        for (int i = 0; i < Cameras.LimelightCameras.length; i++) {
            names[i] = Cameras.LimelightCameras[i].getName();
            Pose3d robotRelativePose = Cameras.LimelightCameras[i].getLocation();
            LimelightHelpers.setCameraPose_RobotSpace(
                names[i], 
                robotRelativePose.getX(), 
                robotRelativePose.getY(), 
                robotRelativePose.getZ(), 
                Rotation2d.fromRadians(robotRelativePose.getRotation().getX()).getDegrees(), 
                Rotation2d.fromRadians(robotRelativePose.getRotation().getY()).getDegrees(), 
                Rotation2d.fromRadians(robotRelativePose.getRotation().getZ()).getDegrees()
            );
        }
        
        enabled = new SmartBoolean("Vision/isEnabled", true);
    }

    public void setTagWhitelist(int ... ids ) {
        for(String name : names) {
            LimelightHelpers.SetFiducialIDFiltersOverride(name, ids);
        }
    }

    public void enable() {
        enabled.set(true);
    }

    public void disable() {
        enabled.set(false);
    }
    
    public void setCameraEnabled(String name, boolean enabled) {
        for(int i = 0; i < names.length; i++) {
            if (names[i].equals(name))
                camerasEnabled[i].set(enabled);
        }
    }


    public void SetIMUMode(int mode) {
        for (String name : names) {
            LimelightHelpers.SetIMUMode(name, mode);
        }
    }
     
    @Override
    public void periodic() {
        if (enabled.get()) {
            for (int i = 0; i < names.length; i++) {
                if (camerasEnabled[i].get()) {              
                   String limelightName = names[i];
                    LimelightHelpers.SetRobotOrientation(
                        limelightName,
                        (CommandSwerveDrivetrain.getInstance().getPose().getRotation().getDegrees() + (Robot.isBlue() ? 0 : 180)) % 360,
                        0,
                        0,
                        0,
                        0,
                        0
                    );

                    PoseEstimate poseEstimate = Robot.isBlue() 
                        ?   LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName)
                        :   LimelightHelpers.getBotPoseEstimate_wpiRed(limelightName);


                    if (poseEstimate != null && poseEstimate.tagCount > 0 ) {
                        Pose2d robotPose = poseEstimate.pose;
                        double timestamp = poseEstimate.timestampSeconds; 
                        
                        // CommandSwerveDrivetrain.getInstance().addVisionMeasurement(robotPose, timestamp, Settings.vision.MT2_STDEVS);
                        SmartDashboard.putNumber("Vision/Pose X Component", robotPose.getX());
                        SmartDashboard.putNumber("Vision/Pose Y Component", robotPose.getY());
                        SmartDashboard.putNumber("Vision/Pose Theta (Degrees)", robotPose.getRotation().getDegrees());
                   
                        SmartDashboard.putBoolean("Vision" + names[i] + "Has Data", true);
                    } else {
                        SmartDashboard.putBoolean("Vision/" + names[i] + " Has Data", false);
                    }
                }
            }
        }
    }   
}   



