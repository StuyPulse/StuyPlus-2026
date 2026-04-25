package com.stuypulse.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.commands.vision.SetPipeline;
import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.vision.LimelightHelpers;
import com.stuypulse.robot.util.vision.LimelightHelpers.IMUData;
import com.stuypulse.robot.util.vision.LimelightHelpers.PoseEstimate;
import com.stuypulse.stuylib.network.SmartBoolean;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.stuypulse.robot.constants.Settings.EnabledSubsystems;

public class LimelightVision extends SubsystemBase{

    private static final LimelightVision instance;

    static {
        instance = new LimelightVision();

        SmartDashboard.putData("Vision/Set Cloudy Pipeline", new SetPipeline(0));
        SmartDashboard.putData("Vision/Set Sunny Pipeline", new SetPipeline(1));
    }

    public static LimelightVision getInstance() {
        return instance;
    }

    private String[] names;
    private SmartBoolean enabled;
    private SmartBoolean[] camerasEnabled;
    private MegaTagMode megaTagMode;

    public enum MegaTagMode {
        MEGATAG1,
        MEGATAG2 
    }

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
                robotRelativePose.getRotation().getMeasureX().in(Degrees),
                robotRelativePose.getRotation().getMeasureY().in(Degrees),
                robotRelativePose.getRotation().getMeasureZ().in(Degrees)
                // Rotation2d.fromRadians(robotRelativePose.getRotation().getX()).getDegrees(), 
                // Rotation2d.fromRadians(robotRelativePose.getRotation().getY()).getDegrees(), 
                // Rotation2d.fromRadians(robotRelativePose.getRotation().getZ()).getDegrees()
            );
        }

        camerasEnabled = new SmartBoolean[Cameras.LimelightCameras.length];
        
        for (int i = 0; i < camerasEnabled.length; i++) {
            camerasEnabled[i] = new SmartBoolean("Vision/" + names[i] + " Is Enabled", true);
            setIMUMode(1);
            SmartDashboard.putBoolean("Vision/" + names[i] + " Has Data", false);
        }

        setMegaTagMode(MegaTagMode.MEGATAG1);
    }

    public void setTagWhitelist(int[] ids) {
        for (String name : names) {
            LimelightHelpers.SetFiducialIDFiltersOverride(name, ids);
        }
    }

    public void enable() {
        EnabledSubsystems.VISION.set(true);
    }

    public void disable() {
        EnabledSubsystems.VISION.set(false);
    }

    public void setCameraEnabled(String name, boolean enabled) {
        for (int i = 0; i < names.length; i++) {
            if (names[i].equals(name)) {
                camerasEnabled[i].set(enabled);
            }
        }
    }

    public void setMegaTagMode(MegaTagMode mode) {
        this.megaTagMode = mode;
    }

    public void setIMUMode(int mode) {
        for (String name : names) {
            LimelightHelpers.SetIMUMode(name, mode);
        }
    }

    public void setPipeline(int pipeline) {
        for (String name : names) {
            LimelightHelpers.setPipelineIndex(name, pipeline);
        }
    }

    /**
     * Allows you to set the convergence speed of the internal LL IMU and robot gyro. 
     *
     * @param assistValue, an double that sets the correction speed of the complementary filter for the IMU. IMU Mode 4 
     * uses the fusing of the internal IMU (1khz) with the external gyro reading as well. Higher values ranging towards 1 
     * indicate a faster convergence of internal IMU to the robot IMU mode. Defaults to 0.001.
     */
    public void setIMUAssistValue(double assistValue) {
        for (String name : names) {
            LimelightHelpers.SetIMUAssistAlpha(name, assistValue);
        }
    }

    public IMUData[] getIMUData() {
        IMUData[] data = new IMUData[Cameras.LimelightCameras.length];

        for (int i = 0; i < Cameras.LimelightCameras.length; i++) {
            data[i] = LimelightHelpers.getIMUData(Cameras.LimelightCameras[i].getName());
        }

        return data;
    }
    

    @Override
    public void periodic() {
        if (!EnabledSubsystems.VISION.get()) {
            return;
        }

        for (int i = 0; i < names.length; i++) {
            if (!camerasEnabled[i].get()) {
                SmartDashboard.putBoolean("Vision/" + names[i] + " Has Data", false);
                continue;
            }

            String limelightName = names[i];

            // Seed robot heading (used by MT2)
            LimelightHelpers.SetRobotOrientation(
                limelightName, 
                (CommandSwerveDrivetrain.getInstance().getPose().getRotation().getDegrees() + (Robot.isBlue() ? 0 : 180)) % 360, 
                0, 
                0, 
                0, 
                0, 
                0
            );

            PoseEstimate poseEstimate;

            // MegaTag switching
            if (megaTagMode == MegaTagMode.MEGATAG1) {
                poseEstimate = Robot.isBlue() 
                    ? LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName)
                    : LimelightHelpers.getBotPoseEstimate_wpiRed(limelightName);
            } else {
                poseEstimate = Robot.isBlue() 
                    ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName)
                    : LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName);
            }
                
            boolean notNull = false;
            boolean withinAngularVelocityTolerance = false;
            boolean withinInvalidPositionTolerance = false;

            // Adding to pose estimator
            if (poseEstimate != null && poseEstimate.tagCount > 0) {
                notNull = true;

                if (poseEstimate.pose.equals(Settings.Vision.INVALID_POSITION)) {
                    withinInvalidPositionTolerance = true;
                }

                if (CommandSwerveDrivetrain.getInstance().getChassisSpeeds().omegaRadiansPerSecond < Settings.Vision.MAX_ANGULAR_VELOCITY_RAD_SEC) {
                    withinAngularVelocityTolerance = true;
                }

                Boolean isValidPose = notNull && withinAngularVelocityTolerance && withinInvalidPositionTolerance;

                SmartDashboard.putBoolean("Vision/isValidPose", isValidPose);
                SmartDashboard.putBoolean("Vision/isWithinAngularVel", withinAngularVelocityTolerance);
                SmartDashboard.putBoolean("Vision/isWithinPos", withinInvalidPositionTolerance);
                Pose2d robotPose = poseEstimate.pose;
                double timestamp = poseEstimate.timestampSeconds;

                if (megaTagMode == MegaTagMode.MEGATAG1 && isValidPose) {
                    CommandSwerveDrivetrain.getInstance().addVisionMeasurement(robotPose, timestamp, Settings.Vision.MT1_STDEVS);
                } else if (megaTagMode == MegaTagMode.MEGATAG2 && isValidPose){
                    CommandSwerveDrivetrain.getInstance().addVisionMeasurement(robotPose, timestamp, Settings.Vision.MT2_STDEVS);
                }
                    
                SmartDashboard.putNumber("Vision/Pose X Component", robotPose.getX());
                SmartDashboard.putNumber("Vision/Pose Y Component", robotPose.getY());
                SmartDashboard.putNumber("Vision/Pose Theta (Degrees)", robotPose.getRotation().getDegrees());

                SmartDashboard.putBoolean("Vision/" + names[i] + " Has Data", true);
            
                SmartDashboard.putString("Vision/MegaTag Mode", megaTagMode.toString());
                // this yaw is seems to be the robot yaw passed into the LL
                SmartDashboard.putNumber("Vision/Pipeline", LimelightHelpers.getCurrentPipelineIndex(limelightName));
                SmartDashboard.putNumber("Vision/Limelight Robot Yaw", LimelightHelpers.getIMUData(limelightName).robotYaw);
                // this is just the yaw of the internal imu 
                SmartDashboard.putNumber("Vision/Limelight Yaw", LimelightHelpers.getIMUData(limelightName).Yaw);
                SmartDashboard.putBoolean("Vision/Has at least 2 tags", poseEstimate.tagCount >= 2);
            }
        }
    }
}