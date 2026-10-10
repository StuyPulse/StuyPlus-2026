/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.vision;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import com.stuypulse.robot.commands.vision.SetPipeline;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.EnabledSubsystems;
import com.stuypulse.robot.constants.Settings.VisionMode;

import com.stuypulse.robot.subsystems.swerve.Swerve;
import com.stuypulse.robot.subsystems.vision.VisionConstants.CameraData;
import com.stuypulse.robot.subsystems.vision.VisionConstants.Cameras;
import com.stuypulse.robot.subsystems.vision.VisionConstants.Pipelines;
import com.stuypulse.robot.subsystems.vision.VisionConstants.VisionSettings;
import com.stuypulse.robot.subsystems.vision.VisionIO.MegaTagMode;
import com.stuypulse.robot.subsystems.vision.VisionIO.PoseObservationType;
import com.stuypulse.robot.subsystems.vision.VisionIO.VisionIOOutputs;
import com.stuypulse.robot.util.FullSubsystem;

import java.util.EnumMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map.Entry;
import org.littletonrobotics.junction.Logger;

public class Vision extends FullSubsystem {
    private static final Vision instance;

    private final Timer hdrTimer;

    static {
        Swerve swerve = Swerve.getInstance();
        EnumMap<Cameras, VisionIO> cameraIOMap = new EnumMap<>(Cameras.class);

        switch (Settings.CURRENT_MODE) {
            case REAL -> {
                for (Cameras camera : Cameras.values()) {
                    if (Settings.VISION_MODE == VisionMode.LIMELIGHT_VISION) {
                        cameraIOMap.put(camera, new VisionIOLimelight(camera.getName(), camera.getRobotToCamera(),swerve::getRotation));
                    } else {
                        cameraIOMap.put(
                                camera, new VisionIOPhotonVision(camera.getName(), camera.getRobotToCamera()));
                    }
                }
            }

            case SIM -> {
                for (Cameras camera : Cameras.values()) {
                    cameraIOMap.put(
                            camera,
                            new VisionIOPhotonVisionSim(
                                    camera.getName(), camera.getRobotToCamera(), swerve::getPose));
                }
            }

            // For replay mode
            default -> {
                for (Cameras camera : Cameras.values()) {
                    cameraIOMap.put(camera, new VisionIO() {
                    });
                }
            }
        }

        instance = new Vision(swerve::accept, cameraIOMap);

        SmartDashboard.putData("Vision/setLowSunPipeline", new SetPipeline(Pipelines.LOW_SUN.getIndex()));
        SmartDashboard.putData("Vision/setHighSunPipeline", new SetPipeline(Pipelines.HIGH_SUN.getIndex()));
    }

    public static Vision getInstance() {
        return instance;
    }

    private final VisionConsumer consumer;
    private final EnumMap<Cameras, VisionIO> io;
    private final EnumMap<Cameras, VisionIOInputsAutoLogged> inputs;
    private final EnumMap<Cameras, VisionIOOutputs> outputs;
    private final EnumMap<Cameras, Alert> disconnectedAlerts;
    private int maxTagCount;

    public Vision(VisionConsumer consumer, EnumMap<Cameras, VisionIO> io) {
        this.consumer = consumer;

        this.io = new EnumMap<>(io);

        // Initialize inputs
        this.inputs = new EnumMap<>(Cameras.class);
        this.outputs = new EnumMap<>(Cameras.class);

        // Initialize disconnected alerts
        this.disconnectedAlerts = new EnumMap<>(Cameras.class);

        this.hdrTimer = new Timer();
        hdrTimer.start();

        for (Cameras camera : Cameras.values()) {
            inputs.put(camera, new VisionIOInputsAutoLogged());
            outputs.put(camera, new VisionIOOutputs());
            disconnectedAlerts.put(
                    camera,
                    new Alert("Vision camera " + camera.getName() + " is disconnected.", Alert.AlertType.kWarning));
        }
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing
     * with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetX(Cameras camera) {
        return inputs.get(camera).latestTargetObservation.tx();
    }

    public int getMaxTagCount() {
        return maxTagCount;
    }

    public boolean isCameraDead(Cameras camera) {
        return !inputs.get(camera).connected;
    }

    public void enable() {
        EnabledSubsystems.VISION.set(true);
    }

    public void disable() {
        EnabledSubsystems.VISION.set(false);
        
    }

    public void setMegaTagMode(MegaTagMode mode) {
        for (Entry<Cameras, VisionIOOutputs> entry : outputs.entrySet()) {
            entry.getValue().megaTagMode = mode;
        }
    }

    public void setPipeline(int pipeline) {
        for (Entry<Cameras, VisionIOOutputs> entry : outputs.entrySet()) {
            entry.getValue().pipeline = pipeline;
        }
    }

    public int getPipeline() {
        return outputs.values().iterator().next().pipeline;
    }

    @Override
    public void periodic() {
        maxTagCount = 0;

        for (Entry<Cameras, VisionIO> entry : io.entrySet()) {
            VisionIO currentIO = entry.getValue();
            VisionIOInputsAutoLogged currentInputs = inputs.get(entry.getKey());
            currentIO.updateInputs(currentInputs);
            Logger.processInputs("Vision/" + entry.getKey().getName(), currentInputs);
        }

        if (!Settings.EnabledSubsystems.VISION.get()) {
            hdrTimer.stop();
            return;
        }

        if (VisionSettings.HDR_ENABLED.get()) {
            hdrTimer.start();
            if (hdrTimer.hasElapsed(VisionSettings.HDR_TIMEOUT)) {
                int currentPipeline = getPipeline();
                if (currentPipeline == Pipelines.LOW_SUN.getIndex()) {
                    setPipeline(Pipelines.HIGH_SUN.getIndex());
                } else {
                    setPipeline(Pipelines.LOW_SUN.getIndex());
                }
                hdrTimer.reset();
            }
        } else {
            hdrTimer.stop();
        }

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Loop over cameras
        for (Entry<Cameras, VisionIO> entry : io.entrySet()) {
            CameraData currentCameraData = entry.getKey().getData();
            VisionIOInputsAutoLogged currentInputs = inputs.get(entry.getKey());

            // Update disconnected alert
            disconnectedAlerts.get(entry.getKey()).set(!currentInputs.connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : currentInputs.tagIds) {
                var tagPose = Field.APRIL_TAG_LAYOUT.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // Loop over pose observations
            for (var observation : currentInputs.poseObservations) {
                maxTagCount = Math.max(maxTagCount, observation.tagCount());
                // Check whether to reject pose
                boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
                        || (observation.tagCount() == 1
                                && observation.ambiguity() > VisionSettings.MAX_AMBIGUITY) // Cannot be high ambiguity
                        || Math.abs(observation.pose().getZ()) > VisionSettings.MAX_Z_ERROR // Must have realistic Z
                                                                                            // coordinate

                        // Must be within the field boundaries
                        || observation.pose().getX() < 0.0
                        || observation.pose().getX() > Field.APRIL_TAG_LAYOUT.getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY() > Field.APRIL_TAG_LAYOUT.getFieldWidth();

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // Calculate standard deviations
                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = VisionSettings.LINEAR_STD_DEV_BASELINE * stdDevFactor;
                double angularStdDev = VisionSettings.ANGULAR_STD_DEV_BASELINE * stdDevFactor;
                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= VisionSettings.LINEAR_STD_DEV_MEGATAG_2_FACTOR;
                    angularStdDev *= VisionSettings.ANGULAR_STD_DEV_MEGATAG_2_FACTOR;
                }
                linearStdDev *= currentCameraData.stdDevFactor();
                angularStdDev *= currentCameraData.stdDevFactor();

                // Send vision observation
                consumer.accept(
                        observation.pose().toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
            }

            // Log camera datadata
            Logger.recordOutput(
                    "Vision/Camera" + currentCameraData.name() + "/TagPoses",
                    tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + currentCameraData.name() + "/RobotPoses",
                    robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + currentCameraData.name() + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + currentCameraData.name() + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput(
                "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesAccepted",
                allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesRejected",
                allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    }

    @Override
    public void periodicAfterScheduler() {
        for (Entry<Cameras, VisionIO> entry : io.entrySet()) {
            VisionIO currentIO = entry.getValue();
            VisionIOOutputs currentOutputs = outputs.get(entry.getKey());

            Logger.recordOutput(
                    "Vision/" + entry.getKey().name() + "/MegaTagMode", currentOutputs.megaTagMode);
            Logger.recordOutput("Vision/" + entry.getKey().name() + "/Pipeline", currentOutputs.pipeline);
            currentIO.applyOutputs(currentOutputs);
        }
    }

    @FunctionalInterface
    public interface VisionConsumer {
        void accept(
                Pose2d visionRobotPoseMeters,
                double timestampSeconds,
                Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
