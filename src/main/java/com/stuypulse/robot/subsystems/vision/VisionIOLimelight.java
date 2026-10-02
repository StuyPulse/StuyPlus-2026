/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.RobotController;

import static edu.wpi.first.units.Units.Radians;

import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
    private final Supplier<Rotation2d> rotationSupplier;
    private final DoubleArrayPublisher orientationPublisher;
    private final DoubleArrayPublisher cameraPosePublisher;
    private final IntegerPublisher pipelinePublisher;
    private final DoubleArrayPublisher aprilTagWhitelistPublisher;

    private final DoubleSubscriber latencySubscriber;
    private final DoubleSubscriber txSubscriber;
    private final DoubleSubscriber tySubscriber;
    private final DoubleArraySubscriber megatag1Subscriber;
    private final DoubleArraySubscriber megatag2Subscriber;

    private MegaTagMode megaTagMode;

    /**
     * Creates a new VisionIOLimelight.
     *
     * @param name             The configured name of the Limelight.
     * @param rotationSupplier Supplier for the current estimated rotation, used for
     *                         MegaTag 2.
     */
    public VisionIOLimelight(String name, Transform3d robotToCamera, Supplier<Rotation2d> rotationSupplier) {
        megaTagMode = MegaTagMode.MEGATAG_1;

        var table = NetworkTableInstance.getDefault().getTable(name);
        this.rotationSupplier = rotationSupplier;
        pipelinePublisher = table.getIntegerTopic("pipeline").publish();
        orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
        cameraPosePublisher = table.getDoubleArrayTopic("camerapose_robotspace_set").publish();
        aprilTagWhitelistPublisher = table.getDoubleArrayTopic("fiducial_id_filters_set").publish();
        latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
        txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
        tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
        megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
        megatag2Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});

        cameraPosePublisher.accept(
                new double[] {
                    robotToCamera.getX(),
                    robotToCamera.getY(),
                    robotToCamera.getZ(),
                    robotToCamera.getRotation().getMeasureX().in(Radians),
                    robotToCamera.getRotation().getMeasureY().in(Radians),
                    robotToCamera.getRotation().getMeasureZ().in(Radians)
                });
        }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Update connection status based on whether an update has been seen in the last
        // 250ms
        inputs.connected = ((RobotController.getTime() - latencySubscriber.getLastChange()) / 1000) < 250;

        // Update target observation
        inputs.latestTargetObservation = new TargetObservation(
                Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

        // Update orientation for MegaTag 2
        orientationPublisher.accept(
                new double[] { rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0 });
        NetworkTableInstance.getDefault()
                .flush(); // Increases network traffic but recommended by Limelight

        // Read new pose observations from NetworkTables
        Set<Integer> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();
        if (megaTagMode == MegaTagMode.MEGATAG_1) {
            for (var rawSample : megatag1Subscriber.readQueue()) {
                if (rawSample.value.length == 0)
                    continue;
                for (int i = 11; i < rawSample.value.length; i += 7) {
                    tagIds.add((int) rawSample.value[i]);
                }
                poseObservations.add(
                        new PoseObservation(
                                // Timestamp, based on server timestamp of publish and latency
                                rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

                                // 3D pose estimate
                                parsePose(rawSample.value),

                                // Ambiguity, using only the first tag because ambiguity isn't applicable for
                                // multitag
                                rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,

                                // Tag count
                                (int) rawSample.value[7],

                                // Average tag distance
                                rawSample.value[9],

                                // Observation type
                                PoseObservationType.MEGATAG_1));
            }
        }

        if (megaTagMode == MegaTagMode.MEGATAG_2) {
            for (var rawSample : megatag2Subscriber.readQueue()) {
                if (rawSample.value.length == 0)
                    continue;
                for (int i = 11; i < rawSample.value.length; i += 7) {
                    tagIds.add((int) rawSample.value[i]);
                }
                poseObservations.add(
                        new PoseObservation(
                                // Timestamp, based on server timestamp of publish and latency
                                rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

                                // 3D pose estimate
                                parsePose(rawSample.value),

                                // Ambiguity, zeroed because the pose is already disambiguated
                                0.0,

                                // Tag count
                                (int) rawSample.value[7],

                                // Average tag distance
                                rawSample.value[9],

                                // Observation type
                                PoseObservationType.MEGATAG_2));
            }
        }

        // Save pose observations to inputs object
        inputs.poseObservations = new PoseObservation[poseObservations.size()];
        for (int i = 0; i < poseObservations.size(); i++) {
            inputs.poseObservations[i] = poseObservations.get(i);
        }

        // Save tag IDs to inputs objects
        inputs.tagIds = new int[tagIds.size()];
        int i = 0;
        for (int id : tagIds) {
            inputs.tagIds[i++] = id;
        }
    }

    @Override
    public void applyOutputs(VisionIOOutputs outputs) {
        this.megaTagMode = outputs.megaTagMode;

        pipelinePublisher.accept(outputs.pipeline);

        aprilTagWhitelistPublisher.accept(Arrays.stream(outputs.aprilTagIDWhitelist).asDoubleStream().toArray());
    }

    /** Parses the 3D pose from a Limelight botpose array. */
    private static Pose3d parsePose(double[] rawLLArray) {
        return new Pose3d(
                rawLLArray[0],
                rawLLArray[1],
                rawLLArray[2],
                new Rotation3d(
                        Units.degreesToRadians(rawLLArray[3]),
                        Units.degreesToRadians(rawLLArray[4]),
                        Units.degreesToRadians(rawLLArray[5])));
    }
}
