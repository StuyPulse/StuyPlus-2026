/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

import com.stuypulse.robot.constants.Field;

import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
    private static VisionSystemSim visionSim;

    private final Supplier<Pose2d> poseSupplier;
    private final PhotonCameraSim cameraSim;

    /**
     * Creates a new VisionIOPhotonVisionSim.
     *
     * @param name         The name of the camera.
     * @param poseSupplier Supplier for the robot pose to use in simulation.
     */
    public VisionIOPhotonVisionSim(
            String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
        super(name, robotToCamera);
        this.poseSupplier = poseSupplier;

        // Initialize vision sim
        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(Field.APRIL_TAG_LAYOUT);
        }

        // Add sim camera
        var cameraProperties = new SimCameraProperties();
        cameraSim = new PhotonCameraSim(camera, cameraProperties);
        visionSim.addCamera(cameraSim, robotToCamera);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        visionSim.update(poseSupplier.get());
        super.updateInputs(inputs);
    }
}
