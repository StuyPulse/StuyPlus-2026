package com.stuypulse.robot.subsystems.vision;


import java.util.List;
import java.util.Optional;


import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Cameras.Camera;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PhotonVision extends SubsystemBase {
    private static final PhotonVision instance;
   
    static {
        instance = new PhotonVision(AprilTagFields.k2026RebuiltAndymark);
    }
   
    public static PhotonVision getInstance() {
        return instance;
    }


    private final CommandSwerveDrivetrain drivetrainInstance;
    private final SwerveDriveSimulation mapleSimDrivetrain;
   
    private final VisionSystemSim visionSim;

    private final PhotonCamera[] realCameras = new PhotonCamera[Cameras.LimelightCameras.length];
    private final PhotonCameraSim[] cameraSims = new PhotonCameraSim[Cameras.LimelightCameras.length];
    private final PhotonPoseEstimator[] poseEstimators = new PhotonPoseEstimator[Cameras.LimelightCameras.length];
    private final Transform3d[] cameraTransforms = new Transform3d[Cameras.LimelightCameras.length];

    private final double FUEL_RADIUS = Units.inchesToMeters(2.955);

    private PhotonVision(AprilTagFields fieldLayout) {
        this.drivetrainInstance = CommandSwerveDrivetrain.getInstance();
        this.mapleSimDrivetrain = drivetrainInstance.getMapleSimDrive();

        this.visionSim = new VisionSystemSim("main");
       
        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(fieldLayout);
        visionSim.addAprilTags(layout);

        for (int i = 0; i < Cameras.LimelightCameras.length; i++) {
            Camera camera = Cameras.LimelightCameras[i];

            PhotonCamera realCamera = new PhotonCamera(camera.name());
            Transform3d cameraTransform = new Transform3d(camera.location().getTranslation(), camera.location().getRotation());
            PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(layout, cameraTransform);

            realCameras[i] = realCamera;
            poseEstimators[i] = poseEstimator;
            cameraTransforms[i] = cameraTransform;

            if (Robot.isSimulation()) {
                SimCameraProperties cameraProps = new SimCameraProperties();

                PhotonCameraSim cameraSim = new PhotonCameraSim(realCamera, cameraProps);
                cameraSims[i] = cameraSim;
                visionSim.addCamera(cameraSim, cameraTransform);
            }


        }
    }

    public Optional<Pose3d> getFuelFieldPose(Transform3d robotToCamera, PhotonTrackedTarget target) {
        final double yawRad = Units.degreesToRadians(target.getYaw());
        final double pitchRad = Units.degreesToRadians(target.getPitch());

        final double range = PhotonUtils.calculateDistanceToTargetMeters(
            robotToCamera.getZ(),
            FUEL_RADIUS,
            robotToCamera.getRotation().getY(),
            pitchRad
        );

        if (range <= 0 || Double.isNaN(range)) {
            return Optional.empty();
        }

        final Translation3d camToPiece = new Translation3d(
            range * Math.cos(yawRad),
            range * Math.sin(yawRad),
            FUEL_RADIUS - robotToCamera.getZ()
        );

        final Pose3d robotPose = new Pose3d(drivetrainInstance.getPose());
        final Pose3d fieldToCamera = robotPose.transformBy(robotToCamera);
        final Pose3d fieldToPiece = fieldToCamera.transformBy(
            new Transform3d(camToPiece, new Rotation3d())
        );

        return Optional.of(fieldToPiece);
    }

    @Override
    public void periodic() {
        if (!Settings.EnabledSubsystems.VISION.get()) {
            return;
        }


        if (Robot.isSimulation()) {
            visionSim.update(mapleSimDrivetrain.getSimulatedDriveTrainPose());
        }


        for (int i = 0; i < realCameras.length; i++) {
            final PhotonCamera currentCamera = realCameras[i];
            final PhotonPoseEstimator currentEstimator = poseEstimators[i];


            final List<PhotonPipelineResult> results = currentCamera.getAllUnreadResults();
            for (PhotonPipelineResult result : results) {
                Optional<EstimatedRobotPose> estimatedPose = currentEstimator.estimateCoprocMultiTagPose(result);
                if (estimatedPose.isEmpty()) {
                    estimatedPose = currentEstimator.estimateLowestAmbiguityPose(result);
                }
               
                if (estimatedPose.isEmpty()) {
                    continue;
                }


                final Pose2d finalPose = estimatedPose.get().estimatedPose.toPose2d();
                drivetrainInstance.addVisionMeasurement(finalPose, result.getTimestampSeconds());


                if (Robot.isSimulation()) {
                    visionSim.getDebugField()
                        .getObject("VisionEstimation" + currentCamera.getName())
                        .setPose(finalPose);
                }

                var fuelPose = this.getFuelFieldPose(cameraTransforms[i], result.getBestTarget());
                if (fuelPose.isPresent()) {
                    DogLog.log("Vision/" + currentCamera.getName() + "_FuelPose", fuelPose.get());
                }
            }
        }
    }
}
