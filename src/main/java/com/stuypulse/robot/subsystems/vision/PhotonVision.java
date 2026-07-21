package com.stuypulse.robot.subsystems.vision;


import java.util.List;
import java.util.Optional;


import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;


import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Cameras.Camera;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PhotonVision extends SubsystemBase {
    private static PhotonVision instance;
   
    static {
        instance = new PhotonVision(AprilTagFields.k2026RebuiltAndymark);
    }
   
    public static PhotonVision getInstance() {
        return instance;
    }


    private CommandSwerveDrivetrain drivetrainInstance;
    private SwerveDriveSimulation mapleSimDrivetrain;
   
    private VisionSystemSim visionSim;


    private PhotonCamera[] realCameras = new PhotonCamera[Cameras.LimelightCameras.length];
    private PhotonCameraSim[] cameraSims = new PhotonCameraSim[Cameras.LimelightCameras.length];
    private PhotonPoseEstimator[] poseEstimators = new PhotonPoseEstimator[Cameras.LimelightCameras.length];


    private PhotonVision(AprilTagFields fieldLayout) {
        this.drivetrainInstance = CommandSwerveDrivetrain.getInstance();
        this.mapleSimDrivetrain = drivetrainInstance.getMapleSimDrive();

        this.visionSim = new VisionSystemSim("main");
       
        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(fieldLayout);
        visionSim.addAprilTags(layout);

        for (int i = 0; i < Cameras.LimelightCameras.length; i++) {
            Camera camera = Cameras.LimelightCameras[i];

            PhotonCamera realCamera = new PhotonCamera(camera.name());
            Transform3d cameraTransform = camera.location();
            PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(layout, cameraTransform);

            realCameras[i] = realCamera;
            poseEstimators[i] = poseEstimator;

            if (Robot.isSimulation()) {
                SimCameraProperties cameraProps = new SimCameraProperties();

                PhotonCameraSim cameraSim = new PhotonCameraSim(realCamera, cameraProps);
                cameraSims[i] = cameraSim;
                visionSim.addCamera(cameraSim, cameraTransform);
            }


        }
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
            PhotonCamera currentCamera = realCameras[i];
            PhotonPoseEstimator currentEstimator = poseEstimators[i];


            List<PhotonPipelineResult> results = currentCamera.getAllUnreadResults();
            for (PhotonPipelineResult result: results) {
                Optional<EstimatedRobotPose> estimatedPose = currentEstimator.estimateCoprocMultiTagPose(result);
                if (estimatedPose.isEmpty()) {
                    estimatedPose = currentEstimator.estimateLowestAmbiguityPose(result);
                }
               
                if (estimatedPose.isEmpty()) {
                    continue;
                }


                Pose2d finalPose = estimatedPose.get().estimatedPose.toPose2d();
                drivetrainInstance.addVisionMeasurement(finalPose, result.getTimestampSeconds());


                if (Robot.isSimulation()) {
                    visionSim.getDebugField()
                        .getObject("VisionEstimation" + currentCamera.getName())
                        .setPose(finalPose);
                }
            }
        }
    }
}
