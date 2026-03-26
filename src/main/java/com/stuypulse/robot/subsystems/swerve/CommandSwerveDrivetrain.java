/************************ PROJECT 2026 *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.TunerConstants.TunerSwerveDrivetrain;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private final static CommandSwerveDrivetrain instance;

    static {
        instance = TunerConstants.createDrivetrain();
    }

    public static CommandSwerveDrivetrain getInstance() {
        return instance;
    }
    
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_moduleTranslationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withDeadband(Settings.Swerve.MODULE_VELOCITY_DEADBAND_M_PER_S)
        .withRotationalDeadband(Settings.Swerve.ROTATIONAL_DEADBAND_RAD_PER_S)
        .withDesaturateWheelSpeeds(true);

    private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withDeadband(Settings.Swerve.MODULE_VELOCITY_DEADBAND_M_PER_S)
        .withRotationalDeadband(Settings.Swerve.ROTATIONAL_DEADBAND_RAD_PER_S)
        .withDesaturateWheelSpeeds(true);

    public SwerveRequest.FieldCentric getFieldCentricSwerveRequest() {
        return this.fieldCentricRequest;
    }

    public SwerveRequest.RobotCentric getRobotCentricSwerveRequest() {
        return this.robotCentricRequest;
    }

    /* SysId routine for characterizing module translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineModuleTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdModuleTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_moduleTranslationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing chassis translation. This is used to find PID gains PID to pose. */
    private final SysIdRoutine m_sysIdRoutineChassisTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in meters per second², but SysId only supports "volts per second" */
            Volts.of(1).per(Second),
            /* This is in meters per second, but SysId only supports "volts" */
            Volts.of(Settings.Swerve.Constraints.MAX_VELOCITY_M_PER_S),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdChassisTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually meters per second, but SysId only supports "volts" */
                setControl(getFieldCentricSwerveRequest().withVelocityX(output.in(Volts)).withVelocityY(0).withRotationalRate(0));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Target X Velocity ('voltage')", output.in(Volts));
                SignalLogger.writeDouble("X Position", getPose().getX());
                SignalLogger.writeDouble("X Velocity", getChassisSpeeds().vxMetersPerSecond * getPose().getRotation().getCos());
            },
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Target_Rate ('voltage')", output.in(Volts));
                SignalLogger.writeDouble("Rotational Position", getPose().getRotation().getRadians());
                SignalLogger.writeDouble("Rotational_Velocity", getState().Speeds.omegaRadiansPerSecond);
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineModuleTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    protected CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    private CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    private CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    @Override
    public void setControl(SwerveRequest request) {
        if (Settings.EnabledSubsystems.SWERVE.get()) {
            super.setControl(request);
        }
        else {
            super.setControl(new SwerveRequest.Idle());
        }
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrgix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    public Pose2d getPose() {
        return getState().Pose;
    }
    
    public Pose2d getShooterPose() {
        return getPose().plus(new Transform2d(0.0, -7.836, new Rotation2d())); // offset is negative because the shooter is behind the robot center
    }

    public void configureAutoBuilder(){
        try{
            AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getChassisSpeeds,
                this::setChassisSpeeds,
                new PPHolonomicDriveController(Gains.Swerve.Alignment.XY, Gains.Swerve.Alignment.THETA),
                RobotConfig.fromGUISettings(),
                () -> false,
                instance
            );
            PathPlannerLogging.setLogActivePathCallback((poses) -> {
                if (Robot.isBlue()) {
                    Field.FIELD2D.getObject("path").setPoses(poses);
                }
                else {
                    Field.FIELD2D.getObject("path").setPoses(Field.transformToOppositeAlliance(poses));
                }
            });
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public Command followPathCommand(String pathName) {
        try {
            return followPathCommand(PathPlannerPath.fromPathFile(pathName));
        }
        catch (Exception e) {
            throw new IllegalArgumentException(pathName + " does not exist");
        }
    }

    public Command followPathCommand(PathPlannerPath path) {
        return AutoBuilder.followPath(path);
    }
  
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            moduleStates[i] = getModule(i).getCurrentState();
        }
        return moduleStates;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getModuleStates());
    }

    public Vector2D getFieldRelativeSpeeds() {
        return new Vector2D(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond)
            .rotate(Angle.fromRotation2d(getPose().getRotation()));
    }

    private void setChassisSpeeds(ChassisSpeeds robotSpeeds) {
        setControl(new SwerveRequest.RobotCentric()
            .withVelocityX(robotSpeeds.vxMetersPerSecond)
            .withVelocityY(robotSpeeds.vyMetersPerSecond)
            .withRotationalRate(robotSpeeds.omegaRadiansPerSecond));
    }

    public void drive(Vector2D velocity, double rotation) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            Robot.isBlue() ? velocity.y : -velocity.y, 
            Robot.isBlue() ? -velocity.x : velocity.x,
            -rotation,
            getPose().getRotation());

        Pose2d robotVel = new Pose2d(
            Settings.DT * speeds.vxMetersPerSecond,
            Settings.DT * speeds.vyMetersPerSecond,
            Rotation2d.fromRadians(Settings.DT * speeds.omegaRadiansPerSecond));
        Twist2d twistVel = new Pose2d().log(robotVel);

        setChassisSpeeds(new ChassisSpeeds(
            twistVel.dx / Settings.DT,
            twistVel.dy / Settings.DT,
            twistVel.dtheta / Settings.DT
        ));
    }

    private final StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
        .getStructTopic("AdvScope/DTPose", Pose2d.struct).publish();
    @Override
    public void periodic() {
        publisher.set(Robot.isBlue() ? getPose() : Field.transformToOppositeAlliance(getPose()));

        SmartDashboard.putNumber("Swerve/Pose/X", getPose().getX());
        SmartDashboard.putNumber("Swerve/Pose/Y", getPose().getY());
        SmartDashboard.putNumber("Swerve/Pose/Theta", getPose().getRotation().getDegrees());

        for (int i = 0; i < 4; i++) {
            SmartDashboard.putNumber("Swerve/Modules/Module " + i + "/Speed (m per s)", getModule(i).getCurrentState().speedMetersPerSecond);
              SmartDashboard.putNumber("Swerve/Modules/Module " + i + "/Target Speed (m per s)", getModule(i).getTargetState().speedMetersPerSecond);
            SmartDashboard.putNumber("Swerve/Modules/Module " + i + "/Angle (deg)", getModule(i).getCurrentState().angle.getDegrees() % 360);
            SmartDashboard.putNumber("Swerve/Modules/Module " + i + "/Target Angle (deg)", getModule(i).getTargetState().angle.getDegrees() % 360);
        }

        Field.FIELD2D.getRobotObject().setPose(Robot.isBlue() ? getPose() : Field.transformToOppositeAlliance(getPose()));

        if (Settings.DEBUG_MODE) {
            for (int i = 0; i < 4; i++) {
                SmartDashboard.putNumber("Swerve/Modules/Module " + i + "/Stator Current", getModule(i).getDriveMotor().getStatorCurrent().getValueAsDouble());
                SmartDashboard.putNumber("Swerve/Modules/Module " + i + "/Supply Current", getModule(i).getDriveMotor().getSupplyCurrent().getValueAsDouble());
            }

            SmartDashboard.putNumber("Swerve/Velocity Robot Relative X (m per s)", getChassisSpeeds().vxMetersPerSecond);
            SmartDashboard.putNumber("Swerve/Velocity Robot Relative Y (m per s)", getChassisSpeeds().vyMetersPerSecond);
    
            SmartDashboard.putNumber("Swerve/Velocity Field Relative X (m per s)", getFieldRelativeSpeeds().x);
            SmartDashboard.putNumber("Swerve/Field Relative Rotation", getPose().getRotation().getDegrees());
            SmartDashboard.putNumber("Swerve/Velocity Field Relative Y (m per s)", getFieldRelativeSpeeds().y);
    
            SmartDashboard.putNumber("Swerve/Angular Velocity (rad per s)", getChassisSpeeds().omegaRadiansPerSecond);
        }
    }
}