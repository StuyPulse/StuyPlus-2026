package com.stuypulse.robot.subsystems.intake;

import java.util.Optional;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


public class IntakeSim extends Intake {
    private final FlywheelSim intakeRollerMotor;
    private final PIDController pivotController;
    private final DutyCycleOut rollerController;
    private final SingleJointedArmSim sim;

    private Optional<Double> pivotVoltageOverride;

    private final StructPublisher<Pose3d> pivotPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("AdvScope/IntakePose", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> hopperPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("AdvScope/HopperPose", Pose3d.struct).publish();
    
    public IntakeSim() {
        DCMotor gearbox = DCMotor.getKrakenX60(1);

        intakeRollerMotor = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                Settings.Intake.J_KG_METERS_SQUARED,
                Settings.Intake.ROLLER_GEAR_RATIO
            ),
            DCMotor.getKrakenX60(1)
        );

        pivotController = new PIDController(
            Gains.Intake.kP,
            Gains.Intake.kI,
            Gains.Intake.kD
        );

        rollerController = new DutyCycleOut(getState().getTargetDutyCycle());

        sim = new SingleJointedArmSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                Settings.Intake.J_KG_METERS_SQUARED,
                Settings.Intake.PIVOT_GEAR_RATIO
            ),
            gearbox,
            Settings.Intake.PIVOT_GEAR_RATIO,
            Settings.Intake.PIVOT_LENGTH,
            Settings.Intake.PIVOT_MIN_ANGLE,
            Settings.Intake.PIVOT_MAX_ANGLE,
            true,
            Settings.Intake.PIVOT_INITIAL_ANGLE.getRadians());
        
        pivotVoltageOverride = Optional.empty();
    }
    @Override
    public Rotation2d getRelativePosition() {
        return new Rotation2d(sim.getAngleRads());
    }

    @Override
    public boolean atAngle() {
        return Math.abs(
            (getRelativePosition().getRotations()) - getState().getTargetAngle().getRotations()) 
                < Settings.Intake.ANGLE_TOLERANCE.getRotations();
    }

    @Override
    public double getRollerRPM() {
        return intakeRollerMotor.getAngularVelocityRPM();
    }

    public void setPivotVoltageOverride(Optional<Double> pivotVoltageOverride) {
        this.pivotVoltageOverride = pivotVoltageOverride;
    }

    public SysIdRoutine getIntakeSysIdRoutine() {
        return SysId.getRoutine(Settings.Intake.RAMP_RATE,
                Settings.Intake.STEP_VOLTAGE,
                "Intake",
                voltage -> setPivotVoltageOverride(Optional.of(voltage)),
                () -> getRelativePosition().getRotations(),
                () -> sim.getVelocityRadPerSec(),
                () -> pivotVoltageOverride.orElse(0.0),
                getInstance()
        );
    }

    @Override
    public void periodic() {
        super.periodic();

        double rollerVoltage = getState().getTargetDutyCycle() * 12;
        intakeRollerMotor.setInputVoltage(rollerVoltage); // TODO: stop doing ts the manual way
        intakeRollerMotor.update(Settings.DT);

        double voltage = pivotController.calculate(sim.getAngleRads(), getState().getTargetAngle().getRadians());
        sim.setInputVoltage(voltage);
        sim.update(Settings.DT);

        pivotPublisher.set(
            new Pose3d(
                0.31,
                0,
                0.225,
                new Rotation3d(
                    sim.getAngleRads() - Math.toRadians(55), // 55 is roughly the offset from CAD zero angle and robot zero angle
                    0,
                    Math.toRadians(90) // turn it to face same direction as robot
                )
            )
        );
        hopperPublisher.set(
            new Pose3d(
                -0.310209692 * Math.cos(
                    getRelativePosition().getRadians() + Math.toRadians(50)
                ) - 0.15,
                0,
                0.275,
                new Rotation3d(Math.toRadians(90), 0, Math.toRadians(90))
            )
        );

        SmartDashboard.putNumber("Intake/rollerVoltage", rollerVoltage);
        SmartDashboard.putNumber("Intake/rollerRPM", getRollerRPM());
        SmartDashboard.putNumber("Intake/pivotVoltage", voltage);
        SmartDashboard.putNumber("Intake/pivotAngle", Math.toDegrees(sim.getAngleRads()));
        SmartDashboard.putNumber("Intake/pivotTarget", getState().getTargetAngle().getDegrees());
    }
}