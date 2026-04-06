package com.stuypulse.robot.subsystems.intake;

import java.util.Optional;

import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SysId;

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
    private final SingleJointedArmSim sim;

    private Optional<Double> pivotVoltageOverride;
    
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

    @Override
    public void setPivotZero() {
        // TODO: make
    }
    
    @Override
    public void setPivotZeroAtBottom() {
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

        SmartDashboard.putNumber("Intake/rollerVoltage", rollerVoltage);
        SmartDashboard.putNumber("Intake/rollerRPM", getRollerRPM());
        SmartDashboard.putNumber("Intake/pivotVoltage", voltage);
        SmartDashboard.putNumber("Intake/pivotAngle", Math.toDegrees(sim.getAngleRads()));
        SmartDashboard.putNumber("Intake/pivotTarget", getState().getTargetAngle().getDegrees());
    }
}