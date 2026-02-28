package com.stuypulse.robot.subsystems.intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class IntakeSim extends Intake {
    private final FlywheelSim intakeRollerMotor;
    private final PIDController pivotController;
    private final DutyCycleOut rollerController;
    private final SingleJointedArmSim sim;
    
    public IntakeSim() {
        DCMotor gearbox = DCMotor.getKrakenX60(1);

        intakeRollerMotor = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                Settings.Intake.JKgMetersSquared,
                Settings.Intake.GEAR_RATIO
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
                Settings.Intake.JKgMetersSquared,
                Settings.Intake.GEAR_RATIO
            ),
            gearbox,
            Settings.Intake.GEAR_RATIO,
            Settings.Intake.ARM_LENGTH,
            Settings.Intake.PIVOT_MIN_ANGLE,
            Settings.Intake.PIVOT_MAX_ANGLE,
            true,
            Settings.Intake.INITIAL_POSITION);
    }
    @Override
    public Rotation2d getRelativePosition() {
        return new Rotation2d(SLMath.clamp(sim.getAngleRads(),
        Settings.Intake.PIVOT_MIN_ANGLE, Settings.Intake.PIVOT_MAX_ANGLE));
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