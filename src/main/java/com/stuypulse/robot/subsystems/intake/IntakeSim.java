package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class IntakeSim extends Intake{
    private DCMotorSim intakeRollerMotor;
    private DCMotorSim intakePivotMotor;
    private PIDController pivotController;
    private final PIDController rollerController; // TODO: find how to implement this
    private final SingleJointedArmSim sim;
    private IntakeVisualizer visualizer;
    
    public IntakeSim() {
        DCMotor gearbox = DCMotor.getKrakenX60(1);
        LinearSystem<N2, N1, N2> system = LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), Settings.Intake.JKgMetersSquared,Settings.Intake.GEAR_RATIO);
        intakePivotMotor = new DCMotorSim(system, gearbox);
        intakeRollerMotor = new DCMotorSim(system, gearbox);

        visualizer = IntakeVisualizer.getInstance();
        pivotController = new PIDController(
            Settings.Intake.Pivot.kP,
            Settings.Intake.Pivot.kI,
            Settings.Intake.Pivot.kD
        );
        rollerController = new PIDController(
            Settings.Intake.Roller.kP,
            Settings.Intake.Roller.kI,
            Settings.Intake.Roller.kD        
        );
        sim = new SingleJointedArmSim(
            system,
            gearbox,
            Settings.Intake.GEAR_RATIO,
            Settings.Intake.ARM_LENGTH,
            Settings.Intake.PIVOT_MIN_ANGLE,
            Settings.Intake.PIVOT_MAX_ANGLE,
            true,
            Settings.Intake.INITIAL_POSITION);
    }
    @Override
    public Rotation2d getRelativeAngle() {
        return Rotation2d.fromDegrees(SLMath.clamp(Math.toDegrees(intakePivotMotor.getAngularPositionRad()),
        Settings.Intake.PIVOT_MIN_ANGLE, Settings.Intake.PIVOT_MAX_ANGLE));
    }

    public double getRollerRPM() {
        return intakeRollerMotor.getAngularVelocityRPM();
    }

    @Override
    public void periodic() {
        super.periodic();

        double voltage = pivotController.calculate(sim.getAngleRads(), Math.toRadians(getState().getAngle()));
        sim.setInputVoltage(voltage);
        sim.update(Settings.DT);
        visualizer.updatePivotAngle(new Rotation2d(sim.getAngleRads()));

        SmartDashboard.putNumber("Pivot/voltage", voltage);
        SmartDashboard.putNumber("Pivot/currentAngle", visualizer.getAngles());
        SmartDashboard.putNumber("Pivot/targetAngle", getState().getAngle());
    }
}