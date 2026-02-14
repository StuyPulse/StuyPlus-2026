    package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.intake.IntakeVisualizer;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
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
import static edu.wpi.first.units.Units.Degrees;


public class IntakeSim extends Intake{
    private DCMotorSim intakeRollerMotor;
    private DCMotorSim intakePivotMotor;
    private PIDController pivotController;
    private final PIDController rollerController;
    private final SingleJointedArmSim sim;
    private IntakeVisualizer visualizer;
    
    public IntakeSim() {
        DCMotor gearbox = DCMotor.getKrakenX60(1);
        LinearSystem<N2, N1, N2> system = LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), Settings.Intake.JKgMetersSquared,Settings.Intake.GEAR_RATIO);
        intakePivotMotor = new DCMotorSim(system, gearbox);
        intakeRollerMotor = new DCMotorSim(system, gearbox);

        visualizer = IntakeVisualizer.getInstance();
        pivotController = new PIDController(
            Gains.Intake.Pivot.kP,
            Gains.Intake.Pivot.kI,
            Gains.Intake.Pivot.kD
        );
        rollerController = new PIDController(
            Gains.Intake.Roller.kP,
            Gains.Intake.Roller.kI,
            Gains.Intake.Roller.kD        
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
        // double currentAngle = visualizer.getAngles();
        // double targetAngle = getState().getAngle(); // degrees
        // double difference = Math.abs(targetAngle - currentAngle);
        // double tolerance = Settings.Intake.ANGLE_TOLERANCE;

        // sim.setInputVoltage(getState().getVoltage());
        // sim.update(Settings.DT);
        // double rollerOutput = rollerController.calculate(getRollerRPM());
        // intakeRollerMotor.setInputVoltage(rollerOutput);
        // intakeRollerMotor.update(Settings.DT);
        
        // SmartDashboard.putNumber("Pivot/targetAngle", targetAngle);
        // SmartDashboard.putNumber("Pivot/currentAngle", currentAngle);
        // SmartDashboard.putNumber("Pivot/difference", difference);
        // SmartDashboard.putNumber("Pivot/tolerance", tolerance);
        // SmartDashboard.putNumber("Pivot/voltage", Settings.Intake.IDLE_VOLTAGE);
        // SmartDashboard.putNumber("Pivot/angularPosition", intakePivotMotor.getAngularPosition().in(Degrees));
        // SmartDashboard.putNumber("Roller/RPM", rollerOutput);
        

        // if (difference > tolerance) {
        //     if (currentAngle < targetAngle) { // let cA = 0 and tA = 90, then the error is 90, but if cA = 350 and tA = 10, then the error is -20, not 340 kys
        //         visualizer.updatePivotAngle(Rotation2d.fromDegrees(currentAngle + 1));
        //     } else if (currentAngle > targetAngle) {
        //         visualizer.updatePivotAngle(Rotation2d.fromDegrees(currentAngle - 1));
        //     }
        // }
        //pivotController.update(Angle.fromDegrees(targetAngle), Angle.fromDegrees(currentAngle));
        //visualizer.updatePivotAngle(Rotation2d.fromDegrees(pivotController.getOutput()));
        //visualizer.updatePivotAngle(new Rotation2d());
    }
}