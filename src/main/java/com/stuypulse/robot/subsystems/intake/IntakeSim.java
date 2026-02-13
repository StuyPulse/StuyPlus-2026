package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.intake.Intake.*;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class IntakeSim extends Intake{
    private DCMotorSim intakeRollerMotor;
    private DCMotorSim intakePivotMotor;
    private AnglePIDController pivotController;
    private final PIDController rollerController;
    private final SingleJointedArmSim sim;
    
    public IntakeSim() {
        pivotController = new AnglePIDController(
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
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), Settings.Intake.JKgMetersSquared,Settings.Intake.GEAR_RATIO),
            DCMotor.getKrakenX60(1), 
            Settings.Intake.GEAR_RATIO, 
            Settings.Intake.ARM_LENGTH, Settings.Intake.PIVOT_MIN_ANGLE, 
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
    public void periodic(){
        super.periodic();
        sim.setInputVoltage(Settings.Intake.IDLE_VOLTAGE);
        sim.update(Settings.dT);
        double rollerOutput = rollerController.calculate(getRollerRPM());
        
        SmartDashboard.putNumber("Pivot/targetAngle", getState().getAngle());
        SmartDashboard.putNumber("Pivot/currentAngle", sim.getAngleRads() * (180 / Math.PI));
        SmartDashboard.putNumber("Pivot/voltage", Settings.Intake.IDLE_VOLTAGE);
        SmartDashboard.putNumber("Roller/RPM", rollerOutput);
        
    }
}