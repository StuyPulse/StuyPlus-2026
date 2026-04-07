package com.stuypulse.robot.subsystems.intake;

import java.util.Optional;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.commands.intake.IntakeSetZero;
import com.stuypulse.robot.commands.intake.IntakeSetZeroAtBottom;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.RobotVisualizer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intake extends SubsystemBase {
    private static final Intake instance;
    private IntakeState state;

    static {
        if (Robot.isReal()) {
            instance = new IntakeImpl();
        } else {
            instance = new IntakeSim();
        }

        SmartDashboard.putData("Intake/Set Pivot 0", new IntakeSetZero());
        SmartDashboard.putData("Intake/Set Pivot 0 at Bottom", new IntakeSetZeroAtBottom());
    }

    public static Intake getInstance() {
        return instance;
    }

    private final BooleanPublisher atTolerance = NetworkTableInstance.getDefault()
            .getBooleanTopic("SmartDashboard/Intake/pivotAtTolerance").publish();
    private final BooleanPublisher pivotStalling = NetworkTableInstance.getDefault()
            .getBooleanTopic("SmartDashboard/Intake/pivotStalling").publish();
    private final BooleanPublisher rollersStalling = NetworkTableInstance.getDefault()
            .getBooleanTopic("SmartDashboard/Intake/rollersStalling").publish();

    public enum IntakeState {
        IDLE(Settings.Intake.IDLE_ANGLE, Settings.Intake.IDLE_DUTY_CYCLE), // (rollers do not run)
        INTAKE(Settings.Intake.PIVOT_DOWN_ANGLE, Settings.Intake.INTAKE_DUTY_CYCLE), // (sucks in the balls) [pivot
                                                                                     // down, rollers running]
        OUTTAKE(Settings.Intake.PIVOT_DOWN_ANGLE, Settings.Intake.OUTTAKE_DUTY_CYCLE), // (trips the balls out) [pivot
                                                                                       // down, rollers running reverse]
        DOWN(Settings.Intake.PIVOT_DOWN_ANGLE, Settings.Intake.IDLE_DUTY_CYCLE),
        HOMING_UP(Settings.Intake.PIVOT_INITIAL_ANGLE, Settings.Intake.IDLE_DUTY_CYCLE),
        HOMING_DOWN(Settings.Intake.PIVOT_DOWN_ANGLE, Settings.Intake.IDLE_DUTY_CYCLE);

        private double dutyCycle;
        private Rotation2d angle;

        private IntakeState(Rotation2d angle, double dutyCycle) {
            this.dutyCycle = dutyCycle;
            this.angle = angle;
        }

        public Rotation2d getTargetAngle() {
            return angle;
        }

        public double getTargetDutyCycle() {
            return dutyCycle;
        }
    }

    protected Intake() {
        this.state = IntakeState.IDLE;
    }

    public void setState(IntakeState state) {
        this.state = state;
    }

    public IntakeState getState() {
        return state;
    }

    public abstract Rotation2d getRelativePosition();

    public boolean atTargetAngle() {
        return Math.abs(
                (getRelativePosition().getRotations())
                        - getState().getTargetAngle().getRotations()) < Settings.Intake.ANGLE_TOLERANCE.getRotations();
    };

    public abstract void setPivotZero();

    public abstract void setPivotZeroAtBottom();

    public abstract double getRollerRPM();

    @Override
    public void periodic() {
        if (Settings.EnabledSubsystems.INTAKE.get()) {
            RobotVisualizer.getInstance().updateIntake(getRelativePosition(), getRollerRPM());
        } else {
            RobotVisualizer.getInstance().updateIntake(IntakeState.IDLE.getTargetAngle(),
                    IntakeState.IDLE.getTargetDutyCycle());
        }

        SmartDashboard.putString("Intake/Intake State", getState().name());
        SmartDashboard.putNumber("Intake/Roller Target Duty Cycle", getState().getTargetDutyCycle());
        SmartDashboard.putNumber("Intake/Roller RPM", getRollerRPM());
        SmartDashboard.putNumber("Intake/Target Angle", getState().getTargetAngle().getDegrees());
        SmartDashboard.putNumber("Intake/Pivot Angle (deg)", getRelativePosition().getDegrees());
        SmartDashboard.putBoolean("Intake/Pivot At Target Angle", atTargetAngle());
    }
}