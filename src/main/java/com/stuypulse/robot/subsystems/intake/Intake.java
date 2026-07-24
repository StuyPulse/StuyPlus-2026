package com.stuypulse.robot.subsystems.intake;

import java.util.Optional;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.*;

import com.stuypulse.robot.util.SysId;
import com.stuypulse.robot.util.simulation.TalonFXSimIds;

public class Intake extends SubsystemBase {
    private static Intake instance;

    private IntakeIO io;
    private IntakeIOInputs inputs;

    private IntakeState state;

    private Optional<Voltage> voltageOverride;

    /** Enum representing the different possible states of the intake. */
    public enum IntakeState {

        AGITATE_DOWN(Settings.Intake.Pivot.AGITATE_DOWN_ANGLE, Settings.Intake.Roller.INTAKE_DUTY_CYCLE),
        /** The intake is stowed and rollers are off. */
        IDLE(Settings.Intake.Pivot.STOW_ANGLE, 0),
        /** The intake is deployed but rollers are off. */
        DOWN(Settings.Intake.Pivot.DEPLOY_ANGLE, 0),
        /** The intake is deployed and rollers are running to take in gamepieces. */
        INTAKE(Settings.Intake.Pivot.DEPLOY_ANGLE, Settings.Intake.Roller.INTAKE_DUTY_CYCLE),
        /**
         * The intake is deployed and rollers are running in reverse to expel
         * gamepieces.
         */
        OUTTAKE(Settings.Intake.Pivot.DEPLOY_ANGLE, Settings.Intake.Roller.OUTTAKE_DUTY_CYCLE),
        /**
         * The intake is brought up repeatedly to an angle between stowed and deployed
         * to dislodge
         * gamepieces. Rollers do not run.
         */
        AGITATE(Settings.Intake.Pivot.AGITATE_UP_ANGLE, Settings.Intake.Roller.INTAKE_DUTY_CYCLE),

        /**
         * The intake is brought up once to an angle between stowed and deployed to
         * dislodge gamepieces.
         * Rollers do not run.
         */
        DIGEST(Settings.Intake.Pivot.DIGEST_ANGLE, 0),
        /** The intake is pushed against the bumpers to re-zero the pivot. */
        HOMING_DOWN(Settings.Intake.Pivot.DEPLOY_ANGLE, 0);

        /** The target angle of the intake pivot. */
        private Angle targetAngle;

        /** The target percentage of voltage of the intake rollers. */
        private double targetDutyCycle;

        /**
         * Constructs an IntakeState with its target values.
         *
         * @param targetAngle     In any unit, the target position of the intake pivot
         * @param targetDutyCycle In any unit, the target percentage of voltage of the
         *                        intake rollers
         */
        private IntakeState(Angle targetAngle, double targetDutyCycle) {
            this.targetAngle = targetAngle;
            this.targetDutyCycle = targetDutyCycle;
        }

        /**
         * Gets the target position of the pivot.
         *
         * @return the target angle
         */
        public Angle getTargetAngle() {
            return targetAngle;
        }

        /**
         * Gets the target position of the pivot.
         *
         * @return the target percentage of voltage of the intake rollers
         */
        public double getTargetDutyCycle() {
            return targetDutyCycle;
        }
    }

    static {
        instance = new Intake();
    }

    public static Intake getInstance() {
        return instance;
    }

    private Intake() {
        setState(IntakeState.IDLE);

        this.io = Robot.isReal()
                ? new IntakeIOImpl(Ports.Intake.PIVOT_LIMIT_SWITCH, Ports.Intake.INTAKE_PIVOT_MOTOR,
                        Ports.Intake.INTAKE_ROLLER_MOTOR_LEFT, Ports.Intake.INTAKE_ROLLER_MOTOR_RIGHT, Settings.CANBUS)
                : new IntakeIOSim(TalonFXSimIds.get(), TalonFXSimIds.get(), TalonFXSimIds.get(),
                        Settings.Intake.Pivot.GEAR_RATIO, Settings.Intake.Roller.GEAR_RATIO);

        this.inputs = new IntakeIOInputs();
        this.voltageOverride = Optional.empty();
    }

    public void setState(IntakeState state) {
        this.state = state;
    }

    public IntakeState getState() {
        return state;
    }

    public void seedPivotDeployed() {
        io.seedPivotAngle(Settings.Intake.Pivot.DEPLOY_ANGLE);
    }

    public void seedPivotStowed() {
        io.seedPivotAngle(Settings.Intake.Pivot.STOW_ANGLE);
    }

    public void seedPivotNinety() {
        io.seedPivotAngle(Degrees.of(-90));
    }

    public Angle getRelativePosition() {
        return inputs.relativePosition;
    }

    public boolean atTargetAngle() {
        return getRelativePosition().minus(getState().getTargetAngle())
                .abs(Rotations) < Settings.Intake.Pivot.ANGLE_TOLERANCE.in(Rotations);
    }

    public boolean isPivotAboveThreshold() {
        return getRelativePosition().gt(Settings.Intake.Pivot.PUSHDOWN_THRESHOLD);
    }

    private void applyPivotControl(IntakeState currentState) {
        switch (currentState) {
            case INTAKE, OUTTAKE, DOWN -> io.applyPushdown();
            case HOMING_DOWN -> io.applyHoming();
            case AGITATE, AGITATE_DOWN -> io.applyPosition(this.state.getTargetAngle(), 1);
            default -> io.applyPosition(this.state.getTargetAngle(), 0);
        }
    }

    private void applyRollerControl(IntakeState currentState) {
        io.applyRollerDutyCycle(currentState.getTargetDutyCycle());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        if (!Settings.EnabledSubsystems.INTAKE.get()) {
            io.stopAllMotors();
            return;
        }

        if (voltageOverride.isPresent()) {
            io.setVoltageOverride(voltageOverride.get());
            return;
        }

        if (inputs.isLimitSwitchHit) {
            seedPivotDeployed();
        }

        if (inputs.isPivotStalling || inputs.isLimitSwitchHit) {
            if (state == IntakeState.HOMING_DOWN || state == IntakeState.DOWN) {
                seedPivotDeployed();

                if (state == IntakeState.HOMING_DOWN) {
                    setState(IntakeState.INTAKE);
                }
            }
        }

        applyPivotControl(state);
        applyRollerControl(state);

        DogLog.forceNt.log("Intake/Rollers/Stalling", inputs.isLeftRollerStalling || inputs.isRightRollerStalling);
        DogLog.forceNt.log("Intake/Pivot/Pushing Down", inputs.isApplyingPushdown);

        DogLog.log("Intake/Pivot/Limit Switch Hit", inputs.isLimitSwitchHit);

        DogLog.log("Intake/Pivot/Target Angle", state.getTargetAngle().in(Degrees));
        DogLog.forceNt.log("Intake/Pivot/Current Angle", inputs.relativePosition.in(Degrees));
        DogLog.forceNt.log("Intake/Pivot/At Target Angle", atTargetAngle());
        DogLog.log("Intake/Pivot/Above Threshold", isPivotAboveThreshold());
        
        DogLog.forceNt.log("Intake/Rollers/Target Duty Cycle", state.getTargetDutyCycle());
        DogLog.log("Intake/Rollers/RPM", inputs.rollerVelocity.in(RPM));

        DogLog.log("Intake/State", state.name());
        DogLog.forceNt.log("States/Intake", state.name());
        io.logHardwareSignals();
    }

    @Override
    public void simulationPeriodic() {
        io.updateSim();
    }

    public void setPivotVoltageOverride(Voltage voltage) {
        this.voltageOverride = Optional.of(voltage);
    }   

    public SysIdRoutine getIntakeSysIdRoutine() {
        return SysId.getRoutine(
                Settings.Intake.Pivot.RAMP_RATE,
                Settings.Intake.Pivot.STEP_VOLTAGE,
                "Intake",
                this::setPivotVoltageOverride,
                () -> inputs.pivotPosition,
                () -> inputs.pivotVelocity,
                () -> inputs.pivotVoltage,
                getInstance());
    }
}
