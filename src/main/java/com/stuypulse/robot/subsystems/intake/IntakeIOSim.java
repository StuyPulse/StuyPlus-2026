package com.stuypulse.robot.subsystems.intake;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;

import com.stuypulse.robot.util.simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.TalonFXSimulation;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import static edu.wpi.first.units.Units.*;

public class IntakeIOSim implements IntakeIO {
    private final SystemSim<SingleJointedArmSim> pivotSim;
    private final SystemSim<DCMotorSim> rollerSim;

    private final TalonFXSimulation pivotMotor;

    private final TalonFXSimulation rollerMotorLeft; // Leader
    private final TalonFXSimulation rollerMotorRight; // Follower

    private final PositionTorqueCurrentFOC positionController;
    private final VoltageOut homingController;
    private final TorqueCurrentFOC pushdownController;
    private final DutyCycleOut rollerController;
    private final Follower rollerFollower;

    private final BooleanSupplier pivotStalling;
    private final BooleanSupplier leftRollerStalling;
    private final BooleanSupplier rightRollerStalling;

    private final Debouncer leftRollerDebouncer;
    private final Debouncer rightRollerDebouncer;

    private static final Angle LIMIT_SWITCH_ANGLE = Settings.Intake.Pivot.DEPLOY_ANGLE;
    private static final Angle LIMIT_SWITCH_TOLERANCE = Degrees.of(2.0);

    public IntakeIOSim(int pivotMotorId, int rollerMotorLeftId, int rollerMotorRightId, double pivotGearRatio, double rollerGearRatio) {
        this.pivotSim = SystemSim.of(
            new SingleJointedArmSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX60(1),
                        Settings.Intake.Pivot.MOI.in(KilogramSquareMeters),
                        Settings.Intake.Pivot.GEAR_RATIO),
                DCMotor.getKrakenX60(1),
                Settings.Intake.Pivot.GEAR_RATIO,
                Settings.Intake.Pivot.PIVOT_ARM_LENGTH.in(Meters),
                Settings.Intake.Pivot.MAX_ANGLE.in(Radians),
                Settings.Intake.Pivot.MIN_ANGLE.in(Radians), // reversed because negative?
                true,
                Settings.Intake.Pivot.INITIAL_ANGLE.in(Radians))
        );

        this.rollerSim = SystemSim.of(
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX60(2),
                        Settings.Intake.Roller.J.in(KilogramSquareMeters),
                        Settings.Intake.Roller.GEAR_RATIO),
                DCMotor.getKrakenX60(2))
        );
        
        this.pivotMotor = new TalonFXSimulation(pivotMotorId, pivotGearRatio, pivotSim);
        Motors.Intake.PIVOT_CONFIG.configure(pivotMotor);

        // zero it at the up position
        pivotMotor.setPosition(Settings.Intake.Pivot.INITIAL_ANGLE);

        this.rollerMotorLeft = new TalonFXSimulation(rollerMotorLeftId, rollerGearRatio, rollerSim);
        Motors.Intake.LEFT_ROLLER_CONFIG.configure(rollerMotorLeft);

        this.rollerMotorRight = new TalonFXSimulation(rollerMotorRightId, rollerGearRatio, rollerSim);
        Motors.Intake.RIGHT_ROLLER_CONFIG.configure(rollerMotorRight);

        this.positionController = new PositionTorqueCurrentFOC(0.0);
        this.homingController = new VoltageOut(Settings.Intake.Pivot.HOMING_DOWN_VOLTAGE).withEnableFOC(true);
        this.pushdownController = new TorqueCurrentFOC(Settings.Intake.Pivot.PUSHDOWN_CURRENT.getAsDouble());
        this.rollerController = new DutyCycleOut(0.0).withEnableFOC(true);
        this.rollerFollower = new Follower(rollerMotorLeftId, MotorAlignmentValue.Opposed);

        rollerMotorRight.setControl(rollerFollower);

        this.pivotStalling = () -> pivotMotor.getStatorCurrent().getValue().gt(Settings.Intake.Pivot.STALL_CURRENT);
        this.leftRollerStalling = () -> rollerMotorLeft.getStatorCurrent().getValue().gt(Settings.Intake.Roller.STALL_CURRENT);
        this.rightRollerStalling = () -> rollerMotorRight.getStatorCurrent().getValue().gt(Settings.Intake.Roller.STALL_CURRENT);

        this.leftRollerDebouncer = new Debouncer(Settings.Intake.Roller.STALL_DEBOUNCE_SEC.in(Seconds), DebounceType.kBoth);
        this.rightRollerDebouncer = new Debouncer(Settings.Intake.Roller.STALL_DEBOUNCE_SEC.in(Seconds), DebounceType.kBoth);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.isLimitSwitchHit = simulatedIsLimitSwitchHit();
        inputs.relativePosition = pivotMotor.getPosition().getValue();
        inputs.rollerVelocity = rollerMotorLeft.getVelocity().getValue();

        inputs.isPivotStalling = pivotStalling.getAsBoolean();
        inputs.isLeftRollerStalling = leftRollerDebouncer.calculate(leftRollerStalling.getAsBoolean());
        inputs.isRightRollerStalling = rightRollerDebouncer.calculate(rightRollerStalling.getAsBoolean());

        inputs.isApplyingPushdown = pivotMotor.getAppliedControl() == pushdownController;
    }

    private boolean simulatedIsLimitSwitchHit() {
        return pivotMotor.getPosition().getValue()
            .minus(LIMIT_SWITCH_ANGLE)
            .abs(Degrees)
            < LIMIT_SWITCH_TOLERANCE.in(Degrees);
    }

    @Override
    public void seedPivotAngle(Angle angle) {
        pivotMotor.setPosition(angle);
    }

    @Override
    public void applyPushdown() {
        pivotMotor.setControl(pushdownController);
    }

    @Override
    public void applyHoming() {
        pivotMotor.setControl(homingController);
    }

    @Override
    public void applyPosition(Angle angle, int gainSlot) {
        pivotMotor.setControl(positionController.withPosition(angle).withSlot(gainSlot));
    }

    @Override
    public void applyRollerDutyCycle(double dutyCycle) {
        rollerMotorLeft.setControl(rollerController.withOutput(dutyCycle));
    }

    @Override
    public void stopRollerMotors() {
        rollerMotorLeft.stopMotor();
        rollerMotorRight.stopMotor();
        // re-add the follow control after stopMotor removes it
        rollerMotorRight.setControl(rollerFollower);
    }

    @Override
    public void stopPivotMotor() {
        pivotMotor.stopMotor();
    }

    @Override
    public void updateSim() {
        pivotSim.update(Settings.DT);
        pivotMotor.refresh();

        rollerSim.update(Settings.DT);
        rollerMotorLeft.refresh();
        rollerMotorRight.refresh();
    }
}
