package com.stuypulse.robot.subsystems.shooter;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import com.stuypulse.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.robot.util.shooter.InterpolationCalculator;
import com.stuypulse.robot.util.simulation.TalonFXSimIds;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

public class Shooter extends SubsystemBase {
    private static Shooter instance;

    private ShooterIO io;
    private ShooterIOInputs inputs;

    private ShooterState state;

    private AngularVelocity bonusVelocity;
    private int gainSlot;

    private Optional<Voltage> voltageOverride; 

    /** Enum representing the different possible states of the shooter. */
    public enum ShooterState {

        // SOTM(() -> 0.0), 
        // FOTM(() -> 0.0),
        /** Shooter doesn't run. */
        IDLE(() -> 0.0),
        /** Shooter wheels spin at it's target RPM, interpolated based on distance to hub. */
        // SHOOT(Settings.Shooter.SHOOT_TUNING_RPM), //TODO:Replace with interpolated RPM after data is gathered
        /** Shooter wheels spin at it's target RPM, interpolated based on distance to ferry zone. */
        // FERRY(Settings.Shooter.FERRY_TUNING_RPM),
        SHOOT(() -> InterpolationCalculator.interpolateShotInfo().targetRPM()),
        FERRY(() -> InterpolationCalculator.interpolateFerryingInfo().targetRPM()),
        /** Shooter wheels spin at a predetermined constant rate without interpolation. */
        MANUAL_HUB(Settings.Shooter.MANUAL_HUB_RPM);

        /** The supplier for the target RPM of the shooter in the corresponding state. */
        private DoubleSupplier RPMSupplier;

        /**
         * Constructs a ShooterState with the given supplier for the target RPM of the shooter.
         * @param RPMSupplier the supplier for the target RPM of the shooter in the corresponding state
         */
        private ShooterState(DoubleSupplier RPMSupplier) {
            this.RPMSupplier = RPMSupplier;
        }

        /**
         * Gets the base target angular velocity of the shooter in the corresponding state before any bonus velocity is added.
         * @return the base target angular velocity of the shooter
         */
        public AngularVelocity getBaseTargetAngularVelocity() {
            return RPM.of(RPMSupplier.getAsDouble());
        }
    }

    static {
        instance = new Shooter();
    }

    public static Shooter getInstance() {
        return instance;
    }

    private Shooter() {
        setState(ShooterState.SHOOT);

        this.io = Robot.isReal()
            ? new ShooterIOImpl(Ports.Shooter.SHOOTER_MOTOR_LEFT, Ports.Shooter.SHOOTER_MOTOR_CENTER, Ports.Shooter.SHOOTER_MOTOR_RIGHT, Settings.CANBUS)
            : new ShooterIOSim(TalonFXSimIds.get(), TalonFXSimIds.get(), TalonFXSimIds.get(), Settings.Shooter.GEAR_RATIO);
        this.inputs = new ShooterIOInputs();

        setGainSlot(0);
        this.bonusVelocity = RPM.of(0);
        
        this.voltageOverride = Optional.empty();
    }

    public void setState(ShooterState state) {
        this.state = state;
    }

    public ShooterState getState() {
        return state;
    }

    public void setGainSlot(int gainSlot) {
        this.gainSlot = gainSlot;
    }

    public void addToBonusVelocity(AngularVelocity velocity) {
        this.bonusVelocity = bonusVelocity.plus(velocity);
    }

    public void addToBonusVelocity(double velocity) {
        this.bonusVelocity = bonusVelocity.plus(RPM.of(velocity));
    }

    public void resetBonusVelocity() {
        this.bonusVelocity = RPM.of(0);
    }

    public AngularVelocity getTargetAngularVelocity() {
        return state.getBaseTargetAngularVelocity().plus(this.bonusVelocity);
    }

    public AngularVelocity getCurrentAngularVelocity() {
        return inputs.angularVelocity;
    }

    public boolean shooterSpunUp() {
        return getCurrentAngularVelocity().gte(getTargetAngularVelocity().minus(Settings.Shooter.SHOOTER_SPUN_UP_TOLERANCE));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        if (!Settings.EnabledSubsystems.SHOOTER.get()) {
            io.stopMotors();
            return;
        }
        if (voltageOverride.isPresent()) {
            io.setVoltageOverride(voltageOverride.get());
            return;
        }

        io.setShooterAngularVelocity(getTargetAngularVelocity(), gainSlot);

        DogLog.log("Shooter/Bonus Velocity", bonusVelocity.in(RPM));
        DogLog.log("Shooter/Target RPM", getTargetAngularVelocity().in(RPM));
        DogLog.log("Shooter/Current RPM", getCurrentAngularVelocity().in(RPM));

        DogLog.forceNt.log("Shooter/At Target RPM", shooterSpunUp());

        inputs.leftMotor.log("Shooter/Left");
        inputs.centerMotor.log("Shooter/Center");
        inputs.rightMotor.log("Shooter/Right");

        DogLog.log("Shooter/State", state.name());
        DogLog.forceNt.log("States/Shooter", state.name());
        io.logHardwareSignals();
    }

    @Override
    public void simulationPeriodic() {
        io.updateSim();
    }

    public void setVoltageOverride(Voltage voltage) {
        this.voltageOverride = Optional.of(voltage);
    }

    public SysIdRoutine getShooterSysIdRoutine() {
        return SysId.getRoutine(
            Settings.Shooter.RAMP_RATE,
            Settings.Shooter.STEP_VOLTAGE,
            "Shooter",
            this::setVoltageOverride,
            () -> inputs.position,
            () -> inputs.angularVelocity,
            () -> inputs.voltage,
            getInstance()
        );
    }
}
