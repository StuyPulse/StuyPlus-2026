package com.stuypulse.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import com.stuypulse.robot.Robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final static Shooter instance;
    private ShooterState state;

    static {
        if (Robot.isReal()) {
            instance = new ShooterImpl();
        } else {
            instance = new ShooterSim();
        }
    }

    public static Shooter getInstance() {
        return instance;
    }

    protected Shooter() {
        setState(ShooterState.IDLE);
    }

    public void setState(ShooterState state) {
        this.state = state;
    }

    public ShooterState getState() {
        return this.state;
    }

    public enum ShooterState {

        SOTM(() -> 0.0), // TODO: Make actual suppliers
        FOTM(() -> 0.0),
        IDLE(() -> 0.0),
        SHOOT(() -> 0.0),
        FERRY(() -> 0.0);

        private DoubleSupplier RPMSupplier;

        private ShooterState(DoubleSupplier RPMSupplier) {
            this.RPMSupplier = RPMSupplier;
        }

        public double getRPM() {
            return RPMSupplier.getAsDouble();
        }
    }
}
