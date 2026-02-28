package com.stuypulse.robot.subsystems.shooter;
import java.util.function.DoubleSupplier;

import com.stuypulse.robot.Robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Shooter extends SubsystemBase {
    public static final Shooter instance;
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

    public enum ShooterState {
        IDLE(() -> 0.0), 
        SHOOTING(() -> Shooter.getInstance().getShootSpeed()),
        FERRYING(() -> Shooter.getInstance().getFerrySpeed()); // supplier because idk

        private final DoubleSupplier targetRPM;
        
        private ShooterState(DoubleSupplier targetRPM) {
            this.targetRPM = targetRPM;
        }

        public double getTargetRPM() {
            return targetRPM.getAsDouble();
        }
    }

    public abstract double getShootSpeed();
    public abstract double getFerrySpeed();

    public Shooter() {
        this.state = ShooterState.IDLE;
    }

    public void setState(ShooterState state) {
        this.state = state;
    }

    public ShooterState getState() {
        return this.state;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Shooter/State", state.name());
        SmartDashboard.putString("States/Shooter", state.name());
    }
}