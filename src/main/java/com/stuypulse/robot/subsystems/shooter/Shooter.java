package com.stuypulse.robot.subsystems.shooter;
import java.util.function.DoubleSupplier;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Shooter extends SubsystemBase {
    public static final Shooter instance;
    private ShooterState state;

    static {
        instance = new ShooterImpl();
    }

    public static Shooter getInstance() {
        return instance;
    }

    public enum ShooterState {
        IDLE(() -> 0.0, 0), 
        SHOOTING(() -> Shooter.getInstance().getShootSpeed(), Settings.Shooter.BOTTOM_MOTOR_RPM),
        FERRYING(() -> Shooter.getInstance().getFerrySpeed(), Settings.Shooter.BOTTOM_MOTOR_RPM); // supplier because idk

        private final DoubleSupplier targetRPM;
        private final double bottomMotorRPM;
        
        private ShooterState(DoubleSupplier targetRPM, double bottomMotorRPM) {
            this.targetRPM = targetRPM;
            this.bottomMotorRPM = bottomMotorRPM;
        }

        public double getShooterSpeed() {
            return targetRPM.getAsDouble();
        }
        
        public double getBottomMotorSpeed() {
            return bottomMotorRPM;
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
    }
}