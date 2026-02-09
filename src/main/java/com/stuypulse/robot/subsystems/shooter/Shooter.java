package com.stuypulse.robot.subsystems.shooter;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.stuypulse.robot.constants.Settings;

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
        IDLE(0), 
        SHOOTING(Shooter.getInstance().getShootSpeed()),
        FERRYING(Shooter.getInstance().getFerrySpeed());

        private final double targetRPM;
        
        private ShooterState(double targetRPM) {
            this.targetRPM = targetRPM;
        }

        public double getSpeed(){
            return this.targetRPM;
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
}