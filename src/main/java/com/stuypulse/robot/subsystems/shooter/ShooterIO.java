package com.stuypulse.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.*;

public interface ShooterIO {
    public static class MotorInputs {
        Voltage voltage = Volts.of(0.0);
        Current supplyCurrent = Amps.of(0.0);
        Current statorCurrent = Amps.of(0.0);
        AngularVelocity angularVelocity = RPM.of(0.0);

        public void log(String path) {
            DogLog.log(path + "/MotorVoltage", voltage);
            DogLog.log(path + "/Supply Current", supplyCurrent);
            DogLog.log(path + "/Stator Current", statorCurrent);
            DogLog.log(path + "/Velocity", angularVelocity);
        }
    }

    public static class ShooterIOInputs {
        /** Representative measurements for the shooter subsystem (leader motor) */
        Angle position = Degrees.of(0.0);
        Voltage voltage = Volts.of(0.0);
        AngularVelocity angularVelocity = RPM.of(0.0);
        
        /** Individual motor telemetry for logging */
        MotorInputs leftMotor = new MotorInputs();
        MotorInputs centerMotor = new MotorInputs();
        MotorInputs rightMotor = new MotorInputs();
    }
    void updateInputs(ShooterIOInputs inputs);

    void stopMotors();
    void setShooterAngularVelocity(AngularVelocity velocity, int gainSlot);

    default void updateSim() {};
    default void logHardwareSignals() {};

    default void setVoltage(Voltage voltage) {};
}
