package com.stuypulse.robot.subsystems.feeder;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public interface FeederIO {
    class FeederIOInputs {
        public AngularVelocity angularVelocity = RPM.of(0.0);
        public Voltage voltage = Volts.of(0.0); 
        public Angle position = Rotations.of(0.0);
    }
    
    void updateInputs(FeederIOInputs inputs);

    void stopMotors();
    void setTargetVoltage(Voltage voltage);

    default void updateSim() {};
    default void logHardwareSignals() {};
}
