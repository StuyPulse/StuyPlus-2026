package com.stuypulse.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

public interface FeederIO {
    class FeederIOInputs {
        AngularVelocity angularVelocity = RPM.of(0.0);
        Voltage voltage = Volts.of(0.0); 
    }
    
    void updateInputs(FeederIOInputs inputs);

    void stopMotors();
    void setMotorVoltage(Voltage voltage);

    default void updateSim() {};
    default void logHardwareSignals() {};
}
