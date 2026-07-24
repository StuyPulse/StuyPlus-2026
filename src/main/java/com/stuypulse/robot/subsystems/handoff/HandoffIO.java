package com.stuypulse.robot.subsystems.handoff;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

public interface HandoffIO {
    class HandoffIOInputs {
        Voltage voltage = Volts.of(0.0);
        AngularVelocity angularVelocity = RPM.of(0.0);
        boolean isStalling = false;
    }
    void updateInputs(HandoffIOInputs inputs);

    void stopMotors();
    void setMotorVoltage(Voltage voltage);

    default void updateSim() {};
    default void logHardwareSignals() {};
}
