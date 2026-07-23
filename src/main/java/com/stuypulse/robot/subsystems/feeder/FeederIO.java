package com.stuypulse.robot.subsystems.feeder;

import com.stuypulse.robot.constants.Settings;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public interface FeederIO {
    /** Enum representing the different possible states of the feeder. */
    public enum FeederState {
        /** Feeder is stopped. */
        IDLE(Volts.of(0.0)),
        /** Motors run forward to feed to the shooter. */
        FORWARD(Settings.Feeder.FORWARD_VOLTAGE),
        /** Motors run backward to work with the intake to outtake fuel from the robot. */
        REVERSE(Settings.Feeder.REVERSE_VOLTAGE);

        /** The target voltage of the feeder motors. */
        private Voltage targetVoltage;

        /**
         * Constructs a FeederState with the given target voltage.
         * @param targetVoltage the target voltage of the feeder motors in the corresponding state.
         */
        private FeederState(Voltage targetVoltage) {
            this.targetVoltage = targetVoltage;
        }

        /** 
         * Gets the target voltage of the feeder motors in the corresponding state.
         * @return the target voltage of the feeder motors
         */
        public Voltage getTargetVoltage() {
            return this.targetVoltage;
        }
    }

    class FeederIOInputs {
        public AngularVelocity currentAngularVelocity;
        public Voltage targetVoltage; 
    }
    void updateInputs(FeederIOInputs inputs);

    void setState(FeederState state);

    FeederState getState();

    void stopMotors();
    void setTargetVoltage(Voltage voltage);
    default void log(FeederIOInputs inputs) {
        FeederState currentState = getState();
        DogLog.log("Feeder/Target Voltage", inputs.targetVoltage);
        DogLog.log("Feeder/Current RPM", inputs.currentAngularVelocity.in(RPM));
        DogLog.log("Feeder/State", currentState.name());
        DogLog.forceNt.log("States/Feeder", currentState.name());
    }
}
