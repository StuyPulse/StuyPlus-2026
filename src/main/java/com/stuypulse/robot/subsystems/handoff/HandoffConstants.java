package com.stuypulse.robot.subsystems.handoff;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.constants.Motors.TalonFXConfig;

import edu.wpi.first.units.measure.Voltage;

public interface HandoffConstants {
    public interface HandoffSettings {
        Voltage IDLE_VOLTAGE = Volts.of(0.0);

        Voltage FORWARD_VOLTAGE = Volts.of(12.0);

        Voltage REVERSE_VOLTAGE = Volts.of(-10.0);

        double STALL_CURRENT = 67;

        // TODO: get and maybe convert to wpilib units
        double STALL_DEBOUNCE = 67;

        double J_KG_METERS_SQUARED = 1;

        double GEAR_RATIO = 1.0 / 3.0; // 1:3
    }

    public interface HandoffDeviceIds {

        int HANDOFF_MOTOR = 50;
    }

    public interface HandoffMotorConfigs {

		TalonFXConfig HANDOFF_MOTOR_CONFIG = new TalonFXConfig()
			.withStatorCurrentLimitAmps(80)
			.withNeutralMode(NeutralModeValue.Coast)
			.withInvertedValue(InvertedValue.CounterClockwise_Positive)
			.withSensorToMechanismRatio(HandoffConstants.HandoffSettings.GEAR_RATIO);
	}
}
