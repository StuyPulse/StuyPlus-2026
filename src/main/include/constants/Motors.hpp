#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include "util/TalonFXConfig.hpp"
#include "Settings.hpp"
#include "Gains.hpp"

#include <units/time.h>
#include <units/voltage.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/moment_of_inertia.h>
#include <units/angle.h>
#include <units/math.h>

/*-
 * File containing all of the configurations that different motors require.
 *
 * Such configurations include:
 *  - If it is Inverted
 *  - The Idle Mode of the Motor
 *  - The Current_Amps Limit
 *  - The Open Loop Ramp Rate
 */
namespace Motors {

    /** Classes to store all of the values a motor needs */
    namespace Intake {
        inline TalonFXConfig PIVOT_CONFIG = TalonFXConfig()
            .withSupplyCurrentLimitAmps(30_A)
            .withStatorCurrentLimitAmps(40_A)
            .withInvertedValue(ctre::phoenix6::signals::InvertedValue::Clockwise_Positive) // not necessarily true, get inverted val
            .withNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake)
            .withPIDConstants(Gains::Intake::kP, Gains::Intake::kI, Gains::Intake::kD, 0)
            .withSensorToMechanismRatio(Settings::Intake::Pivot::GEAR_RATIO);
        
        inline TalonFXConfig LEFT_ROLLER_CONFIG = TalonFXConfig() // TODO: apply later
            .withStatorCurrentLimitAmps(50_A)
            .withInvertedValue(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive) // not necessarily true, get inverted val
            .withNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast)
            .withSensorToMechanismRatio(Settings::Intake::Roller::GEAR_RATIO);
        
        inline TalonFXConfig RIGHT_ROLLER_CONFIG = TalonFXConfig() // TODO: apply later
            .withStatorCurrentLimitAmps(50_A)
            .withInvertedValue(ctre::phoenix6::signals::InvertedValue::Clockwise_Positive) 
            .withNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast)
            .withSensorToMechanismRatio(Settings::Intake::Roller::GEAR_RATIO);

    }

    namespace Feeder {
        // TODO: get values after motor pinion swap
        inline TalonFXConfig LEADER_CONFIG = TalonFXConfig()
            .withStatorCurrentLimitAmps(80_A)
			.withRampRate(0.25_s)
			.withNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast)
			.withInvertedValue(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive);
    }

    namespace Shooter {
        inline TalonFXConfig SHOOTER_MOTOR_LEFT = TalonFXConfig()
				.withPIDConstants(Gains::Shooter::kP, Gains::Shooter::kI, Gains::Shooter::kD, 0)
				.withPIDConstants(Gains::Shooter::FirstShot::kP, Gains::Shooter::FirstShot::kI, Gains::Shooter::FirstShot::kD, 1)
				.withSupplyCurrentLimitAmps(200_A)
				.withStatorCurrentLimitAmps(200_A)
				.withNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast)
				.withFFConstants(Gains::Shooter::kS, Gains::Shooter::kV, Gains::Shooter::kA, 0)
				.withInvertedValue(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive);

		inline TalonFXConfig SHOOTER_MOTOR_CENTER = TalonFXConfig()
				.withPIDConstants(Gains::Shooter::kP, Gains::Shooter::kI, Gains::Shooter::kD, 0)
				.withPIDConstants(Gains::Shooter::FirstShot::kP, Gains::Shooter::FirstShot::kI, Gains::Shooter::FirstShot::kD, 1)
				.withSupplyCurrentLimitAmps(200_A)
				.withStatorCurrentLimitAmps(200_A)
				.withNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast)
				.withFFConstants(Gains::Shooter::kS, Gains::Shooter::kV, Gains::Shooter::kA, 0)
				.withInvertedValue(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive);

		inline TalonFXConfig SHOOTER_MOTOR_RIGHT = TalonFXConfig()
				.withPIDConstants(Gains::Shooter::kP, Gains::Shooter::kI, Gains::Shooter::kD, 0)
				.withPIDConstants(Gains::Shooter::FirstShot::kP, Gains::Shooter::FirstShot::kI, Gains::Shooter::FirstShot::kD, 1)
				.withSupplyCurrentLimitAmps(200_A)
				.withStatorCurrentLimitAmps(200_A)
				.withNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast)
				.withFFConstants(Gains::Shooter::kS, Gains::Shooter::kV, Gains::Shooter::kA, 0)
				.withFFConstants(Gains::Shooter::kS, Gains::Shooter::kV, Gains::Shooter::kA, 1)
				.withInvertedValue(ctre::phoenix6::signals::InvertedValue::Clockwise_Positive);
    }

    namespace Handoff {
        inline TalonFXConfig HANDOFF_MOTOR_CONFIG = TalonFXConfig()
            .withStatorCurrentLimitAmps(80_A)
            .withRampRate(0.25_s)
            .withNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast)
            .withInvertedValue(ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive);
    }
}