#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include "util/TalonFXConfig.hpp"

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
 *  - The Current Limit
 *  - The Open Loop Ramp Rate
 */
namespace Motors {

    /** Classes to store all of the values a motor needs */
    namespace Intake {
        TalonFXConfig PIVOT_CONFIG = TalonFXConfig()
            .withSupplyCurrentLimitAmps(units::current::ampere_t(30))
            .withCurrentLimitAmps(units::current::ampere_t(40))
            .withInvertedValue(InvertedValue.Clockwise_Positive) // not necessarily true, get inverted val
            .withNeutralMode(NeutralModeValue.Brake)
            .withPIDConstants(Gains.Intake.kP.get(), Gains.Intake.kI.get(), Gains.Intake.kD.get(), 0)
            .withSensorToMechanismRatio(Settings.Intake.Pivot.GEAR_RATIO);
        
        TalonFXConfig LEFT_ROLLER_CONFIG = TalonFXConfig() // TODO: apply later
            .withCurrentLimitAmps(units::current::ampere_t(50))
            .withInvertedValue(InvertedValue.CounterClockwise_Positive) // not necessarily true, get inverted val
            .withNeutralMode(NeutralModeValue.Coast)
            .withSensorToMechanismRatio(Settings.Intake.Roller.GEAR_RATIO);
        
        TalonFXConfig RIGHT_ROLLER_CONFIG = TalonFXConfig() // TODO: apply later
            .withCurrentLimitAmps(units::current::ampere_t(50))
            .withInvertedValue(InvertedValue.Clockwise_Positive) 
            .withNeutralMode(NeutralModeValue.Coast)
            .withSensorToMechanismRatio(Settings.Intake.Roller.GEAR_RATIO);

    }

    namespace Feeder {
        // TODO: get values after motor pinion swap
        TalonFXConfig LEADER_CONFIG = TalonFXConfig()
            .withCurrentLimitAmps(units::current::ampere_t(80))
			.withRampRate(units::time::second_t(0.25))
			.withNeutralMode(NeutralModeValue.Coast)
			.withInvertedValue(InvertedValue.CounterClockwise_Positive);
    }

    namespace Shooter {
        TalonFXConfig SHOOTER_MOTOR_CONFIG = TalonFXConfig()
            .withPIDConstants(Gains.Shooter.kP, Gains.Shooter.kI, Gains.Shooter.kD, 0)
            .withCurrentLimitAmps(units::current::ampere_t(80))
			.withRampRate(units::time::second_t(0.25))
			.withNeutralMode(NeutralModeValue.Coast)
            .withFFConstants(Gains.Shooter.kS, Gains.Shooter.kV, Gains.Shooter.kA, 0)
			.withInvertedValue(InvertedValue.CounterClockwise_Positive);

        
        TalonFXConfig SHOOTER_MOTOR_RIGHT = TalonFXConfig()
            .withPIDConstants(Gains.Shooter.kP, Gains.Shooter.kI, Gains.Shooter.kD, 0)
            .withCurrentLimitAmps(units::current::ampere_t(80))
			.withRampRate(units::time::second_t(0.25))
			.withNeutralMode(NeutralModeValue.Coast)
            .withFFConstants(Gains.Shooter.kS, Gains.Shooter.kV, Gains.Shooter.kA, 0)
			.withInvertedValue(InvertedValue.Clockwise_Positive);

        // TalonFXConfig SHOOTER_MOTOR_RIGHT = TalonFXConfig()
        //     .withPIDConstants(Gains.Shooter.kP, Gains.Shooter.kI, Gains.Shooter.kD, 0)
        //     .withCurrentLimitAmps(80)
		// 	.withRampRate(0.25)
		// 	.withNeutralMode(NeutralModeValue.Coast)
        //     .withFFConstants(Gains.Shooter.kS, Gains.Shooter.kV, Gains.Shooter.kA, 0)
		// 	.withInvertedValue(InvertedValue.Clockwise_Positive);

        TalonFXConfig SHOOTER_MOTOR_LEFT = TalonFXConfig()
            .withPIDConstants(Gains.Shooter.kP, Gains.Shooter.kI, Gains.Shooter.kD, 0)
            .withCurrentLimitAmps(units::current::ampere_t(80))
			.withRampRate(units::time::second_t(0.25))
			.withNeutralMode(NeutralModeValue.Coast)
            .withFFConstants(Gains.Shooter.kS, Gains.Shooter.kV, Gains.Shooter.kA, 0)
			.withInvertedValue(InvertedValue.CounterClockwise_Positive);
    }

    namespace Handoff {
        TalonFXConfig HANDOFF_MOTOR_CONFIG = TalonFXConfig()
            .withCurrentLimitAmps(units::current::ampere_t(80))
            .withRampRate(units::time::second_t(0.25))
            .withNeutralMode(NeutralModeValue.Coast)
            .withInvertedValue(InvertedValue.CounterClockwise_Positive);
    }
}