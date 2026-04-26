#pragma once

#include <units/time.h>
#include <units/voltage.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/moment_of_inertia.h>
#include <units/angle.h>
#include <units/math.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/StateSpaceUtil.h>
#include <pathplanner/lib/path/PathConstraints.h>
#include <numbers>
#include <ctre/phoenix6/CANBus.hpp>

namespace Settings
{

    constexpr units::second_t DT = 0.020_s;
    constexpr bool DEBUG_MODE = true;
    constexpr ctre::phoenix6::CANBus CANBus = ctre::phoenix6::CANBus("rio");

    namespace EnabledSubsystems
    {
        constexpr bool FEEDER = true;
        constexpr bool INTAKE = true;
        constexpr bool LED = true;
        constexpr bool HANDOFF = true;
        constexpr bool SHOOTER = true;
        constexpr bool VISION = true;
        constexpr bool SWERVE = true;
    }

    namespace Vision
    {
        const frc::Pose2d INVALID_POSITION = frc::Pose2d{};
        constexpr double MAX_ANGULAR_VELOCITY_RAD_SEC = 2 * std::numbers::pi;
    }

    namespace Intake
    {

        namespace Pivot
        {
            // State angles
            const frc::Rotation2d INITIAL_ANGLE = frc::Rotation2d{0_deg};
            const frc::Rotation2d IDLE_ANGLE = frc::Rotation2d{0_deg};
            const frc::Rotation2d DOWN_ANGLE = frc::Rotation2d{102_deg};

            // Misc
            const frc::Rotation2d ANGLE_TOLERANCE = frc::Rotation2d{0.5_deg};
            const frc::Rotation2d PUSHDOWN_THRESHOLD = frc::Rotation2d{85_deg};
            constexpr units::ampere_t PUSHDOWN_CURRENT = 30_A;
            constexpr units::ampere_t STALL_CURRENT = 25_A;
            constexpr units::second_t STALL_DEBOUNCE = 0.0_s;
            constexpr units::volt_t HOMING_DOWN_VOLTAGE = 3_V;

            // SysId
            constexpr auto RAMP_RATE = 1_V / 1_s;
            constexpr units::volt_t STEP_VOLTAGE = 4_V;

            // Sim
            const frc::Rotation2d MIN_ANGLE = frc::Rotation2d{0_deg};
            const frc::Rotation2d MAX_ANGLE = frc::Rotation2d{102_deg};
            constexpr double GEAR_RATIO = 60.0;
            constexpr units::kilogram_square_meter_t J = 0.001_kg_sq_m;
        }

        namespace Roller
        {
            constexpr units::ampere_t STALL_CURRENT = 50_A;
            constexpr units::second_t STALL_DEBOUNCE = 0.1_s;

            constexpr double GEAR_RATIO = 16.0 / 27.0;
            constexpr units::kilogram_square_meter_t J = 0.001_kg_sq_m;

            constexpr double IDLE_DUTY_CYCLE = 0.0;
            constexpr double INTAKE_DUTY_CYCLE = 1.0;
            constexpr double OUTTAKE_DUTY_CYCLE = -1.0;
        }
    }

    namespace Feeder
    {
        constexpr double FEEDER_REVERSE_DUTY_CYCLE = -1.0;
        constexpr double FEEDER_FORWARD_DUTY_CYCLE = 1.0;

        constexpr double GEAR_RATIO = 1.0;
        constexpr units::kilogram_square_meter_t J = 0.001_kg_sq_m;
    }

    namespace Handoff
    {
        constexpr double IDLE_DUTY_CYCLE = 0.0;
        constexpr double FORWARD_DUTY_CYCLE = 1.0;
        constexpr double REVERSE_DUTY_CYCLE = -1.0;

        constexpr double STALL_CURRENT = 67.0;
        constexpr double STALL_DEBOUNCE = 67.0;

        constexpr double J_KG_METERS_SQUARED = 1.0;
        constexpr double SIM_GEAR_RATIO = 1.0;
    }

    namespace Shooter
    {
        constexpr units::second_t SHOOT_TIME_AUTO = 1.5_s;
        constexpr auto RAMP_RATE = 1_V / 1_s;
        constexpr units::volt_t STEP_VOLTAGE = 7_V;

        constexpr units::inch_t WHEEL_RADIUS = 4_in;
        constexpr units::inch_t FLYWHEEL_RADIUS = 3_in;

        // Sim
        constexpr units::kilogram_square_meter_t J = 0.1_kg_sq_m;
        constexpr double GEAR_RATIO = 0.1;

        constexpr units::revolutions_per_minute_t MANUAL_HUB_RPM = 3000_rpm;

        namespace RPMInterpolation
        {
            constexpr double distanceRPMInterpolationValues[][2] = {
                {1.0, 1000.0},
                {2.0, 1500.0},
                {3.0, 2000.0},
                {4.0, 2500.0},
                {5.0, 3000.0}};
        }

        namespace TOFInterpolation
        {
            constexpr double distanceTOFInterpolationValues[][2] = {
                {1.0, 0.50},
                {2.0, 0.75},
                {3.0, 1.00},
                {4.0, 1.25},
                {5.0, 1.50}};
        }

        namespace FerryRPMInterpolation
        {
            constexpr double ferryDistanceRPMInterpolation[][2] = {
                {1.0, 1000.0},
                {2.0, 1500.0},
                {3.0, 2000.0},
                {4.0, 2500.0},
                {5.0, 3000.0}};
        }

        namespace FerryTOFInterpolation
        {
            constexpr double FerryTOFInterpolationValues[][2] = {
                {1.0, 0.50},
                {2.0, 0.75},
                {3.0, 1.00},
                {4.0, 1.25},
                {5.0, 1.50}};
        }
    }

    namespace Swerve
    {
        constexpr double MODULE_VELOCITY_DEADBAND_M_PER_S = 0.1;
        constexpr double ROTATIONAL_DEADBAND_RAD_PER_S = 0.1;

        namespace Constraints
        {
            constexpr units::meters_per_second_t MAX_VELOCITY = 4.3_mps;
            constexpr units::meters_per_second_squared_t MAX_ACCEL = 15.0_mps_sq;
            constexpr units::radians_per_second_t MAX_ANGULAR_VEL = 400.0_deg_per_s;
            constexpr units::radians_per_second_squared_t MAX_ANGULAR_ACCEL = 900.0_deg_per_s_sq;

            constexpr pathplanner::PathConstraints DEFAULT_CONSTRAINTS{
                MAX_VELOCITY, MAX_ACCEL, MAX_ANGULAR_VEL, MAX_ANGULAR_ACCEL};
        }

        namespace Alignment
        {
            namespace Constraints
            {
                constexpr double DEFAULT_MAX_VELOCITY = 4.3;
                constexpr double DEFAULT_MAX_ACCELERATION = 15.0;
                constexpr double DEFAULT_MAX_ANGULAR_VELOCITY = units::math::fabs(400.0_deg).value();
                constexpr double DEFAULT_MAX_ANGULAR_ACCELERATION = units::math::fabs(900.0_deg).value();
            }

            namespace Tolerances
            {
                constexpr units::inch_t X_TOLERANCE = 2.0_in;
                constexpr units::inch_t Y_TOLERANCE = 2.0_in;
                const frc::Rotation2d THETA_TOLERANCE = frc::Rotation2d{1_deg};

                const frc::Pose2d POSE_TOLERANCE{
                    X_TOLERANCE.convert<units::meters>(),
                    Y_TOLERANCE.convert<units::meters>(),
                    THETA_TOLERANCE};

                constexpr units::meters_per_second_t MAX_VELOCITY_WHEN_ALIGNED = 0.15_mps;
                constexpr units::second_t ALIGNMENT_DEBOUNCE = 0.15_s;
            }

            namespace Targets
            {
                const frc::Rotation2d HUB_LEFT_CORNER = frc::Rotation2d{45_deg};
                const frc::Rotation2d HUB_RIGHT_CORNER = frc::Rotation2d{-45_deg};
            }
        }
    }

    namespace Driver
    {
        constexpr double BUZZ_TIME = 1.0;
        constexpr double BUZZ_INTENSITY = 1.0;

        namespace Drive
        {
            constexpr double DEADBAND = 0.05;
            constexpr double RC = 0.05;
            constexpr double POWER = 2.0;
        }

        namespace Turn
        {
            constexpr double DEADBAND = 0.05;
            constexpr double RC = 0.05;
            constexpr double POWER = 2.0;
        }
    }
}