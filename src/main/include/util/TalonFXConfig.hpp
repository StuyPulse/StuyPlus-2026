#include <units/time.h>
#include <units/voltage.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/moment_of_inertia.h>
#include <units/angle.h>
#include <units/math.h>

#include <ctre/phoenix6/configs/ClosedLoopRampsConfigs.hpp>
#include <ctre/phoenix6/configs/CurrentLimitsConfigs.hpp>
#include <ctre/phoenix6/configs/FeedbackConfigs.hpp>
#include <ctre/phoenix6/configs/MotionMagicConfigs.hpp>
#include <ctre/phoenix6/configs/MotorOutputConfigs.hpp>
#include <ctre/phoenix6/configs/OpenLoopRampsConfigs.hpp>
#include <ctre/phoenix6/configs/Slot0Configs.hpp>
#include <ctre/phoenix6/configs/Slot1Configs.hpp>
#include <ctre/phoenix6/configs/Slot2Configs.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

class TalonFXConfig {
    private:
        ctre::phoenix6::configs::TalonFXConfiguration configuration{};
        ctre::phoenix6::configs::Slot0Configs slot0Configs{};
        ctre::phoenix6::configs::Slot1Configs slot1Configs{};
        ctre::phoenix6::configs::Slot2Configs slot2Configs{};
        ctre::phoenix6::configs::MotorOutputConfigs motorOutputConfigs{};
        ctre::phoenix6::configs::ClosedLoopRampsConfigs closedLoopRampsConfigs{};
        ctre::phoenix6::configs::OpenLoopRampsConfigs openLoopRampsConfigs{};
        ctre::phoenix6::configs::CurrentLimitsConfigs currentLimitsConfigs{};
        ctre::phoenix6::configs::FeedbackConfigs feedbackConfigs{};
        ctre::phoenix6::configs::MotionMagicConfigs motionMagicConfigs{};
    public:
        void configure(ctre::phoenix6::hardware::TalonFX motor) {
            motor.GetConfigurator().Apply(configuration);
        }

        // SLOT 0 CONFIGS

        TalonFXConfig& withPIDConstants(double kP, double kI, double kD, int slot) {
            switch (slot) {
                case 0:
                    slot0Configs.WithKP(kP);
                    slot0Configs.WithKI(kI);
                    slot0Configs.WithKD(kD);
                    configuration.WithSlot0(slot0Configs);
                    break;
                case 1:
                    slot1Configs.WithKP(kP);
                    slot1Configs.WithKI(kI);
                    slot1Configs.WithKD(kD);
                    configuration.WithSlot1(slot1Configs);
                    break;
                case 2:
                    slot2Configs.WithKP(kP);
                    slot2Configs.WithKI(kI);
                    slot2Configs.WithKD(kD);
                    configuration.WithSlot2(slot2Configs);
                    break;
            }
            return *this;
        }

        TalonFXConfig& withFFConstants(double kS, double kV, double kA, int slot) {
            return withFFConstants(kS, kV, kA, 0, slot);
        }

        TalonFXConfig& withFFConstants(double kS, double kV, double kA, double kG, int slot) {
            switch (slot) {
                case 0:
                    slot0Configs.WithKS(kS);
                    slot0Configs.WithKV(kV);
                    slot0Configs.WithKA(kA);
                    slot0Configs.WithKG(kG);
                    configuration.WithSlot0(slot0Configs);
                    break;
                case 1:
                    slot1Configs.WithKS(kS);
                    slot1Configs.WithKV(kV);
                    slot1Configs.WithKA(kA);
                    slot1Configs.WithKG(kG);
                    configuration.WithSlot1(slot1Configs);
                    break;
                case 2:
                    slot2Configs.WithKS(kS);
                    slot2Configs.WithKV(kV);
                    slot2Configs.WithKA(kA);
                    slot2Configs.WithKG(kG);
                    configuration.WithSlot2(slot2Configs);
                    break;
            }
            return *this;
        }

        TalonFXConfig& withGravityType(ctre::phoenix6::signals::GravityTypeValue gravityType) {
            slot0Configs.WithGravityType(gravityType);
            slot1Configs.WithGravityType(gravityType);
            slot2Configs.WithGravityType(gravityType);

            configuration.WithSlot0(slot0Configs);
            configuration.WithSlot1(slot1Configs);
            configuration.WithSlot2(slot2Configs);

            return *this;
        }

        // MOTOR OUTPUT CONFIGS

        TalonFXConfig& withInvertedValue(ctre::phoenix6::signals::InvertedValue invertedValue) {
            motorOutputConfigs.WithInverted(invertedValue);

            configuration.WithMotorOutput(motorOutputConfigs);

            return *this;
        }

        TalonFXConfig& withNeutralMode(ctre::phoenix6::signals::NeutralModeValue neutralMode) {
            motorOutputConfigs.WithNeutralMode(neutralMode);

            configuration.WithMotorOutput(motorOutputConfigs);

            return *this;
        }

        // RAMP RATE CONFIGS

        TalonFXConfig& withRampRate(units::second_t rampRate) {
            closedLoopRampsConfigs.WithDutyCycleClosedLoopRampPeriod(rampRate);
            closedLoopRampsConfigs.WithTorqueClosedLoopRampPeriod(rampRate);
            closedLoopRampsConfigs.WithVoltageClosedLoopRampPeriod(rampRate);

            openLoopRampsConfigs.WithDutyCycleOpenLoopRampPeriod(rampRate);
            openLoopRampsConfigs.WithTorqueOpenLoopRampPeriod(rampRate);
            openLoopRampsConfigs.WithVoltageOpenLoopRampPeriod(rampRate);

            configuration.WithClosedLoopRamps(closedLoopRampsConfigs);
            configuration.WithOpenLoopRamps(openLoopRampsConfigs);

            return *this;
        }

        // CURRENT LIMIT CONFIGS

        TalonFXConfig& withCurrentLimitAmps(units::current::ampere_t currentLimitAmps) {
            currentLimitsConfigs.WithStatorCurrentLimit(currentLimitAmps);
            currentLimitsConfigs.WithStatorCurrentLimitEnable(true);

            configuration.WithCurrentLimits(currentLimitsConfigs);

            return *this;
        }

        TalonFXConfig& withSupplyCurrentLimitAmps(units::current::ampere_t currentLimitAmps) {
            currentLimitsConfigs.WithSupplyCurrentLimit(currentLimitAmps);
            currentLimitsConfigs.WithSupplyCurrentLimitEnable(true);

            configuration.WithCurrentLimits(currentLimitsConfigs);

            return *this;
        }

        // MOTION MAGIC CONFIGS

        TalonFXConfig& withMotionProfile(units::angular_velocity::turns_per_second_t maxVelocity, units::angular_acceleration::turns_per_second_squared_t maxAcceleration) {
            motionMagicConfigs.WithMotionMagicCruiseVelocity(maxVelocity);
            motionMagicConfigs.WithMotionMagicAcceleration(maxAcceleration);

            configuration.WithMotionMagic(motionMagicConfigs);

            return *this;
        }

        // FEEDBACK CONFIGS

        TalonFXConfig& withRemoteSensor(
                int ID, ctre::phoenix6::signals::FeedbackSensorSourceValue source, double rotorToSensorRatio) {
            feedbackConfigs.WithFeedbackRemoteSensorID(ID);
            feedbackConfigs.WithFeedbackSensorSource(source);
            feedbackConfigs.WithRotorToSensorRatio(rotorToSensorRatio);

            configuration.WithFeedback(feedbackConfigs);

            return *this;
        }

        TalonFXConfig& withSensorToMechanismRatio(double sensorToMechanismRatio) {
            feedbackConfigs.WithSensorToMechanismRatio(sensorToMechanismRatio);

            configuration.WithFeedback(feedbackConfigs);

            return *this;
        }
};
