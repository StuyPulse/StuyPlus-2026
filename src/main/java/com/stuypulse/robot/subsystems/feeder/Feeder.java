package com.stuypulse.robot.subsystems.feeder;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.commands.feeder.FeederScramble;
import com.stuypulse.robot.commands.feeder.FeederSetForward;
import com.stuypulse.robot.commands.feeder.FeederSetIdle;
import com.stuypulse.robot.commands.feeder.FeederSetReverse;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.feeder.FeederIO.FeederIOInputs;
import com.stuypulse.robot.subsystems.feeder.FeederIO.FeederState;

import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.shooter.Shooter;

import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
    private static Feeder instance;

    private FeederIOInputs inputs;
    private FeederIO io;

    static {
        instance = new Feeder();

        SmartDashboard.putData(new FeederSetForward());
        SmartDashboard.putData(new FeederScramble());
        SmartDashboard.putData(new FeederSetIdle());
        SmartDashboard.putData(new FeederSetReverse());
    }

    public static Feeder getInstance() {
        return instance;
    }

    private Feeder() {
        this.inputs = new FeederIOInputs();
        this.io = Robot.isReal() 
            ? new FeederIOImpl(Ports.Feeder.FEEDER_MOTOR, Settings.CANBUS)
            : new FeederIOSim(Ports.Feeder.FEEDER_MOTOR, Settings.Feeder.GEAR_RATIO);
    }

    public AngularVelocity getCurrentAngularVelocity() {
        return inputs.currentAngularVelocity;
    }

    public FeederState getState() {
        return io.getState();
    }

    public void setState(FeederState state) {
        io.setState(state);
    }

    @Override
    public void periodic() {
        if (!Settings.EnabledSubsystems.FEEDER.get()) {
            io.stopMotors();
            io.updateInputs(inputs);
            return;
        }

        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
        Shooter shooter = Shooter.getInstance();
        if (!(swerve.isAlignedToTarget(Field.getHubPose()))
                && shooter.getState() == ShooterState.SHOOT) {
            io.setState(FeederState.IDLE);
        }
        if (!(swerve.isAlignedToTarget(Field.getFerryZonePose(swerve.getPose().getTranslation())))
                && shooter.getState() == ShooterState.FERRY) {
            io.setState(FeederState.IDLE);
        }

        io.setTargetVoltage(getState().getTargetVoltage());

        io.updateInputs(inputs);
        io.log(inputs);
    }
}
