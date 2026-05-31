package com.stuypulse.robot.commands.compound;

import java.util.UUID;

import org.jspecify.annotations.NonNull;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

/**
 * A command that does nothing but takes a specified amount of time to finish
 * based on a tunable value.
 */
public class TunableWaitCommand extends WaitCommand {
    private static final double tunableDefaultValue = 0.0;
    private final DoubleSubscriber tunable;

    @NonNull
    private static String makeTunableName(String rawName) {
        return rawName == null ? "Tunable_Wait_" + UUID.randomUUID().toString() : rawName;
    }

    public TunableWaitCommand(DoubleSubscriber tunable) {
        super(tunableDefaultValue);
        this.tunable = tunable;
        SendableRegistry.setName(this, getName() + ": " + tunable.get() + " seconds");
    }

    public TunableWaitCommand(String tunableName) {
        this(DogLog.tunable(makeTunableName(tunableName),
                tunableDefaultValue));
        SendableRegistry.setName(this, getName() + ": " + tunable.get() + " seconds");
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(tunable.get());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("duration", () -> tunable.get(), null);
    }
}
