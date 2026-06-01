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

    /**
     * Creates a tunable wait command based on a DogLog tunable's DoubleSubscriber
     * @param tunable The DoubleSubscriber instance returned by creation of a {@link DogLog#tunable(String, double)}
     */
    public TunableWaitCommand(DoubleSubscriber tunable) {
        super(tunableDefaultValue);
        this.tunable = tunable;
        SendableRegistry.setName(this, getName() + ": " + tunable.get() + " seconds");
    }

    /**
     * Creates a tunable wait command by <strong>creating</strong> a tunable at the path specified
     * 
     * Initializes the tunable to a default value of 0.0
     * @param tunableName path and name of the {@link DogLog#tunable(String, double)} created
     */
    public TunableWaitCommand(String tunableName) {
        this(DogLog.tunable(makeTunableName(tunableName),
                tunableDefaultValue));
    }

    /**
     * Creates a tunable wait command by <strong>creating</strong> a tunable at the path specified
     * @param tunableName path and name of the {@link DogLog#tunable(String, double)} created
     * @param defaultValue value the tunable will be initialized to
     */
    public TunableWaitCommand(String tunableName, double defaultValue) {
        this(DogLog.tunable(makeTunableName(tunableName),
                defaultValue));
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