package com.stuypulse.robot.commands.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.shooter.Shooter;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterFirstShotIncrease extends Command {
    private final Shooter shooter;
    private final LinearFilter currentFilter;
    private final Debouncer shotFinished;
    private double previousCurrent;

    public ShooterFirstShotIncrease() {
        shooter = Shooter.getInstance();
        currentFilter = LinearFilter.singlePoleIIR(0.1, Settings.DT.in(Seconds));
        shotFinished = new Debouncer(Settings.Shooter.FIRST_SHOT_DEBOUNCE.in(Seconds), DebounceType.kRising);
        previousCurrent = 0;
    }

    @Override
    public void initialize() {
        shooter.addToBonusVelocity(Settings.Shooter.FIRST_SHOT_BONUS.get());
        shooter.setGainSlot(1);
    }

    private boolean currentDecreasing() {
        final double rawCurrent = Shooter.getInstance().getCurrentAngularVelocity().in(RPM);
        final double filteredCurrent = currentFilter.calculate(rawCurrent);

        final boolean decreasing = filteredCurrent < this.previousCurrent;

        this.previousCurrent = filteredCurrent;

        DogLog.log("Shooter/First Shot/Raw Current", rawCurrent);
        DogLog.log("Shooter/First Shot/Filtered Current", filteredCurrent);
        DogLog.log("Shooter/First Shot/Is Decreasing", decreasing);

        return decreasing;
    }

    @Override
    public boolean isFinished() {
        return shotFinished.calculate(this.currentDecreasing());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.resetBonusVelocity();
        shooter.setGainSlot(0);
    }
}
