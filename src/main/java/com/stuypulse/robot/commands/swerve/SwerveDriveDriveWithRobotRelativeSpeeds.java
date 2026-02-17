
package com.stuypulse.robot.commands.swerve;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveDriveWithRobotRelativeSpeeds extends Command{
    private final CommandSwerveDrivetrain swerve;
    private double velocityX;
    private double velocityY;
    private double angularVelocity;

    public SwerveDriveDriveWithRobotRelativeSpeeds(double velocityX, double velocityY, double angularVelocity){
        this.swerve = CommandSwerveDrivetrain.getInstance();
        this.velocityX = velocityX;
        this.velocityY = velocityY;
        this.angularVelocity = angularVelocity;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.setControl(swerve.getRobotCentricSwerveRequest()
            .withVelocityX(velocityX)
            .withVelocityY(velocityY)
            .withRotationalRate(angularVelocity));
    }
}
