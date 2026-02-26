package com.stuypulse.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class RobotVisualizer {
    public static RobotVisualizer instance;

    static {
        instance = new RobotVisualizer();
    }

    public static RobotVisualizer getInstance() {
        return instance;
    }

    private final Mechanism2d canvas;
    private double width, height;
    // we only have feeder, intake, and shooter visualizers. copy and paste now

    
    //feeder:
    private MechanismRoot2d feederRoot;
    private final MechanismLigament2d[] feederSpokes;

    
    //intake:
    private MechanismRoot2d intakeRoot;
    private MechanismLigament2d intakePivot;
    private final MechanismLigament2d[] intakeSpokes; // for the rollers

    //shooter:
    private final MechanismRoot2d shooterRoot;
    private final MechanismLigament2d[] shooterSpokes;
    
    private RobotVisualizer() {
        width = 67; // not final
        height = 67;// not final

        canvas = new Mechanism2d(width, height);

        // Feeder
        feederRoot = canvas.getRoot("Feeder Root", 20, 10); // TODO: figure out positioning of each root
        feederSpokes = new MechanismLigament2d[4];
        
        
        // Shooter
        shooterRoot = canvas.getRoot("Shooter Root", 45, 45);
        shooterSpokes = new MechanismLigament2d[4];

        // intake :D
        intakeRoot = canvas.getRoot("Intake Root", 10, 10);
        intakeSpokes = new MechanismLigament2d[4];


        intakePivot = new MechanismLigament2d(
            "Intake Arm",
            Units.inchesToMeters(5),
            IntakeState.IDLE.getTargetAngle().getDegrees(), 4,
            new Color8Bit(Color.kAliceBlue)
        );
        intakePivot.append(intakePivot); // TODO: ആറ് ഏഴ്
        
        // fill all the spoke arrays
        for (int i = 0; i < 4; i++) {
            feederSpokes[i] = new MechanismLigament2d("Feeder Spoke " + i, 6.7, 90 * i, 2, new Color8Bit(Color.kWhite));
            feederRoot.append(feederSpokes[i]);
            shooterSpokes[i] = new MechanismLigament2d("Shooter Spoke " + i, 6.7, 90 * i, 2, new Color8Bit(Color.kYellow));
            shooterRoot.append(shooterSpokes[i]);
            intakeSpokes[i] = new MechanismLigament2d("Intake Spoke " + i, 6.7, 90 * i, 2, new Color8Bit(Color.kRed));
            intakeRoot.append(intakeSpokes[i]); // TODO: spoke lengths and thicknesses
        }
    }

    public void updateFeeder(double RPM) {
        double rot = RPM * 6 * Settings.DT;
        for (int i = 0; i < 4; i++) {
            feederSpokes[i].setAngle(feederSpokes[i].getAngle() + rot);
        }
        SmartDashboard.putData("Visualizers/Robot", canvas);
    }

    public void updateShooter(double RPM){
        double rot = RPM * 6 * Settings.DT;
        for (int i = 0; i < 4; i++) {
            // ആറ് ഏഴ്
            shooterSpokes[i].setAngle(shooterSpokes[i].getAngle() + rot);
        }
        SmartDashboard.putData("Visualizers/Robot", canvas);
    }

    public void updateIntake(Rotation2d pivotAngle, double RPM) {
        intakePivot.setAngle(pivotAngle);
        double rot = RPM * 6 * Settings.DT;
        for (int i = 0; i < 4; i++) {
            intakeSpokes[i].setAngle(intakeSpokes[i].getAngle() + rot);
        }
        
        intakePivot.setColor(new Color8Bit(Color.kCoral));
        SmartDashboard.putData("Visualizers/Robot", canvas);
    }

    public double getIntakeAngle() {
        return intakePivot.getAngle();
    }
}