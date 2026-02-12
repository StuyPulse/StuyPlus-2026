package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ShooterVisualizer {
    private static ShooterVisualizer instance;

    static {
        instance = new ShooterVisualizer();
    }

    public static ShooterVisualizer getInstance() {
        return instance;
    }

    private final Mechanism2d canvas;
    private final MechanismRoot2d shooterRoot;
    private final MechanismLigament2d spoke_1;
    private final MechanismLigament2d spoke_2;
    private final MechanismLigament2d spoke_3;
    private final MechanismLigament2d spoke_4;

    public ShooterVisualizer() {
        canvas = new Mechanism2d(25, 25);
        shooterRoot = canvas.getRoot("Root", 12.5, 12.5);
        spoke_1 = new MechanismLigament2d(
            "spoke_1",
            12,
            0,
            5,
            new Color8Bit(Color.kRed)
        );
        shooterRoot.append(spoke_1);
        spoke_2 = new MechanismLigament2d(
            "spoke_2",
            12,
            90,
            5,
            new Color8Bit(Color.kGreen)
        );
        shooterRoot.append(spoke_2);
        spoke_3 = new MechanismLigament2d(
            "spoke_3",
            12,
            180,
            5,
            new Color8Bit(Color.kBlue)
        );
        shooterRoot.append(spoke_3);
        spoke_4 = new MechanismLigament2d(
            "spoke_4",
            12,
            270,
            5,
            new Color8Bit(Color.kOrange)
        );
        shooterRoot.append(spoke_4);
    }

    public void update(double RPM) {
        SmartDashboard.putData("Visualizers/Shooter", canvas);
        double rot = (RPM * 60) * Settings.DT;
        spoke_1.setAngle(spoke_1.getAngle() + rot);
        spoke_2.setAngle(spoke_2.getAngle() + rot);
        spoke_3.setAngle(spoke_3.getAngle() + rot);
        spoke_4.setAngle(spoke_4.getAngle() + rot);
        SmartDashboard.putNumber("Shooter/visualizerRotation", rot);
    }
}
