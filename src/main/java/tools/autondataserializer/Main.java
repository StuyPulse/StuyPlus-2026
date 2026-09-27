package tools.autondataserializer;

import java.nio.file.Path;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.util.PathUtil.AutonConfig;

import edu.wpi.first.hal.HAL;

public class Main {

    public static void main(String[] args) {
        HAL.initialize(500, 0);

        new RobotContainer();
        try {
            if (args.length > 0 && args[0] instanceof String) {
                AutonDataSerializer.writeAutonData(Path.of(args[0]), AutonConfig.getAll());
            } else {
                AutonDataSerializer.writeAutonData(Path.of("assets/auton_data.json"), AutonConfig.getAll());
            }
        } catch (java.io.IOException e) {
            e.printStackTrace();
        }
    }
}
