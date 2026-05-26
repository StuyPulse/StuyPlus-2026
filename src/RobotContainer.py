import wpilib
import commands2

from typing import ClassVar

class RobotContainer:
    # Gamepad
    driver : commands2.button.CommandXboxController 

    # Subsystems
    # TODO: insert subsystems here once implemented

    # Autons
    __auton_chooser : ClassVar[wpilib.SendableChooser[commands2.Command]]

    def __init__(self):
        pass
    
    def configureDefaultCommands(self):
        pass

    def configureButtonBindings(self):
        pass
    
    def configureAutons(self):
        pass

    def getAutonomousCommand(self) -> commands2.Command:
        return self.__auton_chooser.getSelected()