# TODO: insert robot code here
from wpilib import TimedRobot
from wpilib import DriverStation
from wpilib import SmartDashboard
from wpilib import DataLogManager

from phoenix6 import signal_logger

from commands2 import Command
from commands2 import commandscheduler

from RobotContainer import RobotContainer
from typing import ClassVar

class Robot(TimedRobot):
    __RobotContainer : RobotContainer
    __auto : Command
    __alliance : ClassVar[DriverStation.Alliance]

    @staticmethod
    def isBlue() -> bool:
        # Return whether the current alliance is blue
        return DriverStation.getAlliance() == DriverStation.Alliance.Blue


    def robotInit(self):
        self.__RobotContainer = RobotContainer()
        # Initialize the stored alliance value
        if (DriverStation.isDSAttached()):
            self.__alliance = DriverStation.getAlliance()
        else:
            self.__alliance = DriverStation.Alliance.kBlue

        DataLogManager.start()
        DataLogManager.logNetworkTables(True)
        print("]LOGGING DIRECTORY]: " + DataLogManager.getLogDir())
        signal_logger.SignalLogger.start()

    def driverStationConnected(self):
        self.__alliance = DriverStation.getAlliance()

    def robotPeriodic(self):
        commandscheduler.CommandScheduler.getInstance().run()
        SmartDashboard.putNumber("Bot/Alliance", self.__alliance.name)
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime())
        SmartDashboard.putData("Command Scheduler", commandscheduler.CommandScheduler.getInstance())

    def teleopInit(self):
        autonWon = DriverStation.getGameSpecificMessage() == (self.__alliance.name[0].upper())
        SmartDashboard.putBoolean("Auton Won", autonWon)
    
    def testInit(self):
        commandscheduler.CommandScheduler.getInstance().cancelAll()
