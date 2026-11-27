#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

# Run the program
# ---------------
#
# To run the program you will need to explicitly use the ws-client option:
#
#    # Windows
#    py -3 -m robotpy sim --xrp
#
#    # Linux/macOS/Chromebook
#    python -m robotpy sim --xrp
#
# By default the WPILib simulation GUI will be displayed. To disable the display
# you can add the --nogui option
#

import os
import wpilib
import wpimath
import math

import drivetrain
import arm

# If your XRP isn't at the default address, set that here
os.environ["HALSIMXRP_HOST"] = "192.168.42.1"
os.environ["HALSIMXRP_PORT"] = "3540"

DEADBAND = 0.1
kS = 0.5

def squareSpeed(speed: float) -> float:
    return math.copysign(speed**2,speed)

def apply_ks(speed: float, ks: float = kS) -> float:
    if speed == 0.0:
        return 0.0
    return math.copysign(ks + (1.0 - ks) * abs(speed), speed)

def feedforward(fwd: float,
                turn: float,
                ks: float = kS,
                deadband: float = DEADBAND) -> tuple[float, float]:
    # Apply 2D radial deadband
    magnitude = math.hypot(fwd, turn)
    if magnitude <= deadband:
        return 0.0, 0.0

    # Rescale magnitude smoothly from [deadband, 1.0] -> [0.0, 1.0]
    norm_mag = min((magnitude - deadband) / (1.0 - deadband), 1.0)

    # Apply kS to the overall magnitude, not individual axes
    scaled_mag = ks + (1.0 - ks) * norm_mag

    # Project back onto fwd and turn using the original angle
    scale_factor = scaled_mag / magnitude
    return fwd * scale_factor, turn * scale_factor


class MyRobot(wpilib.TimedRobot):

    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        # Assumes a gamepad plugged into channel 0
        self.joystick = wpilib.Joystick(0)

        # create a drivetrain (contains two motors, two encoders and a gyro)
        self.drivetrain = drivetrain.Drivetrain()
        self.arm = arm.Arm()



    def robotPeriodic(self) -> None:
        """This function is called periodically (by default, 50 times per second)"""
        super().robotPeriodic()

        # 1. driving: take the speeds from joystick
        fwd_speed, turn_speed = feedforward(
            -self.joystick.getRawAxis(1),
            self.joystick.getRawAxis(0),
            kS,
            DEADBAND,
            )
        #if self.drivetrain.getDistanceToObstacle() < 0.15 and fwd_speed > 0:
        #    fwd_speed = 0.

        self.drivetrain.arcadeDrive(fwd_speed,(turn_speed))
        self.drivetrain.periodic()  # updates odometry

    def teleopPeriodic(self) -> None:
        """This function is called periodically when in operator control mode"""


    def teleopInit(self) -> None:
        """This function is initially when operator control mode runs"""
        self.drivetrain.stop()


    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        self.drivetrain.stop()


    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""


    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        self.drivetrain.stop()


    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""


    def testInit(self) -> None:
        """Cancels all running commands at the start of test mode"""
        self.drivetrain.stop()
