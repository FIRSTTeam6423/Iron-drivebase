// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.IronUtil.IronController;
import frc.robot.subsystems.Drive.Drive;

public class DriveCommands {
  private final Drive drive;
  private final IronController controller;
  private final double maxLinearSpeed, maxRotationSpeed;

  public DriveCommands(Drive drive, IronController controller, double maxLinearSpeed, double maxRotationSpeed) {
    this.drive = drive;
    this.controller = controller;
    this.maxLinearSpeed = maxLinearSpeed;
    this.maxRotationSpeed = maxRotationSpeed;
  }
  
  public Command axisRotationControl() {
    return drive.drive(
      () ->
        DriverStation.isAutonomous()
          ? 0
          :  -controller.getJoystickAxis(XboxController.Axis.kLeftX.value) 
            * ((controller.getLeftTriggerAxis() > controller.triggerDeadband)
              ? maxLinearSpeed * .35
              : maxLinearSpeed),
      () -> 
        DriverStation.isAutonomous()
          ? 0
          : -controller.getJoystickAxis(XboxController.Axis.kLeftY.value)
          * ((controller.getLeftTriggerAxis() > controller.triggerDeadband)
            ? maxLinearSpeed * .35
            : maxLinearSpeed),
      () -> 
        DriverStation.isAutonomous()
          ? 0
          : controller.getJoystickAxis(XboxController.Axis.kRightX.value)
          * ((controller.getLeftTriggerAxis() > controller.triggerDeadband)
            ? maxLinearSpeed * .35
            : maxLinearSpeed)
    );
  }

  public Command flickStickControl() {
    return drive.drive(
      () ->
        DriverStation.isAutonomous()
          ? 0
          :  -controller.getJoystickAxis(XboxController.Axis.kLeftX.value) 
            * ((controller.getLeftTriggerAxis() > controller.triggerDeadband)
              ? maxLinearSpeed * .35
              : maxLinearSpeed),
      () -> 
        DriverStation.isAutonomous()
          ? 0
          : -controller.getJoystickAxis(XboxController.Axis.kLeftY.value)
          * ((controller.getLeftTriggerAxis() > controller.triggerDeadband)
            ? maxLinearSpeed * .35
            : maxLinearSpeed),
      () -> controller.getJoystickCircularAngle(drive.getGyroHeading(), XboxController.Axis.kRightX.value, XboxController.Axis.kRightY.value)
      // () -> controller.joystickDeadbandOutput(XboxController.Axis.kRightX.value) * maxRotationSpeed 
    );
  }
}
