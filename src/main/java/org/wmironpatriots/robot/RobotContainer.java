// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wmironpatriots.robot;

import static org.wmironpatriots.robot.Constants.DriveConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import org.wmironpatriots.robot.Constants.ControllerConstants;
import org.wmironpatriots.robot.Util.CommandController;
import org.wmironpatriots.robot.commands.Auton;
import org.wmironpatriots.robot.commands.DriveCommands;
import org.wmironpatriots.robot.subsystems.Drive.Drive;

public class RobotContainer {
  // * ------ SUBSYSTEMS ------
  public final Drive drive;

  // * ------ COMMAND GROUPS ------
  private final DriveCommands driveJoystick; 

  // * ------ AUTO (womp womp) ------
  public final SendableChooser<Command> autoSelector;

  // * ------ CONTROLLERS ------
  public static CommandController driverController = new CommandController(
    0, 
    ControllerConstants.DRIVER_DEADBAND, 
    ControllerConstants.DRIVER_TRIGGER_DEADBAND
  );
  public static CommandController operatorController = new CommandController(
    1, 
    ControllerConstants.OPERATOR_DEADBAND, 
    ControllerConstants.OPERATOR_TRIGGER_DEADBAND
  );
  
  public RobotContainer() {
    //* init subsystems here
    drive = new Drive();
    if (Robot.isReal()) {} else {}

    driveJoystick = new DriveCommands(drive, driverController, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED);

    autoSelector = Auton.configureAutos(drive);
    SmartDashboard.putData("Auto Chooser", autoSelector);
    configureDefaultCommands();
    configureBindings();
  }

  public void configureDefaultCommands() {
    drive.setDefaultCommand(
      driveJoystick.flickStickControl()
    );
  }

  private void configureBindings() {
    // ---- TRIGGERS ----

    // ---- DRIVER BINDS ----
    // * Left Trigger Axis is used for slow mode

    // ---- OPERATOR BINDS ----
  }
}
