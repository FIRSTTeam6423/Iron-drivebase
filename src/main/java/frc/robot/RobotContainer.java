// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.IronUtil.IronController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.Auton;
import frc.robot.commands.DriveJoystick;
import frc.robot.subsystems.Drive.Drive;

public class RobotContainer {
  // * ------ SUBSYSTEMS ------
  public final Drive drive;

  // * ------ COMMANDS ------
  private final DriveJoystick driveJoystick; 

  // * ------ AUTO (womp womp) ------
  public final SendableChooser<Command> autoSelector;

  // * ------ CONTROLLERS ------
  public static IronController driverController = new IronController(0, ControllerConstants.DRIVER_DEADBAND, ControllerConstants.DRIVER_TRIGGER_DEADBAND);
  public static IronController operatorController = new IronController(1, ControllerConstants.OPERATOR_DEADBAND, ControllerConstants.OPERATOR_TRIGGER_DEADBAND);
  
  public RobotContainer() {
    //* init subsystems here
    drive = new Drive();
    if (Robot.isReal()) {
    
    } else {

    }

    driveJoystick = new DriveJoystick(drive, driverController, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED);

    autoSelector = Auton.configureAutos(drive);
    SmartDashboard.putData("Auto Chooser", autoSelector);
    configureDefaultCommands();
    configureBindings();
  }

  public void configureDefaultCommands() {
    drive.setDefaultCommand(
      driveJoystick.execute()
    );
  }

  private void configureBindings() {
    // ---- TRIGGERS ----

    // ---- DRIVER BINDS ----

    // ---- OPERATOR BINDS ----
  }
}
