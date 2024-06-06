// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wmironpatriots.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import static org.wmironpatriots.robot.Constants.DriveConstants.*;
import org.wmironpatriots.robot.subsystems.Drive.Drive;

public class Auton {
  public static SendableChooser<Command> configureAutos(Drive drive) {
    AutoBuilder.configureHolonomic(
        drive::getPose,
        drive::resetOdometry,
        drive::getChassisSpeeds,
        drive::setChassisSpeeds,
        new HolonomicPathFollowerConfig(
            new PIDConstants(ROBOT_TRANSLATION_P, ROBOT_TRANSLATION_I, ROBOT_TRANSLATION_D),
            new PIDConstants(ROBOT_ROTATION_P, ROBOT_ROTATION_I, ROBOT_ROTATION_D),
            4.5,
            Units.inchesToMeters(16.6),
            new ReplanningConfig()),
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        drive);

    SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();
    return chooser;
  }
}