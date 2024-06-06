// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wmironpatriots.robot.subsystems.Drive.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO {

  private final Rotation2d rotation = new Rotation2d();

  @Override
  public void updateInputs(GyroInputs inputs) {}

  @Override
  public Rotation2d getGyroHeading() {
    return rotation;
  }

  @Override
  public void resetGyro() {}
}
