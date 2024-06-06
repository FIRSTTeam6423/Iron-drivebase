// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wmironpatriots.robot.subsystems.Drive.Gyro;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;

public class NavxIO implements GyroIO {

  private final AHRS gyro;

  public NavxIO() {
    gyro = new AHRS();
  }

  @Override
  public void updateInputs(GyroInputs inputs) {}

  @Override
  public Rotation2d getGyroHeading() {
    return gyro.getRotation2d();
  }

  @Override
  public void resetGyro() {
    gyro.reset();
  }
}
