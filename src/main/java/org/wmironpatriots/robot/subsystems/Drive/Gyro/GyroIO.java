// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.wmironpatriots.robot.subsystems.Drive.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

  @AutoLog
  static class GyroInputs {}

  void updateInputs(final GyroInputs inputs);

  /**
   * Gets gyro heading
   *
   * @return Rotation2d
   */
  Rotation2d getGyroHeading();

  /** Resets gyro */
  void resetGyro();
}
