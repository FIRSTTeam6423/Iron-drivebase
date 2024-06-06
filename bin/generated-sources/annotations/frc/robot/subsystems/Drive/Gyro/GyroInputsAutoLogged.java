package frc.robot.subsystems.Drive.Gyro;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.wmironpatriots.robot.subsystems.Drive.Gyro.GyroIO;

public class GyroInputsAutoLogged extends GyroIO.GyroInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
  }

  @Override
  public void fromLog(LogTable table) {
  }

  public GyroInputsAutoLogged clone() {
    GyroInputsAutoLogged copy = new GyroInputsAutoLogged();
    return copy;
  }
}
