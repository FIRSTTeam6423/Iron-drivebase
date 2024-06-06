package frc.robot.subsystems.Drive.Module;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.wmironpatriots.robot.subsystems.Drive.Module.ModuleIO;

public class ModuleInputsAutoLogged extends ModuleIO.ModuleInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("PivotRotation", pivotRotation);
    table.put("PivotPosition", pivotPosition);
    table.put("PivotVelocity", pivotVelocity);
    table.put("PivotAppliedVoltage", pivotAppliedVoltage);
    table.put("DrivePosition", drivePosition);
    table.put("DriveVelocity", driveVelocity);
    table.put("DriveAppliedVoltage", driveAppliedVoltage);
  }

  @Override
  public void fromLog(LogTable table) {
    pivotRotation = table.get("PivotRotation", pivotRotation);
    pivotPosition = table.get("PivotPosition", pivotPosition);
    pivotVelocity = table.get("PivotVelocity", pivotVelocity);
    pivotAppliedVoltage = table.get("PivotAppliedVoltage", pivotAppliedVoltage);
    drivePosition = table.get("DrivePosition", drivePosition);
    driveVelocity = table.get("DriveVelocity", driveVelocity);
    driveAppliedVoltage = table.get("DriveAppliedVoltage", driveAppliedVoltage);
  }

  public ModuleInputsAutoLogged clone() {
    ModuleInputsAutoLogged copy = new ModuleInputsAutoLogged();
    copy.pivotRotation = this.pivotRotation;
    copy.pivotPosition = this.pivotPosition;
    copy.pivotVelocity = this.pivotVelocity;
    copy.pivotAppliedVoltage = this.pivotAppliedVoltage;
    copy.drivePosition = this.drivePosition;
    copy.driveVelocity = this.driveVelocity;
    copy.driveAppliedVoltage = this.driveAppliedVoltage;
    return copy;
  }
}
