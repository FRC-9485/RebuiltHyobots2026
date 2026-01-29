package frc.frc_java9485.loggers;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CustomBooleanLog extends BooleanLogEntry {

  private static boolean isFMS;
  private String name;
  private boolean value;

  public CustomBooleanLog(String name) {
    super(DataLogManager.getLog(), name);
    this.name = name;
    CustomBooleanLog.isFMS = DriverStation.isFMSAttached();
    this.value = false;
    append(false);
  }

  public void append(boolean newValue) {
    if (DriverStation.isEnabled() && newValue != value) {
      this.value = newValue;
      super.append(newValue);
    }

    if (!isFMS) {
      SmartDashboard.putBoolean(name, newValue);
    }
  }
}
