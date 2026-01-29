package frc.frc_java9485.loggers;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CustomStringLog extends StringLogEntry {

  private static boolean isFMS;
  private String name;
  private String loggedValue;

  public CustomStringLog(String name) {
    super(DataLogManager.getLog(), name);
    this.name = name;
    CustomStringLog.isFMS = DriverStation.isFMSAttached();
    this.loggedValue = "";
    append("");
  }

  public void append(String value) {
    if (DriverStation.isEnabled() && value != loggedValue) {
      this.loggedValue = value;
      super.append(value);
    }

    if (!isFMS) {
      SmartDashboard.putString(name, value);
    }
  }
}
