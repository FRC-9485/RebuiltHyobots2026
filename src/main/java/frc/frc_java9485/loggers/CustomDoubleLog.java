package frc.frc_java9485.loggers;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CustomDoubleLog extends DoubleLogEntry {

  private static boolean isFMS;
  private String name;
  private double loggedValue;

  public CustomDoubleLog(String name) {
    super(DataLogManager.getLog(), name);
    this.name = name;
    CustomDoubleLog.isFMS = DriverStation.isFMSAttached();
    this.loggedValue = 0.0;
    append(0.0);
  }

  public void append(double value) {
    if (DriverStation.isEnabled() && value != loggedValue) {
      this.loggedValue = value;
      super.append(value);
    }

    if (!isFMS) {
      SmartDashboard.putNumber(name, value);
    }
  }
}
