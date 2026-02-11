package frc.frc_java9485.loggers;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LoggedPathPlannerPIDConstants {
    private final PIDController controller;

    public LoggedPathPlannerPIDConstants(String key, double kP, double kI, double kD, double iZone) {
        controller = new PIDController(kP, kI, kD);
        controller.setIZone(iZone);

        SmartDashboard.putData(key, controller);
    }

    public LoggedPathPlannerPIDConstants(String key, double kP, double kI, double kD) {
        this(key, kP, kD, kI, 1.0);
    }

    public PIDConstants getPIDConsants() {
        return new PIDConstants(controller.getP(),
                                controller.getI(),
                                controller.getD(),
                                controller.getIZone());
    }
}
