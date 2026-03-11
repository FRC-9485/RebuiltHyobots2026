package frc.robot.subsystems.mechanism.shooter.turret.turretOFC;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public interface TurretIO {

    @AutoLog
    public static class TurretIOinputs{
        public boolean inbuxing = false;
        public boolean turretAtSetpoint = false;
        public boolean automatic = false;
        public double turretSetpoint = 0;
        public double hoodSetpoint = 0;
    }

    public double getDegreesPerRotationTurret();
    public double getMapValue(Pose2d pose2d);
    public double regulateTa();

    public void shotWithRPM(double RPM);
    public void turnOnFuelToTurret(double speed);
    public void turnHoodFromSetpoint(double setpoint, double motor);
    public void turnToMapSetpoint(double setpoint);
    public void turnHoodFromSetpoint(double setpoint, double motor, BooleanSupplier tag);
    public void turnHoodFromSetpoint(double setpoint);
    public void setSetpoint(double setpoint);
    public void automatic();
    public void turnOfAllComponents();
    public void shootWithForce(double speed);
    public void updateInputs(TurretIOinputsAutoLogged turretIOinputsAutoLogged);

    public boolean inbuxing();
    public boolean turretOnSetpoint();
    public boolean isAutomatic();

    public Command normalTurretCommand(DoubleSupplier turret, DoubleSupplier hood, DoubleSupplier shooter, BooleanSupplier acel,
                                       BooleanSupplier spellFuels);
}
