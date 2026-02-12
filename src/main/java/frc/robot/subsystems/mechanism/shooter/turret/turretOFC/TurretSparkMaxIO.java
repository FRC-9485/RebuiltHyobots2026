// package frc.robot.subsystems.mechanism.shooter.turret.turretOFC;

// import static edu.wpi.first.units.Units.Amps;
// import static edu.wpi.first.units.Units.RadiansPerSecond;
// import static edu.wpi.first.units.Units.Volts;

// import com.ctre.phoenix6.controls.NeutralOut;

// import edu.wpi.first.units.measure.Voltage;

// import frc.frc_java9485.constants.mechanisms.TurretConsts;
// import frc.frc_java9485.motors.spark.SparkFlexMotor;
// import frc.frc_java9485.motors.spark.SparkMaxMotor;

// public class TurretSparkMaxIO implements TurretIO {
//     private final SparkFlexMotor leftMotor;
//     private final SparkFlexMotor rightMotor;
//     private final SparkMaxMotor fuelToTurret;
//     private final SparkMaxMotor turnTurret;

//     private final double turnPosition;
//     private final double turnVelocity;
//     private final double turnCurrent;

//     private final double leftShootSpeed;
//     private final double rightShootSpeed;

//     private final double leftShootCurrent;
//     private final double rightShootCurrent;

//     private final double rightShootVoltage;
//     private final double leftShootVoltage;

//     private final Voltage turnTurretVoltage;
//     private final Voltage shooterVoltage;

//     private final NeutralOut neutralOut = new NeutralOut();

//     public TurretSparkMaxIO() {
//         leftMotor = new SparkFlexMotor(TurretConsts.LEFT_SHOOTER, "left shooter");
//         rightMotor = new SparkFlexMotor(TurretConsts.RIGHT_SHOOTER, "right shooter");
//         fuelToTurret = new SparkMaxMotor(TurretConsts.FUEL_TO_TURRET, "CATCH FUELS TO TURRET");
//         turnTurret = new SparkMaxMotor(TurretConsts.TURN_TURRET, "turn turret");

//         rightMotor.followMotor(TurretConsts.RIGHT_SHOOTER);
//         leftMotor.setInvert();

//         rightShootSpeed = rightMotor.getRate();
//         rightShootVoltage = rightMotor.getVoltage();

//         turnTurretVoltage = Volts.of(turnTurret.getVoltage());
//         shooterVoltage = Volts.of(leftMotor.getVoltage());

//         leftShootSpeed = leftMotor.getRate();
//         leftShootVoltage = leftMotor.getVoltage();

//         leftShootCurrent = leftMotor.getCurrent();
//         rightShootCurrent = rightMotor.getCurrent();

//         turnCurrent = turnTurret.getCurrent();
//         turnPosition = turnTurret.getPosition();
//         turnVelocity = turnTurret.getRate();
//     }

//     @Override
//     public void updateInputs(TurretIOInputs inputs) {
//         inputs.flywheelCurrent = Amps.of(leftShootCurrent + rightShootCurrent);
//         inputs.flywheelSpeed = RadiansPerSecond.of(leftShootSpeed);
//         inputs.turnCurrent = Amps.of(turnCurrent);
//         inputs.turnAppliedVolts = turnTurretVoltage;
//         inputs.turnVelocity = RadiansPerSecond.of(turnVelocity);
//         inputs.turnPosition = turnPosition;
//     }

//     public void setTurnOutput(Voltage out) {
//       turnTurret.setVoltage(out);
//     }

//     public void setHoodOutput(Voltage out) {

//     }

//     public void setFlywheelOutput(Voltage out) {

//     }

//     public void setShootOutput(Voltage out) {
//         leftMotor.setVoltage(out);
//     }

//     public void stopTurn() {
//         turnTurret.setVoltage(0);
//     }

//     public void stopHood() {
//     }

//     public void stopFlywheel() {
//         leftMotor.setVoltage(0);
//     }

//     public void stopShoot() {
//         leftMotor.setVoltage(0);
//     }
// }
