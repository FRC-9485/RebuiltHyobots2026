package frc.frc_java9485.constants.mechanisms;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.frc_java9485.utils.TunableControls.ControlConstants;
import frc.frc_java9485.utils.TunableControls.TunableControlConstants;
import frc.robot.subsystems.mechanism.shooter.turret.turretOFC.TurretCalculator.ShotData;

public class TurretConsts {

  public static final int RIGHT_SHOOTER = 11;
  public static final int LEFT_SHOOTER = 12;
  public static final int TURN_TURRET = 13;
  public static final int FUEL_TO_TURRET = 14;

  public static final int LOOKAHEAD_ITERATIONS = 3;
  public static final Distance FLY_WHEEL_RADIUS = Inches.of(2);
  public static final Distance DISTANCE_ABOVE_FUNNEL = Inches.of(20); // how high to clear the funnel

  public static final Transform3d ROBOT_TO_TURRET_TRANSFORM =
                new Transform3d(new Translation3d(Inches.zero(), Inches.of(8), Inches.of(17.5)), Rotation3d.kZero);

  // public static final ControlConstants TURRET_CONSTANTS = new ControlConstants()
  // .withProfile(100, 80)//testes
  // .withPID(0.4, 0, 0.2);

  public static final Angle MAX_TURN_ANGLE = Rotations.of(270);
  public static final Angle MIN_TURN_ANGLE = Rotations.of(0);

  public static final InterpolatingTreeMap<Double, ShotData> SHOT_MAP = new InterpolatingTreeMap<>(
    InverseInterpolator.forDouble(), ShotData::interpolate);

    public static final InterpolatingDoubleTreeMap TOF_MAP = new InterpolatingDoubleTreeMap();

      static {
          SHOT_MAP.put(5.34, new ShotData(RPM.of(2850), Degrees.of(27)));
          TOF_MAP.put(5.34, 1.30);

          SHOT_MAP.put(4.90, new ShotData(RPM.of(2810), Degrees.of(26)));
          TOF_MAP.put(4.90, 1.42);

          SHOT_MAP.put(4.44, new ShotData(RPM.of(2800), Degrees.of(25.5)));
          TOF_MAP.put(4.44, 1.34);

          SHOT_MAP.put(4.05, new ShotData(RPM.of(2790), Degrees.of(25)));
          TOF_MAP.put(4.05, 1.36);

          SHOT_MAP.put(3.74, new ShotData(RPM.of(2720), Degrees.of(24)));
          TOF_MAP.put(3.74, 1.21);

          SHOT_MAP.put(3.42, new ShotData(RPM.of(2670), Degrees.of(23)));
          TOF_MAP.put(3.42, 1.40);

          SHOT_MAP.put(3.06, new ShotData(RPM.of(2600), Degrees.of(22)));
          TOF_MAP.put(3.06, 1.38);

          SHOT_MAP.put(2.73, new ShotData(RPM.of(2500), Degrees.of(20.5)));
          TOF_MAP.put(2.73, 1.34);

          SHOT_MAP.put(2.45, new ShotData(RPM.of(2450), Degrees.of(19.5)));
          TOF_MAP.put(2.45, 1.28);

          SHOT_MAP.put(2.14, new ShotData(RPM.of(2400), Degrees.of(18)));
          TOF_MAP.put(2.14, 1.31);

          SHOT_MAP.put(1.86, new ShotData(RPM.of(2350), Degrees.of(17)));
          TOF_MAP.put(1.86, 1.24);

          SHOT_MAP.put(1.55, new ShotData(RPM.of(2275), Degrees.of(15)));
          TOF_MAP.put(1.55, 1.23);
      }

  public static final ControlConstants TURRET_CONSTANTS = new ControlConstants()
  .withProfile(1000, 700)//testes
  .withPID(6, 0.0, 0.0);

  public static final int SHOOTER_CURRENT_LIMIT = 40;
  public static final int TURN_TURRET_CURRENT_LIMIT = 20;
  public static final int FUEL_TO_TURRET_CURRENT_LIMIT = 30;

  public static final TunableControlConstants TURRET_TUNABLE = new TunableControlConstants("shooter tunableConstants", TURRET_CONSTANTS);
}
