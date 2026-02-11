package frc.frc_java9485.constants.mechanisms;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import frc.frc_java9485.utils.TunableControls.ControlConstants;
import frc.frc_java9485.utils.TunableControls.TunableControlConstants;

public class TurretConsts {

  public static final int RIGHT_SHOOTER = 11;
  public static final int LEFT_SHOOTER = 12;
  public static final int TURN_TURRET = 13;
  public static final int FUEL_TO_TURRET = 14;

  public static final int LOOKAHEAD_ITERATIONS = 3;
  public static final Distance DISTANCE_ABOVE_FUNNEL = Inches.of(20); // how high to clear the funnel

  public static final Transform3d ROBOT_TO_TURRET_TRANSFORM =
                new Transform3d(new Translation3d(Inches.zero(), Inches.of(8), Inches.of(17.5)), Rotation3d.kZero);

  public static final ControlConstants TURRET_CONSTANTS = new ControlConstants()
  .withProfile(10, 4)//testes
  .withPID(0.01, 0, 0);

  public static final TunableControlConstants TURRET_TUNABLE = new TunableControlConstants("shooter tunableConstants", TURRET_CONSTANTS);
}
