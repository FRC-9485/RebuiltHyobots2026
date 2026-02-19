package frc.frc_java9485.utils;

import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltHub;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltOutpost;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.frc_java9485.constants.FieldConsts;

public class Simulation {
  private static Simulation m_instance;

  private final Translation2d[] blueDepositFuelStartPoses =
      new Translation2d[] {
        new Translation2d(0.1, 5.6), new Translation2d(0.1, 5.75), new Translation2d(0.1, 5.9),
        new Translation2d(0.1, 6.05), new Translation2d(0.1, 6.20), new Translation2d(0.1, 6.35),
        new Translation2d(0.254, 5.6), new Translation2d(0.254, 5.75), new Translation2d(0.254, 5.9),
        new Translation2d(0.254, 6.05), new Translation2d(0.254, 6.20),new Translation2d(0.512, 6.35),
        new Translation2d(0.254, 6.35), new Translation2d(0.408, 5.6), new Translation2d(0.408, 5.75),
        new Translation2d(0.408, 5.9), new Translation2d(0.408, 6.05), new Translation2d(0.408, 6.20),
        new Translation2d(0.408, 6.35), new Translation2d(0.512, 5.6), new Translation2d(0.512, 5.75),
        new Translation2d(0.512, 5.9), new Translation2d(0.512, 6.05), new Translation2d(0.512, 6.20),
      };


  private final boolean generateFuels = true;

  private final RebuiltHub redHub;
  private final RebuiltHub blueHub;
  private final SimulatedArena arena;
  private final RebuiltOutpost blueOutpost;
  private final RebuiltOutpost redOutpost;
  private final Arena2026Rebuilt rebuiltArena;

  public static Simulation getInstance() {
    if (m_instance == null) {
      m_instance = new Simulation();
    }

    return m_instance;
  }

  private Simulation() {

    rebuiltArena = new Arena2026Rebuilt(false);

    redHub = new RebuiltHub(rebuiltArena, false);
    blueHub = new RebuiltHub(rebuiltArena, true);

    redOutpost = new RebuiltOutpost(rebuiltArena, false);
    blueOutpost = new RebuiltOutpost(rebuiltArena, true);

    SimulatedArena.overrideInstance(rebuiltArena);

    arena = SimulatedArena.getInstance();

    if (generateFuels) {
      // Blue Deposit Fuel
      for (Translation2d fuelPose : blueDepositFuelStartPoses) {
        arena.addGamePiece(new RebuiltFuelOnField(fuelPose));
      }
      // // Red Deposit Fuel
      // for (Translation2d fuelPose : blueDepositFuelStartPoses) {
      //   arena.addGamePiece(new RebuiltFuelOnField(flipFuelToRed(fuelPose)));
      // }

      // generateMiddleFuels();
    }
  }

  public void updateArena() {
    arena.simulationPeriodic();
    Pose3d[] fuelPoses = arena.getGamePiecesArrayByType("Fuel");
    Logger.recordOutput("Field Simulation/Fuel poses", fuelPoses);
  }

  private Translation2d flipFuelToRed(Translation2d translation) {
    translation = AllianceFlip.flipTranslation2dToRed(translation);
    return new Translation2d(translation.getX(), (FieldConsts.FIELD_WIDTH_METERS - 0.122) - translation.getY());
  }

  private void generateMiddleFuels() {
    Translation2d centerLine =
        new Translation2d(
            FieldConsts.FIELD_CENTER_POSE.getX(), FieldConsts.FIELD_CENTER_POSE.getY());

    Translation2d startTopLeft =
        new Translation2d(
            (centerLine.getX() - 0.7935), (FieldConsts.FIELD_WIDTH_METERS / 2) + 0.0375);

    int rows = 15;
    int columns = 12;

    for (int i = 0; i < 2; i++) {
      for (int x = 0; x < columns; x++) {
        for (int y = 0; y < rows; y++) {
          Translation2d position =
              i == 1
                  ? new Translation2d(
                      startTopLeft.getX() + (x * FieldConsts.FUEL_SPACING),
                      startTopLeft.getY() + (y * FieldConsts.FUEL_SPACING))
                  : new Translation2d(
                      startTopLeft.getX()
                          + ((x * FieldConsts.FUEL_SPACING) - FieldConsts.FUEL_SPACING / 2),
                      startTopLeft.getY() + (y * FieldConsts.FUEL_SPACING));

          arena.addGamePiece(new RebuiltFuelOnField(i == 1 ? position : flipFuelToRed(position)));
        }
      }
    }
  }
}
