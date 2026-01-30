package frc.robot.subsystems.mechanism;

import frc.robot.subsystems.mechanism.SuperStructure.Actions;


import edu.wpi.first.wpilibj2.command.Command;

public interface SuperStructureIO {
  public Command setAction(Actions action);
  public Command setActionSim(Actions action);

  public Actions getAction();
}
