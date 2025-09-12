package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.wrist.Wrist;

public class shoot extends SequentialCommandGroup {
  private double wristPos = -35;
  private double armPos = -0.335;

  public shoot(Arm arm, Wrist wrist) {
    addCommands(arm.goToPosition(armPos), wrist.goToPosition(wristPos));
  }
}
