package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.wrist.Wrist;

public class shoot2 extends SequentialCommandGroup {
  private double wristPos = -8.775;
  private double armPos = -0.31;

  public shoot2(Arm arm, Wrist wrist) {
    addCommands(arm.goToPosition(armPos), wrist.goToPosition(wristPos));
  }
}
