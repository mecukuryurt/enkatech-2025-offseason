package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.wrist.Wrist;

public class goToHangar extends SequentialCommandGroup {
  private double wristPos = -24;
  private double armPos = -0.276;

  public goToHangar(Arm arm, Wrist wrist) {
    addCommands(arm.goToPosition(armPos), wrist.goToPosition(wristPos));
  }
}
