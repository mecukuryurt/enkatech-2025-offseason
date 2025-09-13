package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.wrist.Wrist;

public class idle extends SequentialCommandGroup {
  private double wristPos = 0.4;
  private double armPos = -0.174;

  public idle(Arm arm, Wrist wrist) {
    addCommands(arm.goToPosition(armPos), wrist.goToPosition(wristPos));
  }
}
