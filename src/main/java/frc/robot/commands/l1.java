package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.wrist.Wrist;

public class l1 extends SequentialCommandGroup {

  public l1(Arm arm, Wrist wrist) {

    addCommands(arm.goToPosition(Constants.L1ArmPos), wrist.goToPosition(Constants.L1WristPos));
  }
}
