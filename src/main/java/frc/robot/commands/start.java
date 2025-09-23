package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.wrist.Wrist;

public class start extends SequentialCommandGroup {
  public start(Arm arm, Wrist wrist) {
    addCommands(
        arm.goToPosition(Constants.StartArmPosition),
        wrist.goToPosition(Constants.StartWristPosition));
  }
}
