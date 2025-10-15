package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.wrist.Wrist;

public class shootL3 extends SequentialCommandGroup {

  public shootL3(Arm arm, Wrist wrist) {
    addCommands(
        arm.goToPosition(Constants.ShootL3ArmPos), wrist.goToPosition(Constants.ShootL3WristPos));
  }
}
