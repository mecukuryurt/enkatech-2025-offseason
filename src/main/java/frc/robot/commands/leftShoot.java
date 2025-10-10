package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.wrist.Wrist;

public class leftShoot extends SequentialCommandGroup {

  public leftShoot(Arm arm, Wrist wrist) {

    addCommands(
        arm.goToPosition(Constants.ShootArmPosition),
        wrist.goToPosition(Constants.LeftShootWristPosition));
  }
}
