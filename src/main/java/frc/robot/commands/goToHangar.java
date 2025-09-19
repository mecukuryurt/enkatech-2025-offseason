package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.wrist.Wrist;

public class goToHangar extends SequentialCommandGroup {

  public goToHangar(Arm arm, Wrist wrist) {
    addCommands(
        arm.goToPosition(Constants.HangarArmPos), wrist.goToPosition(Constants.HangarWristPos));
  }
}
