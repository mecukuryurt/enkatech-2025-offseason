package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.wrist.Wrist;
import org.littletonrobotics.junction.Logger;

public class shoot2 extends SequentialCommandGroup {

  public shoot2(Arm arm, Wrist wrist) {

    addCommands(
        arm.goToPosition(Constants.Shooter2ArmPosition),
        wrist.goToPosition(Constants.Shooter2WristPosition));
    Logger.recordOutput("wristStatus", "right");
  }
}
