package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.wrist.Wrist;
import org.littletonrobotics.junction.Logger;

public class shoot extends SequentialCommandGroup {

  public shoot(Arm arm, Wrist wrist) {

    addCommands(
        arm.goToPosition(Constants.Shooter1ArmPosition),
        wrist.goToPosition(Constants.Shooter1WristPosition));
    Logger.recordOutput("wristStatus", "left");
  }
}
