package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.SharedValues;
import org.littletonrobotics.junction.Logger;

public class shoot2 extends SequentialCommandGroup {
  private double wristPos = -8.775;
  private double armPos = -0.31;

  private static SharedValues sharedValues;

  public shoot2(Arm arm, Wrist wrist) {
    sharedValues.shooterStatus = "left";
    addCommands(arm.goToPosition(armPos), wrist.goToPosition(wristPos));
    Logger.recordOutput("wristStatus", "right");
  }
}
