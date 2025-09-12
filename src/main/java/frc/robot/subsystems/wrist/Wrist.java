package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  WristIO io;

  public Wrist(WristIO io) {
    this.io = io;
  }

  public Wrist() {}

  public Command goToPosition(double pos) {
    return new InstantCommand(
        () -> {
          io.goToPosition(pos);
        });
  }

  @Override
  public void periodic() {}
}
