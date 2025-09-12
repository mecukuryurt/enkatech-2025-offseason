package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  ArmIO io;

  public Arm(ArmIO io) {
    this.io = io;
  }

  public Arm() {}

  public Command goToPosition(double pos) {
    return new InstantCommand(
        () -> {
          io.goToPosition(pos);
        });
  }

  @Override
  public void periodic() {}
}
