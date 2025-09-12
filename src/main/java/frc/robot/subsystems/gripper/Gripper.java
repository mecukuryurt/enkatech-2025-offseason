package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
  GripperIO io;

  public Gripper(GripperIO io) {
    this.io = io;
  }

  public Gripper() {}

  public Command runAtVoltage(double v) {
    return new InstantCommand(
        () -> {
          io.runAtVoltage(v);
        });
  }

  @Override
  public void periodic() {}
}
