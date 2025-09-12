package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  ShooterIO io;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public Shooter() {}

  public Command runAtVoltage(double v) {
    return new InstantCommand(
        () -> {
          io.runAtVoltage(v);
        });
  }

  @Override
  public void periodic() {}
}
