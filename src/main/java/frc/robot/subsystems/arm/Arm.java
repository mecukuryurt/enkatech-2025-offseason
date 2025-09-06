// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new Shooter. */
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
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
