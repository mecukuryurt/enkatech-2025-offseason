// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
  /** Creates a new Gripper. */
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
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
