// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.wrist.Wrist;

public class shoot extends SequentialCommandGroup {
  public shoot(Arm arm, Wrist wrist) {
    addCommands(arm.goToPosition(-0.335), wrist.goToPosition(-35));
  }
}
