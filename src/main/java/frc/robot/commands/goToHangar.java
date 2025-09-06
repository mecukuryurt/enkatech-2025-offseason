// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.wrist.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class goToHangar extends Command {
  /** Creates a new goToHangar. */
  private static final double WRIST_POS = 0;

  private static final double ARM_POS = 0;
  private static Wrist wrist_subsystem;
  private static Arm arm_subsystem;

  public goToHangar(Wrist wrist, Arm arm) {
    wrist_subsystem = wrist;
    arm_subsystem = arm;
    addRequirements(wrist_subsystem, arm_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist_subsystem.goToPosition(WRIST_POS);
    arm_subsystem.goToPosition(ARM_POS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
