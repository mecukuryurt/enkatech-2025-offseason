// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoShootL3 extends SequentialCommandGroup {
  /** Creates a new NewScoreL4. */
  public autoShootL3(Drive drive, Arm arm, Wrist wrist, Shooter shooter, boolean isRight) {

    addCommands(
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                isRight ? new shootL3(arm, wrist) : new leftShootL3(arm, wrist),
                new approachToReef(drive)),
            new WaitCommand(0.5),
            new ConditionalCommand(
                new SequentialCommandGroup(
                    shooter.runAtVoltage(isRight ? Constants.ShooterL3V : -Constants.ShooterL3V),
                    new WaitCommand(0.6),
                    shooter.runAtVoltage(0)),
                new InstantCommand(() -> {}),
                () -> LimelightHelpers.getFiducialID("limelight") != -1)));
  }
}
