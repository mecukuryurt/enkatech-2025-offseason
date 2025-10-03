// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class approachToReef extends SequentialCommandGroup {
  public static void logFiducial() {
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight");
    for (RawFiducial fiducial : fiducials) {
      int id = fiducial.id;
      double txnc = fiducial.txnc;
      double tync = fiducial.tync;
      double ta = fiducial.ta;
      double distToCamera = fiducial.distToCamera;
      double distToRobot = fiducial.distToRobot;
      double ambiguity = fiducial.ambiguity;

      if (true) {
        new PrintCommand("guys look i found a cat").execute();
        Logger.recordOutput("patatesx", txnc);
        Logger.recordOutput("patatesid", id);
        Logger.recordOutput("patatesdist", distToCamera);
        Logger.recordOutput("patatesa", ta);
      }
      break;
    }
  }

  /** Creates a new approachToReef. */
  public approachToReef(Drive drive) {
    Pose3d pose = LimelightHelpers.getTargetPose3d_CameraSpace("limelight");

    Logger.recordOutput("begonya", pose);

    PIDController pidX = new PIDController(0.6, 0, 0);
    PIDController pidY = new PIDController(0.1, 0, 0);
    PIDController pidR = new PIDController(0.06, 0, 0);

    double forwardMovement = 0; // pose.getY();
    double sideMovement = 0; // pose.getX();
    double rotationMovement = pidR.calculate(pose.getRotation().getAngle());

    Logger.recordOutput("papatya", rotationMovement);

    addCommands(
        DriveCommands.joystickDrive(
            drive, () -> forwardMovement, () -> sideMovement, () -> rotationMovement));
  }
}
