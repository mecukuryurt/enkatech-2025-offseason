// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.drive.Drive;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.Logger;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class approachToReef extends SequentialCommandGroup {

  public static class Reef {
    Pose2d pose;
    int blueID;
    int redID;

    Reef(Pose2d pose, int blueID, int redID) {
      this.blueID = blueID;
      this.pose = pose;
      this.redID = redID;
    }

    int getID() {
      return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue
          ? this.blueID
          : this.redID;
    }
  }

  public static List<Reef> reefs =
      Arrays.asList(
          new Reef(new Pose2d(3.78, 2.83, new Rotation2d(Units.degreesToRadians(-120))), 17, 8),
          new Reef(new Pose2d(3.14, 4.02, new Rotation2d(Units.degreesToRadians(180))), 18, 7),
          new Reef(
              new Pose2d(3.81, 5.21, new Rotation2d(Units.degreesToRadians(120))), 19, 6), // 120
          new Reef(new Pose2d(5.21, 5.21, new Rotation2d(Units.degreesToRadians(60))), 20, 11),
          new Reef(new Pose2d(5.88, 4.02, new Rotation2d(Units.degreesToRadians(0))), 21, 10),
          new Reef(new Pose2d(5.16, 2.82, new Rotation2d(Units.degreesToRadians(-60))), 22, 9));

  public static Rotation2d getReefRotation(double id) {
    // System.out.println(id);
    for (Reef reef : reefs) {
      if (reef.redID == id || reef.blueID == id) {
        return reef.pose.getRotation();
      }
    }
    return new Rotation2d();
  }

  public static double getModuloRotation(double rawYaw) {
    double modified = (Math.abs(rawYaw) % (360)) * Math.signum(rawYaw);
    if (modified < -180) modified += 360;
    if (modified > 180) modified -= 360;
    return modified;
  }

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
      Pose3d target2robot = LimelightHelpers.getTargetPose3d_CameraSpace("limelight");
      Logger.recordOutput(
          "uzaklik",
          Math.sqrt(
              target2robot.getY() * target2robot.getY()
                  + target2robot.getZ() * target2robot.getZ()));
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

  public static class commonPIDandLimelightValues {}

  /** Creates a new approachToReef. */
  public approachToReef(Drive drive) {

    addCommands(
        new FunctionalCommand(
            () -> {},
            () -> {
              RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight");
              double distToCamera = 0, distToRobot, txnc = 0;
              double id = 0;
              for (RawFiducial fiducial : fiducials) {
                id = fiducial.id;
                txnc = fiducial.txnc;
                double tync = fiducial.tync;
                double ta = fiducial.ta;
                distToCamera = fiducial.distToCamera;
                distToRobot = fiducial.distToRobot;
                double ambiguity = fiducial.ambiguity;
                break;
              }
              // Pose2d pose = drive.getPose();
              Pose2d pose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").pose;
              // Pose3d pose = LimelightHelpers.getTargetPose3d_CameraSpace("limelight");
              // Logger.recordOutput("karanfil", LimelightHelpers.getTV("limelight"));

              // Logger.recordOutput("begonya", pose);

              PIDController pidX = new PIDController(0.04, 0, 0);
              PIDController pidY = new PIDController(2, 1, 0);
              PIDController pidR = new PIDController(0.06, 0, 0);

              pidR.setSetpoint(0);
              pidY.setSetpoint(0.55);
              pidX.setSetpoint(0.3);
              double forwardMovement = pidY.calculate(distToCamera); // pose.getY();
              double sideMovement = pidX.calculate(txnc); // pose.getX();
              double rotationMovement =
                  pidR.calculate(
                      getModuloRotation(
                          getReefRotation(id).getDegrees() - pose.getRotation().getDegrees()));

              Logger.recordOutput("kasimpati", sideMovement);

              drive.runVelocity(
                  new ChassisSpeeds(-forwardMovement, sideMovement, -rotationMovement));

              Logger.recordOutput("papatya", pose.getRotation().getDegrees());
            },
            (Boolean end) -> {
              drive.runVelocity(new ChassisSpeeds());
            },
            () -> {
              RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight");
              double distToCamera = 0, distToRobot;
              for (RawFiducial fiducial : fiducials) {
                int id = fiducial.id;
                double txnc = fiducial.txnc;
                double tync = fiducial.tync;
                double ta = fiducial.ta;
                distToCamera = fiducial.distToCamera;
                distToRobot = fiducial.distToRobot;
                double ambiguity = fiducial.ambiguity;
                break;
              }
              return (distToCamera < 0.65);
            },
            drive));
  }
}
