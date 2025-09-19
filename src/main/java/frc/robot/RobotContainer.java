// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.goToHangar;
import frc.robot.commands.idle;
import frc.robot.commands.shoot;
import frc.robot.commands.shoot2;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperIOSim;
import frc.robot.subsystems.gripper.GripperIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristIOTalonFX;
import frc.robot.util.SharedValues;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  // Controllers
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Operational Motors:
  private final Gripper gripper;
  private final Shooter shooter;
  private final Arm arm;
  private final Wrist wrist;
  public boolean isWristInLeft = false;

  private static SharedValues sharedValues;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        gripper = new Gripper(new GripperIOTalonFX(Constants.GripperCANID));
        shooter = new Shooter(new ShooterIOTalonFX(Constants.ShooterCANID));
        arm = new Arm(new ArmIOTalonFX(Constants.ArmCANID));
        wrist = new Wrist(new WristIOTalonFX(Constants.WristCANID));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        gripper = new Gripper(new GripperIOSim());
        shooter = new Shooter(new ShooterIOSim());
        arm = new Arm(new ArmIOSim());
        wrist = new Wrist(new WristIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        gripper = new Gripper();
        shooter = new Shooter();
        arm = new Arm();
        wrist = new Wrist();
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driveButtonBindings();
    operatorButtonBindings();
  }

  private void operatorButtonBindings() {
    // operator.rightBumper().onTrue(gripper.runAtVoltage(4)).onFalse(gripper.runAtVoltage(0));
    // operator.leftBumper().onTrue(shooter.runAtVoltage(5)).onFalse(shooter.runAtVoltage(0));
    // operator.y().onTrue(new goToHangar(arm, wrist));
    // operator.b().onTrue(new shoot(arm, wrist));
    driver.rightBumper().onTrue(gripper.runAtVoltage(4)).onFalse(gripper.runAtVoltage(0));

    driver
        .leftBumper()
        .onTrue(shooter.runAtVoltage((sharedValues.shooterStatus == "left") ? 5 : -5))
        .onFalse(shooter.runAtVoltage(0));
    driver.povUp().onTrue(new goToHangar(arm, wrist));
    driver.povDown().onTrue(new shoot(arm, wrist));

    driver.povLeft().onTrue(new shoot2(arm, wrist));
    driver.povRight().onTrue(new idle(arm, wrist));
    driver.y().onTrue(shooter.runAtVoltage(-5)).onFalse(shooter.runAtVoltage(0));
  }

  private void driveButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    // Lock to 0° when A button is held
    driver
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    driver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void limelightPoseEstimatorMegaTag2() {
    // First, tell Limelight your robot's current orientation
    LimelightHelpers.setPipelineIndex("limelight", 0);
    LimelightHelpers.SetRobotOrientation(
        "limelight", drive.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    ;

    // Get the pose estimate
    LimelightHelpers.PoseEstimate limelightMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

    // Add it to your pose estimator

    if (LimelightHelpers.getFiducialID("limelight") != -1) {
      drive.poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
      drive.poseEstimator.addVisionMeasurement(
          limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
    }

    Logger.recordOutput("odometry", drive.getPose());
  }
}
