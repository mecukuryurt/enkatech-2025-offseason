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

// import com.google.flatbuffers.Constants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.approachToReef;
import frc.robot.commands.goToHangar;
import frc.robot.commands.l1;
import frc.robot.commands.leftShoot;
import frc.robot.commands.shoot;
import frc.robot.commands.start;
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
  private SlewRateLimiter xLimiter = new SlewRateLimiter(4);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(4);

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

    registerNamedCommands();

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
    // drive.setPose(Constants.initialPose);
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

    // operator
    // .leftBumper()
    // .onTrue(shooter.runAtVoltage(Constants.ShooterV))
    // .onFalse(shooter.runAtVoltage(0));
    // operator.povUp().onTrue(new goToHangar(arm, wrist));
    // operator.povLeft().onTrue(new shoot(arm, wrist));
    // operator.povDown().onTrue(new idle(arm, wrist));
    // operator.povRight().onTrue(new start(arm, wrist));

    driver
        .rightBumper()
        .onTrue(gripper.runAtVoltage(Constants.GripperInTakeV))
        .onFalse(gripper.runAtVoltage(0));
    operator
        .rightBumper()
        .onTrue(shooter.runAtVoltage(Constants.ShooterV))
        .onFalse(shooter.runAtVoltage(0));
    operator
        .leftBumper()
        .onTrue(shooter.runAtVoltage(-Constants.ShooterV))
        .onFalse(shooter.runAtVoltage(0));

    operator.x().onTrue(new l1(arm, wrist));
    operator
        .y()
        .onTrue(gripper.runAtVoltage(Constants.GripperBallV))
        .onFalse(gripper.runAtVoltage(0));
    operator.povUp().onTrue(new goToHangar(arm, wrist));
    operator.povLeft().onTrue(new leftShoot(arm, wrist));
    operator.povDown().onTrue(new start(arm, wrist));
    operator.povRight().onTrue(new shoot(arm, wrist));
  }

  private void driveButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -yLimiter.calculate(driver.getLeftY()),
            () -> -xLimiter.calculate(driver.getLeftX()),
            () -> -driver.getRightX() * 0.6));

    // Lock to 0° when A button is held
    driver
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    // driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    driver
        .x()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    driver.b().whileTrue(new approachToReef(drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("volkswagen");
  }

  public void limelightPoseEstimatorMegaTag2() {
    // First, tell Limelight your robot's current orientation
    LimelightHelpers.setPipelineIndex("limelight", 0);
    LimelightHelpers.SetRobotOrientation(
        "limelight", drive.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    Logger.recordOutput("ll-val", LimelightHelpers.getTX("limelight"));

    // Get the pose estimate
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    // Add it to your pose estimator

    if (mt2 != null && mt2.tagCount != 0) {
      Logger.recordOutput("odometry-ll", mt2.pose);
      drive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds, VecBuilder.fill(.7, .7, 999999));
    }

    Logger.recordOutput("odometry", drive.getPose());
  }

  public void logFiducial() {
    approachToReef.logFiducial();
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("Intake", gripper.runAtVoltage(Constants.GripperInTakeV));
    NamedCommands.registerCommand("ShootRight", shooter.runAtVoltage(Constants.ShooterV));
    NamedCommands.registerCommand("ShootLeft", shooter.runAtVoltage(-Constants.ShooterV));
    NamedCommands.registerCommand("L1Position", new l1(arm, wrist));
    NamedCommands.registerCommand(
        "L1ShootAndAlgaeCommand", gripper.runAtVoltage(Constants.GripperBallV));
    NamedCommands.registerCommand("goToHangar", new goToHangar(arm, wrist));
    NamedCommands.registerCommand("goToShootRight", new shoot(arm, wrist));
    NamedCommands.registerCommand("goToShootLeft", new leftShoot(arm, wrist));
    NamedCommands.registerCommand("robotStartState", new start(arm, wrist));
  }

  // public double dist(){

  // }
  // ;
}
