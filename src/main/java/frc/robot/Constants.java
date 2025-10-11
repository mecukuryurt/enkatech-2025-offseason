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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  };

  public static final Pose2d initialPose = new Pose2d(3, 3, new Rotation2d(0));
  // Motor CANID Constants
  public static int GripperCANID = 35;
  public static int ShooterCANID = 31;
  public static int ArmCANID = 32;
  public static int WristCANID = 8;
  public static int PigeonCANID = 1;

  // Voltage
  public static double ShooterV = -2;
  public static double GripperInTakeV = 4;
  public static double GripperBallV = -4;

  // Motor Positions
  public static double ArmOffset = 0;
  public static double WristOffset = 0;

  public static double StartArmPosition = -0.005;
  public static double StartWristPosition = 0 + WristOffset;

  public static double ShootArmPosition = -0.306 + ArmOffset;
  public static double ShootWristPosition = -12.4 + WristOffset;

  public static double LeftShootWristPosition = -35.6 + WristOffset;

  public static double IdleArmPos = -0.145 + ArmOffset;
  public static double IdleWristPos = -22.4 + WristOffset;

  public static double HangarArmPos = -0.064 + ArmOffset;
  public static double HangarWristPos = -22.4 + WristOffset;
  public static double L1ArmPos = -0.362 + ArmOffset;
  public static double L1WristPos = -21.92 + WristOffset;
}
