// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class GripperIOTalonFX implements GripperIO {
  VoltageOut voltageOut = new VoltageOut(0);
  TalonFX motor;

  public GripperIOTalonFX(int CANID) {
    this.motor = new TalonFX(CANID);
  }

  public void runAtVoltage(double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }
}
