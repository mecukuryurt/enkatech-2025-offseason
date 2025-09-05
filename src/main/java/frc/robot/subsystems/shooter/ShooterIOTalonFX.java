// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class ShooterIOTalonFX implements ShooterIO {
  TalonFX motor;
  VoltageOut voltageOut = new VoltageOut(0);

  public ShooterIOTalonFX(int CANID) {
    this.motor = new TalonFX(CANID, "canivore");
  }

  public void runAtVoltage(double v) {
    motor.setControl(voltageOut.withOutput(v));
  }
}
