// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class WristIOTalonFX implements WristIO {
  MotionMagicVoltage m_req = new MotionMagicVoltage(0);
  TalonFX motor;

  public WristIOTalonFX(int CANID) {
    this.motor = new TalonFX(CANID, "canivore");
  }

  public void goToPosition(double pos) {

    motor.setControl(m_req.withPosition(pos));
  }
}
