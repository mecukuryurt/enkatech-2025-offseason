package frc.robot.subsystems.arm;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ArmIOTalonFX implements ArmIO {
  MotionMagicVoltage m_req = new MotionMagicVoltage(-0.1);
  TalonFX motor;

  public ArmIOTalonFX(int CANID) {
    this.motor = new TalonFX(CANID, "canivore");
  }

  public void goToPosition(double pos) {
    motor.setControl(m_req.withPosition(pos));
  }
}
