package frc.robot.subsystems.gripper;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class GripperIOTalonFX implements GripperIO {
  VoltageOut voltageOut = new VoltageOut(0);
  TalonFX motor;

  public GripperIOTalonFX(int CANID) {
    this.motor = new TalonFX(CANID, "canivore");
  }

  public void runAtVoltage(double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }
}
