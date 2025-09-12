package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

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
