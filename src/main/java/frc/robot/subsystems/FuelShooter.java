package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FuelShooter extends SubsystemBase {
  private final TalonFX kickerMotor1 = new TalonFX(29);
  private final TalonFX kickerMotor2 = new TalonFX(28);
  private final TalonFX shooterMotor1 = new TalonFX(3);
  private final TalonFX shooterMotor2 = new TalonFX(26);

  private double ShooterMotorPower = 1;

  private final TalonFX[] motors = { kickerMotor1, kickerMotor2, shooterMotor1, shooterMotor2 };
  private final DutyCycleOut motorRequest = new DutyCycleOut(0);

  public FuelShooter() {
  }

  private void setOutputAll(double output) {
    var request = motorRequest.withOutput(output);

    for (TalonFX motor : motors) {
      motor.setControl(request);
    }
  }

  public void run() {
    setOutputAll(ShooterMotorPower);
  }

  public void stop() {
    setOutputAll(0);
  }
}
