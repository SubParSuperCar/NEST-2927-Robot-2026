package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FuelShooter extends SubsystemBase {
  // TODO: See comment in FuelIntake. - Elliot
  private final TalonFX kickerMotor1 = new TalonFX(29, "Default Name");
  private final TalonFX kickerMotor2 = new TalonFX(28, "Default Name");
  private final TalonFX shooterMotor1 = new TalonFX(3, "Default Name");
  private final TalonFX shooterMotor2 = new TalonFX(26, "Default Name");

  private double ShooterMotorPower = 1;

  private final TalonFX[] motor1s = { kickerMotor1, shooterMotor1 };
  private final TalonFX[] motor2s = { kickerMotor2, shooterMotor2 };
  
  private final DutyCycleOut motorRequest = new DutyCycleOut(0);

  public FuelShooter() {
  }

  private void setOutputAll(double output) {
    var request1 = motorRequest.withOutput(output);
    var request2 = motorRequest.withOutput(-output);
    
    for (TalonFX motor : motor1s) {
      motor.setControl(request1);
    }
    
    for (TalonFX motor : motor2s) {
      motor.setControl(request2);
    }
  }

  public void run() {
    setOutputAll(ShooterMotorPower);
  }

  public void stop() {
    setOutputAll(0);
  }
}
