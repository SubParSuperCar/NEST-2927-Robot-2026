package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FuelShooter extends SubsystemBase {
  // TODO: See comments in FuelIntake. - Elliot
  private final TalonFX kickerMotor1 = new TalonFX(29, "Default Name");
  private final TalonFX kickerMotor2 = new TalonFX(28, "Default Name");
  private final TalonFX shooterMotor1 = new TalonFX(3, "Default Name");
  private final TalonFX shooterMotor2 = new TalonFX(26, "Default Name");
  
  private double KickerMotorOutput = 0.8;
  private double ShooterMotorOutput = 0.6;

  private final TalonFX[] kickerMotors = { kickerMotor1, kickerMotor2, };
  private final TalonFX[] shooterMotors = { shooterMotor1, shooterMotor2 };
  
  private final DutyCycleOut motorRequest = new DutyCycleOut(0);

  public FuelShooter() {
  }

  private void setOutputAll(double output) {
    var request1 = motorRequest.withOutput(output * KickerMotorPower);
    var request2 = motorRequest.withOutput(output * ShooterMotorPower);
    
    for (TalonFX motor : kickerMotors) {
      motor.setControl(request1);
    }
    
    for (TalonFX motor : shooterMotors) {
      motor.setControl(request2);
    }
  }

  public void run() {
    setOutputAll(1);
  }

  public void stop() {
    setOutputAll(0);
  }
}
