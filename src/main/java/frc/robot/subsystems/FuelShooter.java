package frc.robot.subsystems;

import frc.robot.generated.TunerConstants;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("FieldCanBeLocal")
public class FuelShooter extends SubsystemBase {
  private final TalonFX kickerMotor1 = new TalonFX(29, TunerConstants.kCANBus);
  private final TalonFX kickerMotor2 = new TalonFX(28, TunerConstants.kCANBus);
  private final TalonFX shooterMotor1 = new TalonFX(3, TunerConstants.kCANBus);
  private final TalonFX shooterMotor2 = new TalonFX(26, TunerConstants.kCANBus);

  private final double ShooterMotorOutputDefault = 11f / 16f;
  private final double ShooterMotorOutputMin = 5f / 8f;
  private final double ShooterMotorOutputMax = 7f / 8f;
  private final double ShooterMotorOutputIncrement = 1f / 16f;

  private final double KickerMotorOutput = 5f / 6f;
  private double ShooterMotorOutput = ShooterMotorOutputDefault;

  private final TalonFX[] kickerMotors = { kickerMotor1, kickerMotor2, };
  private final TalonFX[] shooterMotors = { shooterMotor1, shooterMotor2 };

  private final DutyCycleOut motorRequest = new DutyCycleOut(0);

  private void setOutputAll(double output) {
    var kickerRequest = motorRequest.withOutput(output * KickerMotorOutput);
    var shooterRequest = motorRequest.withOutput(output * ShooterMotorOutput);

    for (TalonFX motor : kickerMotors) {
      motor.setControl(kickerRequest);
    }

    for (TalonFX motor : shooterMotors) {
      motor.setControl(shooterRequest);
    }
  }

  public void run() {
    setOutputAll(1);
  }

  public void stop() {
    setOutputAll(0);
  }

  public void increaseSpeed() {
    ShooterMotorOutput = Math.min(
        ShooterMotorOutputMax,
        ShooterMotorOutput + ShooterMotorOutputIncrement);
  }

  public void decreaseSpeed() {
    ShooterMotorOutput = Math.max(
        ShooterMotorOutputMin,
        ShooterMotorOutput - ShooterMotorOutputIncrement);
  }

  public void resetSpeed() {
    ShooterMotorOutput = ShooterMotorOutputDefault;
  }
}
