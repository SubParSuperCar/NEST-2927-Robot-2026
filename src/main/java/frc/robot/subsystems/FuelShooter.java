package frc.robot.subsystems;

import frc.robot.generated.TunerConstants;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FuelShooter extends SubsystemBase {
  private final TalonFX kickerMotor1 = new TalonFX(29, TunerConstants.kCANBus);
  private final TalonFX kickerMotor2 = new TalonFX(28, TunerConstants.kCANBus);
  private final TalonFX shooterMotor1 = new TalonFX(3, TunerConstants.kCANBus);
  private final TalonFX shooterMotor2 = new TalonFX(26, TunerConstants.kCANBus);

  private double KickerMotorOutput = 1;
  private double ShooterMotorOutput = 0.625;

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

  public void move() {

  }
}
