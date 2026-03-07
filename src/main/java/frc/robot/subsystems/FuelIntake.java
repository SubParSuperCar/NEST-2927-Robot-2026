package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FuelIntake extends SubsystemBase {
  private final TalonFX intakeMotor = new TalonFX(99);
  private final TalonFX deployMotor = new TalonFX(99);

  private final DutyCycleOut motorRequest = new DutyCycleOut(0);

  private static final double MOTOR_OUTPUT = 1;

  public void intakeIn() {
    intakeMotor.setControl(motorRequest.withOutput(MOTOR_OUTPUT));
  }

  public void intakeOut() {
    intakeMotor.setControl(motorRequest.withOutput(-MOTOR_OUTPUT));
  }

  public void stop() {
    intakeMotor.setControl(motorRequest.withOutput(0));
  }

  public void deployOn() {
    deployMotor.setControl(motorRequest.withOutput(MOTOR_OUTPUT));
  }

  public void deployOff() {
    deployMotor.setControl(motorRequest.withOutput(0));
  }
}
