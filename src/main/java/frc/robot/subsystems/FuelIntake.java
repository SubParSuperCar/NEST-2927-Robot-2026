package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.generated.TunerConstants;

public class FuelIntake extends SubsystemBase {
  private final TalonFX intakeMotor = new TalonFX(30, TunerConstants.kCANBus);
  private final TalonFX deployMotor = new TalonFX(31, TunerConstants.kCANBus);

  private final double IntakeMotorOutput = 7f / 8f;
  @SuppressWarnings("FieldCanBeLocal")
  private final double DeployMotorOutput = 3f / 16f;

  private final DutyCycleOut motorRequest = new DutyCycleOut(0);

  private void setOutput(TalonFX motor, double output) {
    motor.setControl(motorRequest.withOutput(output));
  }

  public void intakeIn() {
    setOutput(intakeMotor, IntakeMotorOutput);
  }

  public void intakeOut() {
    setOutput(intakeMotor, -IntakeMotorOutput);
  }

  public void intakeStop() {
    setOutput(intakeMotor, 0);
  }

  public void deployExtend() {
    setOutput(deployMotor, DeployMotorOutput);
  }

  public void deployStop() {
    setOutput(deployMotor, 0);
  }
}
