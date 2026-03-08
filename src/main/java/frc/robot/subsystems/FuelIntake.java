package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FuelIntake extends SubsystemBase {
  private final TalonFX intakeMotor = new TalonFX(30);
  private final TalonFX deployMotor = new TalonFX(31);

  private double IntakeMotorOutput = 1;
  private double DeployMotorOutput = 1 / 3;

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

  public void stop() {
    setOutput(intakeMotor, 0);
  }

  public void deployExtend() {
    setOutput(deployMotor, DeployMotorOutput);
  }

  public void deployRetract() {
    setOutput(deployMotor, -DeployMotorOutput);
  }

  public void deployStop() {
    setOutput(deployMotor, 0);
  }
}
