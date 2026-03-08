package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FuelIntake extends SubsystemBase {
  // TODO: Rename CANivores, tuner-project.json modules, etc. with default names to something clearer. - Elliot
  private final TalonFX intakeMotor = new TalonFX(30, "Default Name");
  private final TalonFX deployMotor = new TalonFX(31, "Default Name");

  private double IntakeMotorOutput = 0.8;
  private double DeployMotorOutput = 0.5;

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

  // TODO: Use energy-efficient method to coast / freewheel motors to a stop.
  // Do the same to FuelShooter if possible. Consult Mr. P for clarity. - Elliot
  public void intakeStop() {
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
