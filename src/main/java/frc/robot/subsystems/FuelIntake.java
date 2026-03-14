package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceConstants;

public class FuelIntake extends SubsystemBase {

  private final TalonFX intakeMotor = new TalonFX(
      DeviceConstants.FuelIntake.INTAKE_MOTOR_ID, DeviceConstants.CAN_BUS);
  private final TalonFX deployMotor = new TalonFX(
      DeviceConstants.FuelIntake.DEPLOY_MOTOR_ID, DeviceConstants.CAN_BUS);

  /** Intake roller duty-cycle output (87.5%). Fixed from integer-division bug. */
  private static final double INTAKE_OUTPUT = 7.0 / 8.0;

  /** Deploy arm duty-cycle output (33.3%). Fixed from integer-division bug. */
  private static final double DEPLOY_OUTPUT = 1.0 / 3.0;

  // Separate request instances to avoid aliasing between motors.
  private final DutyCycleOut intakeRequest = new DutyCycleOut(0);
  private final DutyCycleOut deployRequest = new DutyCycleOut(0);

  private void setIntake(double output) {
    intakeMotor.setControl(intakeRequest.withOutput(output));
  }

  private void setDeploy(double output) {
    deployMotor.setControl(deployRequest.withOutput(output));
  }

  // Intake roller

  public void intakeIn()   { setIntake( INTAKE_OUTPUT); }
  public void intakeOut()  { setIntake(-INTAKE_OUTPUT); }
  public void intakeStop() { setIntake(0); }

  // Deploy arm

  public void deployExtend()  { setDeploy( DEPLOY_OUTPUT); }
  public void deployRetract() { setDeploy(-DEPLOY_OUTPUT); }
  public void deployStop()    { setDeploy(0); }
}
