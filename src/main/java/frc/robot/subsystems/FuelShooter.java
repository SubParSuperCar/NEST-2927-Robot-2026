package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceConstants;

public class FuelShooter extends SubsystemBase {

  private final TalonFX kickerMotor1  = new TalonFX(
      DeviceConstants.FuelShooter.KICKER_MOTOR_1_ID,  DeviceConstants.CAN_BUS);
  private final TalonFX kickerMotor2  = new TalonFX(
      DeviceConstants.FuelShooter.KICKER_MOTOR_2_ID,  DeviceConstants.CAN_BUS);
  private final TalonFX shooterMotor1 = new TalonFX(
      DeviceConstants.FuelShooter.SHOOTER_MOTOR_1_ID, DeviceConstants.CAN_BUS);
  private final TalonFX shooterMotor2 = new TalonFX(
      DeviceConstants.FuelShooter.SHOOTER_MOTOR_2_ID, DeviceConstants.CAN_BUS);

  private final TalonFX[] kickerMotors  = { kickerMotor1,  kickerMotor2  };
  private final TalonFX[] shooterMotors = { shooterMotor1, shooterMotor2 };

  /** Kicker wheel duty-cycle scale (75%). */
  private static final double KICKER_OUTPUT  = 0.75;

  /** Shooter flywheel duty-cycle scale (50%). */
  private static final double SHOOTER_OUTPUT = 0.5;

  // Separate request instances to avoid aliasing between motor groups.
  private final DutyCycleOut kickerRequest  = new DutyCycleOut(0);
  private final DutyCycleOut shooterRequest = new DutyCycleOut(0);

  /**
   * Drives all kicker and shooter motors at a fraction of their output constants.
   *
   * @param scale Multiplier in [0, 1]: 1 = full power, 0 = stopped.
   */
  private void setOutputAll(double scale) {
    for (TalonFX m : kickerMotors)  m.setControl(kickerRequest.withOutput(scale * KICKER_OUTPUT));
    for (TalonFX m : shooterMotors) m.setControl(shooterRequest.withOutput(scale * SHOOTER_OUTPUT));
  }

  public void run()  { setOutputAll(1.0); }
  public void stop() { setOutputAll(0.0); }
}
