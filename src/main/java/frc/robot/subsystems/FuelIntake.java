package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FuelIntake extends SubsystemBase {
	private final TalonFX intakeMotor = new TalonFX(30);
	private final TalonFX deployMotor = new TalonFX(31);

	private final DutyCycleOut motorRequest = new DutyCycleOut(0);

	private void setOutput(TalonFX motor, double output) {
		motor.setControl(motorRequest.withOutput(output));
	}

	public void intakeIn() {
		setOutput(intakeMotor, 1);
	}

	public void intakeOut() {
		setOutput(intakeMotor, -1);
	}

	public void stop() {
		setOutput(intakeMotor, 0);
	}

	public void deployOn() {
		setOutput(deployMotor, 1);
	}

	public void deployOff() {
		setOutput(deployMotor, 0);
	}
}
