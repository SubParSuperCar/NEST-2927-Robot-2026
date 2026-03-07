package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FuelShooter extends SubsystemBase {
    private final TalonFX leftMotor1 = new TalonFX(99);
    private final TalonFX leftMotor2 = new TalonFX(99);
    private final TalonFX rightMotor1 = new TalonFX(99);
    private final TalonFX rightMotor2 = new TalonFX(99);

    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    private static final double LEFT_MOTOR_PAIR_OUTOUT = 1;
    private static final double RIGHT_MOTOR_PAIR_OUTPUT = 1;

    public FuelShooter() {
    }

    public void runLeftPair() {
        leftMotor1.setControl(dutyCycleRequest.withOutput(LEFT_MOTOR_PAIR_OUTOUT));
        leftMotor2.setControl(dutyCycleRequest.withOutput(LEFT_MOTOR_PAIR_OUTOUT));
    }

    public void stopLeftPair() {
        leftMotor1.setControl(dutyCycleRequest.withOutput(0));
        leftMotor2.setControl(dutyCycleRequest.withOutput(0));
    }

    public void runRightPair() {
        rightMotor1.setControl(dutyCycleRequest.withOutput(RIGHT_MOTOR_PAIR_OUTPUT));
        rightMotor2.setControl(dutyCycleRequest.withOutput(RIGHT_MOTOR_PAIR_OUTPUT));
    }

    public void stopRightPair() {
        rightMotor1.setControl(dutyCycleRequest.withOutput(0));
        rightMotor2.setControl(dutyCycleRequest.withOutput(0));
    }
}
