package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FuelShooter extends SubsystemBase {

    // ── Placeholder CAN IDs ─────────────────────────────────────────────────
    // TODO: Replace with actual CAN IDs from Tuner X
    private static final int LEFT_MOTOR_1_ID = 20;
    private static final int LEFT_MOTOR_2_ID = 21;
    private static final int RIGHT_MOTOR_1_ID = 22;
    private static final int RIGHT_MOTOR_2_ID = 23;

    // ── Motors ───────────────────────────────────────────────────────────────
    private final TalonFX leftMotor1 = new TalonFX(LEFT_MOTOR_1_ID);
    private final TalonFX leftMotor2 = new TalonFX(LEFT_MOTOR_2_ID);
    private final TalonFX rightMotor1 = new TalonFX(RIGHT_MOTOR_1_ID);
    private final TalonFX rightMotor2 = new TalonFX(RIGHT_MOTOR_2_ID);

    // ── Control request (open-loop duty cycle) ───────────────────────────────
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    // ── Shooter speeds ───────────────────────────────────────────────────────
    // TODO: Tune these values during testing
    private static final double LEFT_PAIR_SPEED = 0.75;
    private static final double RIGHT_PAIR_SPEED = 0.75;

    public FuelShooter() {
        // TODO: Add motor config (inversion, current limits, etc.) as needed
    }

    // ── Left trigger pair (motors 1 & 2) ────────────────────────────────────
    public void runLeftPair() {
        leftMotor1.setControl(dutyCycleRequest.withOutput(LEFT_PAIR_SPEED));
        leftMotor2.setControl(dutyCycleRequest.withOutput(LEFT_PAIR_SPEED));
    }

    public void stopLeftPair() {
        leftMotor1.setControl(dutyCycleRequest.withOutput(0));
        leftMotor2.setControl(dutyCycleRequest.withOutput(0));
    }

    // ── Right trigger pair (motors 3 & 4) ───────────────────────────────────
    public void runRightPair() {
        rightMotor1.setControl(dutyCycleRequest.withOutput(RIGHT_PAIR_SPEED));
        rightMotor2.setControl(dutyCycleRequest.withOutput(RIGHT_PAIR_SPEED));
    }

    public void stopRightPair() {
        rightMotor1.setControl(dutyCycleRequest.withOutput(0));
        rightMotor2.setControl(dutyCycleRequest.withOutput(0));
    }

    @Override
    public void periodic() {
        // SmartDashboard logging can go here later
    }
}
