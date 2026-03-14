package frc.robot;

/**
 * Central registry of every physical device ID and sensor offset on the robot.
 * Hardware changes only need to be made here.
 *
 * Each entry pairs a human-readable NAME string (matching the label in Tuner X)
 * with its numeric ID, organized into nested static classes by subsystem.
 */
public final class DeviceConstants {

  private DeviceConstants() {}

  /** Shared CAN bus name for all swerve and mechanism devices. */
  public static final String CAN_BUS = "Default Name";

  // Drivetrain  - swerve modules and IMU

  public static final class Drivetrain {
    public static final String PIGEON_NAME          = "Pigeon2";
    public static final int    PIGEON_ID            = 50;

    public static final String FL_DRIVE_NAME        = "FL Drive";
    public static final int    FL_DRIVE_ID          = 7;
    public static final String FL_STEER_NAME        = "FL Steer";
    public static final int    FL_STEER_ID          = 8;
    public static final String FL_ENCODER_NAME      = "FL CANcoder";
    public static final int    FL_ENCODER_ID        = 14;
    /** CANcoder absolute offset in rotations (from Tuner X). */
    public static final double FL_ENCODER_OFFSET    = -0.146484375;

    public static final String FR_DRIVE_NAME        = "FR Drive";
    public static final int    FR_DRIVE_ID          = 5;
    public static final String FR_STEER_NAME        = "FR Steer";
    public static final int    FR_STEER_ID          = 6;
    public static final String FR_ENCODER_NAME      = "FR CANcoder";
    public static final int    FR_ENCODER_ID        = 13;
    public static final double FR_ENCODER_OFFSET    = -0.396240234375;

    public static final String BL_DRIVE_NAME        = "BL Drive";
    public static final int    BL_DRIVE_ID          = 1;
    public static final String BL_STEER_NAME        = "BL Steer";
    public static final int    BL_STEER_ID          = 2;
    public static final String BL_ENCODER_NAME      = "BL CANcoder";
    public static final int    BL_ENCODER_ID        = 10;
    public static final double BL_ENCODER_OFFSET    = 0.2470703125;

    public static final String BR_DRIVE_NAME        = "BR Drive";
    public static final int    BR_DRIVE_ID          = 25;
    public static final String BR_STEER_NAME        = "BR Steer";
    public static final int    BR_STEER_ID          = 4;
    public static final String BR_ENCODER_NAME      = "BR CANcoder";
    public static final int    BR_ENCODER_ID        = 11;
    public static final double BR_ENCODER_OFFSET    = -0.2275390625;
  }

  // Fuel Intake subsystem

  public static final class FuelIntake {
    public static final String INTAKE_MOTOR_NAME    = "Intake Motor";
    public static final int    INTAKE_MOTOR_ID      = 30;

    public static final String DEPLOY_MOTOR_NAME    = "Deploy Motor";
    public static final int    DEPLOY_MOTOR_ID      = 31;
  }

  // Fuel Shooter subsystem

  public static final class FuelShooter {
    public static final String KICKER_MOTOR_1_NAME  = "Kicker Motor 1";
    public static final int    KICKER_MOTOR_1_ID    = 29;

    public static final String KICKER_MOTOR_2_NAME  = "Kicker Motor 2";
    public static final int    KICKER_MOTOR_2_ID    = 28;

    public static final String SHOOTER_MOTOR_1_NAME = "Shooter Motor 1";
    public static final int    SHOOTER_MOTOR_1_ID   = 3;

    public static final String SHOOTER_MOTOR_2_NAME = "Shooter Motor 2";
    public static final int    SHOOTER_MOTOR_2_ID   = 26;
  }

  // Driver controller

  public static final class Controllers {
    public static final String DRIVER_NAME          = "Driver Xbox Controller";
    public static final int    DRIVER_PORT           = 0;
  }
}
