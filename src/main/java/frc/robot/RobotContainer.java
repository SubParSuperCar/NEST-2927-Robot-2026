// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FuelIntake;
import frc.robot.subsystems.FuelShooter;

public class RobotContainer {
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
  // speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
  // max angular velocity
  private double InputDeadband = 0.075; // Apply a 7.5% deadband

  private final FuelIntake intake = new FuelIntake();
  private final FuelShooter fuelShooter = new FuelShooter();

  // Setting up bindings for necessary control of the swerve drive platform
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * InputDeadband)
      .withRotationalDeadband(MaxAngularRate * InputDeadband)
      .withDriveRequestType(
          DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  /*
   * private final SwerveRequest.SwerveDriveBrake brake = new
   * SwerveRequest.SwerveDriveBrake();
   * private final SwerveRequest.PointWheelsAt point = new
   * SwerveRequest.PointWheelsAt();
   */

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public RobotContainer() {
    configureBindings();
  }

  private static double curve(double raw, double deadband) {
    if (Math.abs(raw) < deadband)
      return 0f;

    double scaled = (Math.abs(raw) - deadband) / (1f - deadband);

    return Math.copySign(scaled * scaled, raw);
  }

  private void configureBindings() {
    // Fuel shooter joypad bindings
    joystick.rightTrigger()
        .whileTrue(new RunCommand(() -> fuelShooter.run(), fuelShooter));
    joystick.rightTrigger().onFalse(new InstantCommand(() -> fuelShooter.stop(), fuelShooter));

    // Intake joypad bindings
    joystick.rightBumper().whileTrue(new InstantCommand(intake::intakeIn, intake));
    joystick.leftBumper().whileTrue(new InstantCommand(intake::intakeOut, intake));
    joystick.rightBumper().onFalse(new InstantCommand(intake::intakeStop, intake));
    joystick.leftBumper().onFalse(new InstantCommand(intake::intakeStop, intake));

    // Intake deploy joypad bindings
    joystick.button(1).onFalse(new InstantCommand(intake::deployStop, intake));
    joystick.button(1).whileTrue(new InstantCommand(intake::deployExtend, intake));

    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () -> drive
                .withVelocityX(curve(-joystick.getLeftY(), InputDeadband) * MaxSpeed) // Drive forward with negative Y
                .withVelocityY(
                    curve(-joystick.getLeftX(), InputDeadband) * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(
                    curve(-joystick.getRightX(), InputDeadband) * MaxAngularRate) // Drive counterclockwise with
        // negative X (left)
        ));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on left bumper press.
    // joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  /*
   * public Command getAutonomousCommand() {
   * return new InstantCommand(intake::deployExtend, intake)
   * .andThen(new WaitCommand(1.0))
   * .andThen(new InstantCommand(intake::deployStop, intake));
   * }
   */
}
