// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FuelIntake;
import frc.robot.subsystems.FuelShooter;

public class RobotContainer {

  // Drive input constants

  /** Fraction of kSpeedAt12Volts to use as the top translational speed. */
  private static final double MAX_SPEED_SCALE  = 1.0;

  /** Maximum rotational rate  - 3/4 of a full rotation per second in rad/s. */
  private static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  /**
   * Joystick deadband as a fraction of full throw (12.5%).
   * Axis values within this range are zeroed. Beyond it, output is linearly
   * re-scaled from 0 at the edge to 1 at full throw  - no sudden step.
   * The same value is mirrored into the SwerveRequest as a low-level safety
   * floor; because curve() already outputs 0 inside the deadband, the
   * request-level deadband only activates if curve() is bypassed.
   */
  private static final double INPUT_DEADBAND   = 0.125;

  // Cached top speed so the drive lambda captures a plain double, not a
  // full Units expression evaluated every 20 ms.
  private final double maxSpeed = MAX_SPEED_SCALE * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  // Subsystems

  private final FuelIntake              intake      = new FuelIntake();
  private final FuelShooter             fuelShooter = new FuelShooter();
  public  final CommandSwerveDrivetrain drivetrain  = TunerConstants.createDrivetrain();

  // Field-centric open-loop drive request.
  // Deadbands are set in physical units (m/s, rad/s) as a safety floor.
  // Primary deadband + input shaping is handled upstream by curve().
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(maxSpeed * INPUT_DEADBAND)
      .withRotationalDeadband(MAX_ANGULAR_RATE * INPUT_DEADBAND)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /*
   * private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
   * private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
   */

  private final Telemetry             logger   = new Telemetry(maxSpeed);
  private final CommandXboxController joystick =
      new CommandXboxController(DeviceConstants.Controllers.DRIVER_PORT);

  public RobotContainer() {
    configureBindings();
  }

  /**
   * Applies a deadband then a squared-input curve to a raw joystick axis value.
   *
   * Values within [-deadband, +deadband] return 0.0. Values outside are
   * linearly re-scaled so output is 0 at the deadband edge and 1 at full throw
   * (no discontinuous jump), then squared for improved fine-control sensitivity
   * while still reaching full output at the mechanical stop.
   *
   * @param raw      Raw axis value in [-1, 1].
   * @param deadband Deadband threshold as a fraction of full throw.
   * @return Shaped output in [-1, 1].
   */
  private static double curve(double raw, double deadband) {
    if (Math.abs(raw) < deadband) return 0.0;
    double scaled = (Math.abs(raw) - deadband) / (1.0 - deadband);
    return Math.copySign(scaled * scaled, raw);
  }

  private void configureBindings() {

    // Fuel shooter  - hold right trigger to spin up; auto-stops on release.
    joystick.rightTrigger()
        .whileTrue(Commands.startEnd(fuelShooter::run, fuelShooter::stop, fuelShooter));

    // Fuel intake  - right bumper in, left bumper out.
    // startEnd ensures the motor stops on release even if the command is interrupted.
    joystick.rightBumper()
        .whileTrue(Commands.startEnd(intake::intakeIn,  intake::intakeStop, intake));
    joystick.leftBumper()
        .whileTrue(Commands.startEnd(intake::intakeOut, intake::intakeStop, intake));

    // Deploy arm  - hold A (button 1) to extend; retracts automatically on release.
    joystick.button(1)
        .whileTrue(Commands.startEnd(intake::deployExtend, intake::deployRetract, intake));

    // Field-centric heading reset  - back button alone.
    // Negating Y and X prevents conflict with the SysId combos below.
    joystick.back()
        .and(joystick.y().negate())
        .and(joystick.x().negate())
        .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    // Default drive command.
    // curve() shapes each axis before the request receives it, giving a
    // smooth linear ramp from the deadband edge to full speed.
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() ->
            drive
                .withVelocityX(     curve(-joystick.getLeftY(),   INPUT_DEADBAND) * maxSpeed)
                .withVelocityY(     curve(-joystick.getLeftX(),   INPUT_DEADBAND) * maxSpeed)
                .withRotationalRate(curve(-joystick.getRightX(),  INPUT_DEADBAND) * MAX_ANGULAR_RATE)
        )
    );

    // Idle request while disabled  - applies neutral mode to drive motors on
    // the field before the match starts.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // SysId characterization  - run each routine exactly once per log session.
    // back  + Y → dynamic forward       back  + X → dynamic reverse
    // start + Y → quasistatic forward   start + X → quasistatic reverse
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  /*
   * public Command getAutonomousCommand() {
   *   return new InstantCommand(intake::deployExtend, intake)
   *       .andThen(new WaitCommand(1.0))
   *       .andThen(new InstantCommand(intake::deployStop, intake));
   * }
   */
}
