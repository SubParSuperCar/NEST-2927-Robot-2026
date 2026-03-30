// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

  private double DriveInputDeadband = 1f / 12f;
  private double DriveInputSign = -1;

  private final FuelIntake intake = new FuelIntake();
  public final FuelShooter fuelShooter = new FuelShooter();

  // Setting up bindings for necessary control of the swerve drive platform
  public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(0.01)
      .withRotationalDeadband(0.01)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public RobotContainer() {
    configureBindings();
  }

  public void negateDriveControls() {
    DriveInputSign = -DriveInputSign;
  }

  private double applyDeadband(double raw, double deadband) {
    var abs = Math.abs(raw);
    if (Math.abs(raw) < deadband)
      return 0.0;

    return Math.copySign(
        Math.pow((abs - deadband) / (1.0 - deadband), 2), raw);
  }

  private void configureBindings() {
    // Deploy joypad bindings
    joystick.button(1).whileTrue(new InstantCommand(intake::deployExtend, intake));
    joystick.button(1).onFalse(new InstantCommand(intake::deployStop, intake));

    // Intake joypad bindings
    joystick.rightBumper().whileTrue(new InstantCommand(intake::intakeIn, intake));
    joystick.leftBumper().whileTrue(new InstantCommand(intake::intakeOut, intake));
    joystick.rightBumper().onFalse(new InstantCommand(intake::intakeStop, intake));
    joystick.leftBumper().onFalse(new InstantCommand(intake::intakeStop, intake));

    // Fuel shooter joypad bindings
    joystick.rightTrigger().whileTrue(new RunCommand(() -> fuelShooter.run(), fuelShooter));
    joystick.rightTrigger().onFalse(new InstantCommand(() -> fuelShooter.stop(), fuelShooter));

    joystick.button(2).onTrue(new InstantCommand(fuelShooter::increaseSpeed, fuelShooter));
    joystick.button(3).onTrue(new InstantCommand(fuelShooter::decreaseSpeed, fuelShooter));
    joystick.button(4).onTrue(new InstantCommand(fuelShooter::resetSpeed, fuelShooter));

    // Drivetrain negation binding
    joystick.button(8).onTrue(new InstantCommand(() -> negateDriveControls()));

    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () -> drive
                .withVelocityX(applyDeadband(joystick.getLeftY() * DriveInputSign, DriveInputDeadband) * MaxSpeed)
                .withVelocityY(
                    applyDeadband(joystick.getLeftX() * DriveInputSign, DriveInputDeadband) * MaxSpeed)
                .withRotationalRate(
                    applyDeadband(joystick.getRightX() * DriveInputSign, DriveInputDeadband) * MaxAngularRate)));

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

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return drivetrain.applyRequest(
        () -> drive.withVelocityX(1))
        .alongWith(Commands.runOnce(intake::deployExtend, intake))
        .andThen(new WaitCommand(1))
        .andThen(new InstantCommand(intake::deployStop, intake))
        .alongWith(new WaitCommand(1))
        .andThen(drivetrain.applyRequest(
            () -> drive.withVelocityX(0))
            .andThen(new InstantCommand(() -> fuelShooter.run(), fuelShooter)));
  }
}
