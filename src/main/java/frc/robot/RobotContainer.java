// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.WaitCommand;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.FuelShooter;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity
    private double InputDeadband = 0.075;

    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final FuelShooter fuelShooter = new FuelShooter();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Left trigger → runs left motor pair while held
        joystick.leftTrigger().whileTrue(
                new edu.wpi.first.wpilibj2.command.RunCommand(
                        () -> fuelShooter.runLeftPair(),
                        fuelShooter))
                .onFalse(
                        new edu.wpi.first.wpilibj2.command.InstantCommand(
                                () -> fuelShooter.stopLeftPair(),
                                fuelShooter));

        // Right trigger → runs right motor pair while held
        joystick.rightTrigger().whileTrue(
                new edu.wpi.first.wpilibj2.command.RunCommand(
                        () -> fuelShooter.runRightPair(),
                        fuelShooter))
                .onFalse(
                        new edu.wpi.first.wpilibj2.command.InstantCommand(
                                () -> fuelShooter.stopRightPair(),
                                fuelShooter));

        joystick.rightBumper().whileTrue(
                new InstantCommand(intake::intakeIn, intake));
        joystick.leftBumper().whileTrue(
                new InstantCommand(intake::intakeOut, intake));
        // When the bumper is released, the motor stops automatically
        joystick.rightBumper().onFalse(
                new InstantCommand(intake::stop, intake));
        joystick.leftBumper().onFalse(
                new InstantCommand(intake::stop, intake));
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                   // negative Y
                                                                                                   // (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return new InstantCommand(intake::deployOn, intake)
                .andThen(new WaitCommand(1.0))
                .andThen(new InstantCommand(intake::deployOff, intake))
                .andThen(/* rest of your auto path */);
    }

}

/*
 * // Simple drive forward auton
 * final var idle = new SwerveRequest.Idle();
 * return Commands.sequence(
 * // Reset our field centric heading to match the robot
 * // facing away from our alliance station wall (0 deg).
 * drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
 * // Then slowly drive forward (away from us) for 5 seconds.
 * drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
 * .withVelocityY(0)
 * .withRotationalRate(0))
 * .withTimeout(5.0),
 * // Finally idle for the rest of auton
 * drivetrain.applyRequest(() -> idle));
 */
