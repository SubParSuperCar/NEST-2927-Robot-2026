// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // Log and replay timestamp and joystick data
  private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay().withTimestampReplay()
      .withJoystickReplay();
  public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(0.01)
      .withRotationalDeadband(0.01)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    m_timeAndJoystickReplay.update();
    CommandScheduler.getInstance().run();
  }

  /*
   * @Override
   * public void disabledInit() {
   * }
   *
   * @Override
   * public void disabledPeriodic() {
   * }
   *
   * @Override
   * public void disabledExit() {
   * }
   */

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
    // Drives backward maybe
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () -> drive
                .withVelocityX(1) // Drive forward with negative Y
        )
            .andThen(new WaitCommand(1))
            .andThen(drivetrain.applyRequest(
                () -> drive
                    .withVelocityX(0))));
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }
  }

  @Override
  public void teleopPeriodic() {
    // LimelightHelpers.setLEDMode_ForceBlink("");
  }

  /*
   * @Override
   * public void teleopExit() {
   * }
   *
   * @Override
   * public void testInit() {
   * CommandScheduler.getInstance().cancelAll();
   * }
   *
   * @Override
   * public void testPeriodic() {
   * }
   *
   * @Override
   * public void testExit() {
   * }
   *
   * @Override
   * public void simulationPeriodic() {
   * }
   */
}
