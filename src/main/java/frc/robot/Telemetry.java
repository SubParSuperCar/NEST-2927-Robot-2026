package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Telemetry {

  private final double maxSpeed;

  /**
   * @param maxSpeed Robot top speed in meters per second, used to scale module speed widgets.
   */
  public Telemetry(double maxSpeed) {
    this.maxSpeed = maxSpeed;
    SignalLogger.start();
    for (int i = 0; i < 4; ++i) {
      SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
    }
  }

  private final NetworkTableInstance inst           = NetworkTableInstance.getDefault();

  // DriveState table  - swerve drive state published for dashboards and logging.
  private final NetworkTable driveStateTable        = inst.getTable("DriveState");
  private final StructPublisher<Pose2d>             drivePose           =
      driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
  private final StructPublisher<ChassisSpeeds>      driveSpeeds         =
      driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
  private final StructArrayPublisher<SwerveModuleState>    driveModuleStates  =
      driveStateTable.getStructArrayTopic("ModuleStates",   SwerveModuleState.struct).publish();
  private final StructArrayPublisher<SwerveModuleState>    driveModuleTargets =
      driveStateTable.getStructArrayTopic("ModuleTargets",  SwerveModuleState.struct).publish();
  private final StructArrayPublisher<SwerveModulePosition> driveModulePositions =
      driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
  private final DoublePublisher driveTimestamp       =
      driveStateTable.getDoubleTopic("Timestamp").publish();
  private final DoublePublisher driveOdometryFrequency =
      driveStateTable.getDoubleTopic("OdometryFrequency").publish();

  // Pose table  - Field2d-compatible robot position for the dashboard field widget.
  private final NetworkTable        poseTable    = inst.getTable("Pose");
  private final DoubleArrayPublisher fieldPub    = poseTable.getDoubleArrayTopic("robotPose").publish();
  private final StringPublisher      fieldTypePub = poseTable.getStringTopic(".type").publish();

  // Mechanism2d widgets  - one per swerve module, shown on SmartDashboard.
  private final Mechanism2d[] m_moduleMechanisms = {
      new Mechanism2d(1, 1), new Mechanism2d(1, 1),
      new Mechanism2d(1, 1), new Mechanism2d(1, 1),
  };

  // Speed ligaments  - length scales with module speed.
  private final MechanismLigament2d[] m_moduleSpeeds = {
      m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
      m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
      m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
      m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
  };

  // Direction ligaments  - angle tracks module heading, fixed length.
  private final MechanismLigament2d[] m_moduleDirections = {
      m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
          .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
      m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
          .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
      m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
          .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
      m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
          .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
  };

  // Reusable pose array  - avoids allocating a new double[] every loop.
  private final double[] m_poseArray = new double[3];

  /** Publishes the swerve drive state to SmartDashboard and the SignalLogger. */
  public void telemeterize(SwerveDriveState state) {
    drivePose.set(state.Pose);
    driveSpeeds.set(state.Speeds);
    driveModuleStates.set(state.ModuleStates);
    driveModuleTargets.set(state.ModuleTargets);
    driveModulePositions.set(state.ModulePositions);
    driveTimestamp.set(state.Timestamp);
    driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

    SignalLogger.writeStruct("DriveState/Pose",     Pose2d.struct,         state.Pose);
    SignalLogger.writeStruct("DriveState/Speeds",   ChassisSpeeds.struct,  state.Speeds);
    SignalLogger.writeStructArray("DriveState/ModuleStates",    SwerveModuleState.struct,    state.ModuleStates);
    SignalLogger.writeStructArray("DriveState/ModuleTargets",   SwerveModuleState.struct,    state.ModuleTargets);
    SignalLogger.writeStructArray("DriveState/ModulePositions", SwerveModulePosition.struct, state.ModulePositions);
    SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

    // Field2d widget pose.
    fieldTypePub.set("Field2d");
    m_poseArray[0] = state.Pose.getX();
    m_poseArray[1] = state.Pose.getY();
    m_poseArray[2] = state.Pose.getRotation().getDegrees();
    fieldPub.set(m_poseArray);

    // Update per-module Mechanism2d widgets.
    for (int i = 0; i < 4; ++i) {
      m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
      m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
      m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * maxSpeed));
    }
  }
}
