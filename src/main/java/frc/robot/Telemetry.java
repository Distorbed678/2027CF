// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Publishes drivetrain state to NetworkTables for AdvantageScope / Glass. Registered with {@code
 * drivetrain.registerTelemetry(logger::telemeterize)}, so it runs on the odometry thread.
 */
public class Telemetry {
  private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("DriveState");
  private final StructPublisher<Pose2d> m_posePub =
      m_table.getStructTopic("Pose", Pose2d.struct).publish();
  private final StructPublisher<ChassisSpeeds> m_speedsPub =
      m_table.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> m_moduleStatesPub =
      m_table.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> m_moduleTargetsPub =
      m_table.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
  private final DoublePublisher m_odomFreqPub =
      m_table.getDoubleTopic("OdometryFrequency").publish();

  private final Field2d m_field = new Field2d();

  public Telemetry() {
    SmartDashboard.putData("Field", m_field);
  }

  /** Accept the swerve drive state and publish it. */
  public void telemeterize(SwerveDriveState state) {
    m_posePub.set(state.Pose);
    m_speedsPub.set(state.Speeds);
    m_moduleStatesPub.set(state.ModuleStates);
    m_moduleTargetsPub.set(state.ModuleTargets);
    m_odomFreqPub.set(1.0 / state.OdometryPeriod);
    m_field.setRobotPose(state.Pose);
  }
}
