// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Reads AprilTags from four PhotonVision cameras and feeds the resulting robot poses into the
 * drivetrain's pose estimator, where they are fused with wheel odometry and the Pigeon 2 heading.
 *
 * <p>Per camera result, the estimation strategy is:
 *
 * <ul>
 *   <li>2+ tags: the multi-tag PnP pose solved on the coprocessor (most accurate).
 *   <li>1 tag, heading known: PnP-distance-trig solve, which uses the robot's gyro heading plus the
 *       tag distance instead of the ambiguous single-tag PnP solution.
 *   <li>1 tag, heading not yet known: lowest-ambiguity PnP solution with an ambiguity filter.
 * </ul>
 *
 * <p>Heading: while the robot is disabled, multi-tag estimates are allowed to correct heading so
 * the robot learns its field orientation from the tags before a match. While enabled, vision
 * heading is ignored and the Pigeon is the sole source of heading.
 */
public class Vision extends SubsystemBase {
  private final CommandSwerveDrivetrain m_drivetrain;
  private final List<Camera> m_cameras;

  /** True once heading can be trusted for the gyro-assisted single-tag solve. */
  private boolean m_headingTrusted = false;

  private Pose2d m_lastMultiTagPose = null;
  private double m_lastMultiTagTimestamp = Double.NEGATIVE_INFINITY;

  // Simulation only
  private VisionSystemSim m_visionSim;

  public Vision(CommandSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;
    m_cameras =
        List.of(
            new Camera(kFrontLeftCameraName, kRobotToFrontLeftCam),
            new Camera(kFrontRightCameraName, kRobotToFrontRightCam),
            new Camera(kBackLeftCameraName, kRobotToBackLeftCam),
            new Camera(kBackRightCameraName, kRobotToBackRightCam));

    if (RobotBase.isSimulation()) {
      setupSimulation();
    }
  }

  @Override
  public void periodic() {
    // Once enabled, heading is either seeded by the driver, reset by a PathPlanner auto, or
    // already corrected by multi-tag vision while disabled.
    if (DriverStation.isEnabled()) {
      m_headingTrusted = true;
    }

    // Give every estimator the latest fused heading for the gyro-assisted single-tag solve.
    double now = Timer.getFPGATimestamp();
    Rotation2d heading = m_drivetrain.getState().Pose.getRotation();
    for (Camera cam : m_cameras) {
      cam.estimator.addHeadingData(now, heading);
    }

    for (Camera cam : m_cameras) {
      for (PhotonPipelineResult result : cam.camera.getAllUnreadResults()) {
        processResult(cam, result);
      }
    }
  }

  private void processResult(Camera cam, PhotonPipelineResult result) {
    if (!result.hasTargets()) {
      return;
    }

    boolean isMultiTag = result.multitagResult.isPresent();
    boolean usedAmbiguityFallback = false;
    Optional<EstimatedRobotPose> estimate;
    if (isMultiTag) {
      estimate = cam.estimator.estimateCoprocMultiTagPose(result);
    } else if (m_headingTrusted) {
      estimate = cam.estimator.estimatePnpDistanceTrigSolvePose(result);
    } else {
      estimate = Optional.empty();
    }
    if (estimate.isEmpty() && !isMultiTag) {
      estimate = cam.estimator.estimateLowestAmbiguityPose(result);
      usedAmbiguityFallback = true;
    }
    if (estimate.isEmpty()) {
      return;
    }

    EstimatedRobotPose est = estimate.get();
    Pose3d pose3d = est.estimatedPose;
    Pose2d pose2d = pose3d.toPose2d();

    // Tag count and average distance from camera to the tags used.
    int tagCount = 0;
    double totalDistance = 0.0;
    double maxAmbiguity = 0.0;
    for (PhotonTrackedTarget target : est.targetsUsed) {
      if (kTagLayout.getTagPose(target.getFiducialId()).isEmpty()) {
        continue;
      }
      tagCount++;
      totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
      maxAmbiguity = Math.max(maxAmbiguity, target.getPoseAmbiguity());
    }

    String rejectReason = null;
    double avgDistance = tagCount > 0 ? totalDistance / tagCount : Double.POSITIVE_INFINITY;
    if (tagCount == 0) {
      rejectReason = "no known tags";
    } else if (Math.abs(pose3d.getZ()) > kMaxZError) {
      rejectReason = "off the floor";
    } else if (!isOnField(pose2d)) {
      rejectReason = "off the field";
    } else if (usedAmbiguityFallback && maxAmbiguity > kMaxAmbiguity) {
      rejectReason = "ambiguous";
    } else if (avgDistance > (tagCount > 1 ? kMaxMultiTagDistance : kMaxSingleTagDistance)) {
      rejectReason = "too far";
    }

    cam.tagCountPub.set(tagCount);
    if (rejectReason != null) {
      cam.rejectedPosePub.set(pose2d);
      cam.statusPub.set("Rejected: " + rejectReason);
      return;
    }

    // Trust falls off with distance squared and improves with more tags.
    boolean multi = tagCount > 1;
    double xyStdDev =
        (multi ? kMultiTagBaseXYStdDev : kSingleTagBaseXYStdDev)
            * avgDistance
            * avgDistance
            / tagCount;
    double thetaStdDev =
        (multi && DriverStation.isDisabled()) ? kDisabledMultiTagThetaStdDev : kIgnoreThetaStdDev;
    Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);

    m_drivetrain.addVisionMeasurement(pose2d, est.timestampSeconds, stdDevs);

    if (multi) {
      m_lastMultiTagPose = pose2d;
      m_lastMultiTagTimestamp = est.timestampSeconds;
      if (DriverStation.isDisabled()) {
        m_headingTrusted = true;
      }
    }

    cam.acceptedPosePub.set(pose2d);
    cam.statusPub.set(String.format("Accepted (%s, %.2f m)", est.strategy, avgDistance));
  }

  private static boolean isOnField(Pose2d pose) {
    return pose.getX() >= -kFieldBorderMargin
        && pose.getX() <= kTagLayout.getFieldLength() + kFieldBorderMargin
        && pose.getY() >= -kFieldBorderMargin
        && pose.getY() <= kTagLayout.getFieldWidth() + kFieldBorderMargin;
  }

  /**
   * Returns the most recent accepted multi-tag pose, if one was seen in the last half second.
   * Useful for snapping the robot's pose (including heading) to vision.
   */
  public Optional<Pose2d> getRecentMultiTagPose() {
    if (m_lastMultiTagPose == null || Timer.getFPGATimestamp() - m_lastMultiTagTimestamp > 0.5) {
      return Optional.empty();
    }
    return Optional.of(m_lastMultiTagPose);
  }

  // ---------------------------------------------------------------------------------------------
  // Simulation
  // ---------------------------------------------------------------------------------------------

  private void setupSimulation() {
    m_visionSim = new VisionSystemSim("main");
    m_visionSim.addAprilTags(kTagLayout);

    // Roughly an Arducam OV9281-class global shutter camera.
    SimCameraProperties props = new SimCameraProperties();
    props.setCalibration(1280, 800, Rotation2d.fromDegrees(70));
    props.setCalibError(0.35, 0.10);
    props.setFPS(30);
    props.setAvgLatencyMs(35);
    props.setLatencyStdDevMs(5);

    for (Camera cam : m_cameras) {
      PhotonCameraSim cameraSim = new PhotonCameraSim(cam.camera, props, kTagLayout);
      m_visionSim.addCamera(cameraSim, cam.robotToCamera);
    }

    SmartDashboard.putData("Vision Sim Field", m_visionSim.getDebugField());
  }

  @Override
  public void simulationPeriodic() {
    // CTRE's drivetrain sim has no separate ground-truth pose, so the simulated cameras see the
    // field from the estimated pose. This exercises the full vision pipeline end to end.
    m_visionSim.update(m_drivetrain.getState().Pose);
  }

  /** One PhotonVision camera, its pose estimator, and its NetworkTables debug outputs. */
  private static class Camera {
    final PhotonCamera camera;
    final PhotonPoseEstimator estimator;
    final Transform3d robotToCamera;
    final StructPublisher<Pose2d> acceptedPosePub;
    final StructPublisher<Pose2d> rejectedPosePub;
    final IntegerPublisher tagCountPub;
    final StringPublisher statusPub;

    Camera(String name, Transform3d robotToCamera) {
      this.camera = new PhotonCamera(name);
      this.robotToCamera = robotToCamera;
      this.estimator = new PhotonPoseEstimator(kTagLayout, robotToCamera);

      NetworkTable table = NetworkTableInstance.getDefault().getTable("Vision/" + name);
      acceptedPosePub = table.getStructTopic("AcceptedPose", Pose2d.struct).publish();
      rejectedPosePub = table.getStructTopic("RejectedPose", Pose2d.struct).publish();
      tagCountPub = table.getIntegerTopic("TagCount").publish();
      statusPub = table.getStringTopic("Status").publish();
    }
  }
}
