// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    /** Joystick deadband, as a fraction of full stick travel. */
    public static final double kDeadband = 0.1;
  }

  public static class DriveConstants {
    /** Top translational speed in m/s. */
    public static final double kMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    /** Top rotational speed in rad/s (0.75 rotations per second). */
    public static final double kMaxAngularRate =
        RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    /** Speed multiplier while the slow-mode button is held. */
    public static final double kSlowModeScale = 0.35;
  }

  public static class VisionConstants {
    /** AprilTag layout. TODO: switch to the 2027 field once WPILib publishes it. */
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Camera names MUST match the names set for each camera in the PhotonVision UI.
    public static final String kFrontLeftCameraName = "FrontLeft";
    public static final String kFrontRightCameraName = "FrontRight";
    public static final String kBackLeftCameraName = "BackLeft";
    public static final String kBackRightCameraName = "BackRight";

    // Robot-to-camera transforms (robot center on the floor -> camera lens).
    // X forward, Y left, Z up. Negative pitch tilts the camera UP.
    // TODO: PLACEHOLDERS - measure the real mounting positions from CAD or the robot.
    private static final double kCamXY = Units.inchesToMeters(11.5);
    private static final double kCamZ = Units.inchesToMeters(10.0);
    private static final double kCamPitch = Units.degreesToRadians(-15.0);

    public static final Transform3d kRobotToFrontLeftCam =
        new Transform3d(
            new Translation3d(kCamXY, kCamXY, kCamZ),
            new Rotation3d(0, kCamPitch, Units.degreesToRadians(45)));
    public static final Transform3d kRobotToFrontRightCam =
        new Transform3d(
            new Translation3d(kCamXY, -kCamXY, kCamZ),
            new Rotation3d(0, kCamPitch, Units.degreesToRadians(-45)));
    public static final Transform3d kRobotToBackLeftCam =
        new Transform3d(
            new Translation3d(-kCamXY, kCamXY, kCamZ),
            new Rotation3d(0, kCamPitch, Units.degreesToRadians(135)));
    public static final Transform3d kRobotToBackRightCam =
        new Transform3d(
            new Translation3d(-kCamXY, -kCamXY, kCamZ),
            new Rotation3d(0, kCamPitch, Units.degreesToRadians(-135)));

    // ---- Measurement trust (standard deviations) ----
    // XY std dev = base * (avg tag distance)^2 / tag count. Smaller = more trusted.
    public static final double kSingleTagBaseXYStdDev = 0.3;
    public static final double kMultiTagBaseXYStdDev = 0.1;
    /** Heading std dev for multi-tag estimates while DISABLED, so tags can set field heading. */
    public static final double kDisabledMultiTagThetaStdDev = 0.5;
    /** Heading std dev otherwise: effectively "ignore vision heading, trust the Pigeon". */
    public static final double kIgnoreThetaStdDev = 9_999_999.0;

    // ---- Rejection thresholds ----
    /** Single-tag estimates whose tags are farther than this (m) are rejected. */
    public static final double kMaxSingleTagDistance = 4.0;
    /** Multi-tag estimates whose tags average farther than this (m) are rejected. */
    public static final double kMaxMultiTagDistance = 6.0;
    /** Max pose ambiguity for the lowest-ambiguity single-tag fallback. */
    public static final double kMaxAmbiguity = 0.2;
    /** Max distance (m) the estimated robot pose may be off the floor. */
    public static final double kMaxZError = 0.25;
    /** Margin (m) outside the field boundary that is still accepted. */
    public static final double kFieldBorderMargin = 0.5;
  }
}
