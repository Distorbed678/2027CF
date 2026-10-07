// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
  private final Vision m_vision = new Vision(m_drivetrain);
  private final Telemetry m_telemetry = new Telemetry();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /* Swerve requests */
  private final SwerveRequest.FieldCentric m_fieldCentricDrive =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.Idle m_idle = new SwerveRequest.Idle();

  private final SendableChooser<Command> m_autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    registerNamedCommands();

    if (AutoBuilder.isConfigured()) {
      m_autoChooser = AutoBuilder.buildAutoChooser();
    } else {
      // PathPlanner robot config is missing; see CommandSwerveDrivetrain#configureAutoBuilder.
      m_autoChooser = new SendableChooser<>();
      m_autoChooser.setDefaultOption("None (configure PathPlanner)", Commands.none());
    }
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    configureBindings();
    m_drivetrain.registerTelemetry(m_telemetry::telemeterize);
  }

  /**
   * Register commands that PathPlanner autos can trigger by name (event markers / named commands).
   * Must run before the auto chooser is built.
   */
  private void registerNamedCommands() {
    // Example: NamedCommands.registerCommand("Intake", m_intake.intakeCommand());
  }

  private void configureBindings() {
    // Field-centric drive: left stick translates, right stick X rotates.
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    m_drivetrain.setDefaultCommand(
        m_drivetrain.applyRequest(
            () -> {
              double scale =
                  m_driverController.rightBumper().getAsBoolean()
                      ? DriveConstants.kSlowModeScale
                      : 1.0;
              return m_fieldCentricDrive
                  .withVelocityX(
                      shapeInput(-m_driverController.getLeftY()) * DriveConstants.kMaxSpeed * scale)
                  .withVelocityY(
                      shapeInput(-m_driverController.getLeftX()) * DriveConstants.kMaxSpeed * scale)
                  .withRotationalRate(
                      shapeInput(-m_driverController.getRightX())
                          * DriveConstants.kMaxAngularRate
                          * scale);
            }));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    RobotModeTriggers.disabled()
        .whileTrue(m_drivetrain.applyRequest(() -> m_idle).ignoringDisable(true));

    // X: lock wheels in an X pattern to resist being pushed.
    m_driverController.x().whileTrue(m_drivetrain.applyRequest(() -> m_brake));

    // Start: re-zero field heading. Point the robot away from your driver station first.
    m_driverController
        .start()
        .onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric).ignoringDisable(true));

    // Back: snap the pose (position and heading) to the latest multi-tag vision estimate.
    m_driverController
        .back()
        .onTrue(
            Commands.runOnce(
                    () -> m_vision.getRecentMultiTagPose().ifPresent(m_drivetrain::resetPose))
                .ignoringDisable(true));
  }

  /** Applies deadband and squares the input (keeping sign) for finer low-speed control. */
  private static double shapeInput(double value) {
    value = MathUtil.applyDeadband(value, OperatorConstants.kDeadband);
    return Math.copySign(value * value, value);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
