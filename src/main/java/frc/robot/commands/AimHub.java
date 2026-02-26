package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class AimHub extends Command {

  private final DriveSubsystem m_drive;

  private static final String LIMELIGHT_NAME = "limelight";

  // ── AJUSTA ESTO AL HUB DE TU CAMPO ───────────────────────────────────────
  private static final Translation2d HUB_POSITION = new Translation2d(8.27, 4.10);
  // ──────────────────────────────────────────────────────────────────────────

  private static final double ROTATION_TOLERANCE_RAD          = Units.degreesToRadians(2.0);
  private static final double MAX_ROT_VEL_RAD_PER_S           = 4.0;
  private static final double MAX_ROT_ACCEL_RAD_PER_S2        = 6.0;
  private static final double MAX_YAW_RATE_FOR_VISION_DEG_PER_S = 720.0;
  private static final double FLIGHT_TIME_S                   = 0.2;
  private static final double DRIVER_SPEED                    = 0.5;

  private final ProfiledPIDController m_rotationController = new ProfiledPIDController(
      4.0, 0.0, 0.1,
      new TrapezoidProfile.Constraints(MAX_ROT_VEL_RAD_PER_S, MAX_ROT_ACCEL_RAD_PER_S2));

  private boolean m_isAligned = false;

  public AimHub(DriveSubsystem drive) {
    this.m_drive = drive;
    addRequirements(drive);
    m_rotationController.setTolerance(ROTATION_TOLERANCE_RAD);
    m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    m_rotationController.reset(m_drive.getPose().getRotation().getRadians());
    m_isAligned = false;
  }

  @Override
  public void execute() {
    // 1. Fusionar visión
    updatePoseWithVision();

    // 2. Pose actual
    Pose2d robotPose = m_drive.getPose();

    // 3. Lead compensation (compensar movimiento del robot)
    ChassisSpeeds fieldSpeeds = m_drive.getFieldRelativeSpeeds();
    double targetX = HUB_POSITION.getX() - fieldSpeeds.vxMetersPerSecond * FLIGHT_TIME_S;
    double targetY = HUB_POSITION.getY() - fieldSpeeds.vyMetersPerSecond * FLIGHT_TIME_S;

    // 4. Ángulo hacia el hub compensado
    double desiredAngleRad = Math.atan2(
        targetY - robotPose.getY(),
        targetX - robotPose.getX());

    // 5. Distancia (para SmartDashboard)
    double distanceToHub = Math.hypot(
        HUB_POSITION.getX() - robotPose.getX(),
        HUB_POSITION.getY() - robotPose.getY());

    // 6. PID rotación
    double rotationOutput = m_rotationController.calculate(
        robotPose.getRotation().getRadians(), desiredAngleRad);

    m_isAligned = m_rotationController.atGoal();

    // 7. Driver controla traslación, robot controla rotación
    double y = -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(), OIConstants.kDriveDeadband);
    double x = -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftX(), OIConstants.kDriveDeadband);

    m_drive.drive(y * DRIVER_SPEED, x * DRIVER_SPEED, rotationOutput, true);

    SmartDashboard.putNumber("AimHub/DistanceToHub_m",    distanceToHub);
    SmartDashboard.putNumber("AimHub/DesiredAngle_deg",   Math.toDegrees(desiredAngleRad));
    SmartDashboard.putNumber("AimHub/CurrentAngle_deg",   robotPose.getRotation().getDegrees());
    SmartDashboard.putNumber("AimHub/RotError_deg",       Math.toDegrees(m_rotationController.getPositionError()));
    SmartDashboard.putBoolean("AimHub/IsAligned",         m_isAligned);
  }

  private void updatePoseWithVision() {
    LimelightHelpers.SetRobotOrientation(
        LIMELIGHT_NAME,
        m_drive.getHeading(),
        m_drive.getYawRateDegPerSec(),
        0, 0, 0, 0);

    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);

    if (estimate == null)                                          return;
    if (estimate.tagCount == 0)                                    return;
    if (estimate.pose.getX() == 0 && estimate.pose.getY() == 0)   return;
    if (m_drive.getYawRateDegPerSec() > MAX_YAW_RATE_FOR_VISION_DEG_PER_S) return;

    m_drive.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);

    SmartDashboard.putNumber("AimHub/Vision/TagCount",    estimate.tagCount);
    SmartDashboard.putNumber("AimHub/Vision/AvgTagDist_m", estimate.avgTagDist);
  }

  public boolean isAligned() { return m_isAligned; }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, true);
  }

  @Override
  public boolean isFinished() { return false; }
}