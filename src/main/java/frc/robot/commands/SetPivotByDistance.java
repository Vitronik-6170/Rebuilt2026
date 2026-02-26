package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class SetPivotByDistance extends Command {

  private final PivotSubsystem m_pivot;
  private final DriveSubsystem m_drive;

  private static final Translation2d HUB_POSITION = new Translation2d(8.27, 4.10);

  // ── CALIBRAR EN CANCHA ────────────────────────────────────────────────────
  private static final double[] kDistances = { 1.0,  2.0,  3.0,  4.0,  5.0  };
  private static final double[] kAngles    = { 0.30, 0.55, 0.80, 1.05, 1.30 };
  // ──────────────────────────────────────────────────────────────────────────

  public SetPivotByDistance(PivotSubsystem pivot, DriveSubsystem drive) {
    this.m_pivot = pivot;
    this.m_drive = drive;
    addRequirements(pivot);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Pose2d robotPose = m_drive.getPose();
    double distance = Math.hypot(
        HUB_POSITION.getX() - robotPose.getX(),
        HUB_POSITION.getY() - robotPose.getY());

    double targetAngle = interpolate(distance);
    m_pivot.setAngle(targetAngle);

    SmartDashboard.putNumber("Pivot/Distance_m",   distance);
    SmartDashboard.putNumber("Pivot/TargetAngle_rad", targetAngle);
  }

  @Override
  public void end(boolean interrupted) {
    m_pivot.setAngle(PivotSubsystem.kMinAngleRad);
  }

  @Override
  public boolean isFinished() { return false; }

  private double interpolate(double distance) {
    if (distance <= kDistances[0])                         return kAngles[0];
    if (distance >= kDistances[kDistances.length - 1])     return kAngles[kAngles.length - 1];

    for (int i = 0; i < kDistances.length - 1; i++) {
      if (distance >= kDistances[i] && distance <= kDistances[i + 1]) {
        double t = (distance - kDistances[i]) / (kDistances[i + 1] - kDistances[i]);
        return kAngles[i] + t * (kAngles[i + 1] - kAngles[i]);
      }
    }
    return kAngles[0];
  }
}