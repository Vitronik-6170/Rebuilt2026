package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SetShooterByDistance extends Command {

  private final ShooterSubsystem m_shooter;
  private final DriveSubsystem   m_drive;

  private static final Translation2d HUB_POSITION = new Translation2d(8.27, 4.10);

  // ── CALIBRAR EN CANCHA (mismas distancias que el pivot) ───────────────────
  private static final double[] kDistances = { 1.0,  2.0,  3.0,  4.0,  5.0   };
  private static final double[] kRpms      = { 2000, 2500, 3000, 3500, 4000  };
  // ──────────────────────────────────────────────────────────────────────────

  public SetShooterByDistance(ShooterSubsystem shooter, DriveSubsystem drive) {
    this.m_shooter = shooter;
    this.m_drive   = drive;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Pose2d robotPose = m_drive.getPose();
    double distance = Math.hypot(
        HUB_POSITION.getX() - robotPose.getX(),
        HUB_POSITION.getY() - robotPose.getY());

    double targetRpm = interpolate(distance);
    m_shooter.setRpm(targetRpm);

    SmartDashboard.putNumber("Shooter/Distance_m",  distance);
    SmartDashboard.putNumber("Shooter/TargetRPM",   targetRpm);
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  }

  @Override
  public boolean isFinished() { return false; }

  private double interpolate(double distance) {
    if (distance <= kDistances[0])                         return kRpms[0];
    if (distance >= kDistances[kDistances.length - 1])     return kRpms[kRpms.length - 1];

    for (int i = 0; i < kDistances.length - 1; i++) {
      if (distance >= kDistances[i] && distance <= kDistances[i + 1]) {
        double t = (distance - kDistances[i]) / (kDistances[i + 1] - kDistances[i]);
        return kRpms[i] + t * (kRpms[i + 1] - kRpms[i]);
      }
    }
    return kRpms[0];
  }
}