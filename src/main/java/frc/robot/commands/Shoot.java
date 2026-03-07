// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.IntakeExtension;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Shoot extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Shooter m_shooter; 
  private final Feeder m_feeder; 
  private final Pivot m_pivot; 
  private final DriveSubsystem m_drive;
  private final IntakeExtension m_intakeExtension; 


  private static final Translation2d HUB_POSITION = new Translation2d(4.625, 4.0345);

  private static final double[] kDistances = {
    2.825,  // kPosition4 (centro)
    3.279,  // kPosition3 (cerca trinchera izquierda)
    3.321  // kPosition5 (cerca trinchera derecha)
    //3.453,  // kPosition1 (trinchera izquierda)
    //3.517   // kPosition2 (trinchera derecha)
  };
  // Angulo del pivot en radianes para cada distancia
  private static final double[] kAngles    = { 0.0, 0.5, 0.55};

  // RPM del flywheel para cada distancia (calibradas con el ángulo de arriba)
  private static final double[] kRpms = { 2400, 2500, 2500};

  // RPM del feeder — constante, solo alimenta cuando el shooter está listo
  private static final double kFeederRpm = 4000;

  //variables de retraerse y contraerse: 
  private final Timer m_firingTimer = new Timer();
  private boolean m_firingStarted  = false;
  private boolean m_intakeRetracted = false;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Shoot(Shooter shooter, Feeder feeder, Pivot pivot, DriveSubsystem drive, IntakeExtension intakeExtension) {
    m_shooter = shooter;
    m_feeder = feeder;
    m_pivot = pivot; 
    m_drive = drive;
    m_intakeExtension = intakeExtension; 

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter,feeder,pivot,intakeExtension);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     m_firingStarted   = false;
    m_intakeRetracted = false;
    m_firingTimer.reset();
    m_firingTimer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // 1. Calculamos distancia actual al hub usando la pose global
    Pose2d robotPose = m_drive.getPose();
    double distance = Math.hypot(
        HUB_POSITION.getX() - robotPose.getX(),
        HUB_POSITION.getY() - robotPose.getY());

    // 2. Interpolamos ángulo y RPM según la distancia
    double targetAngle = interpolate(distance, kAngles);
    double targetRpm   = interpolate(distance, kRpms);

    // 3. Siempre movemos el pivot y el shooter hacia sus valores objetivo
    m_pivot.setAngle(targetAngle);
    m_shooter.shoot(targetRpm);

    // 4. El feeder solo alimenta si el pivot Y el shooter están listos
    //    Esto evita disparos imprecisos mientras el sistema está ajustando
    if (m_pivot.atSetpoint() && m_shooter.atTargetRpm()) {
      m_feeder.prepareShoot(kFeederRpm);
      if (!m_firingStarted) {
        m_firingStarted = true;
        m_firingTimer.reset();
        m_firingTimer.start();
      }
      if (!m_intakeRetracted && m_firingTimer.hasElapsed(2.0)) {
        m_intakeExtension.setExtensionPosition(0);
        m_intakeRetracted = true;
      }
    } else {
      m_feeder.stop(); // Esperamos a que todo esté listo
    }

    // 5. Debug
    SmartDashboard.putNumber("Shoot/Distance_m",     distance);
    SmartDashboard.putNumber("Shoot/TargetAngle_rad", targetAngle);
    SmartDashboard.putNumber("Shoot/TargetRPM",       targetRpm);
    SmartDashboard.putBoolean("Shoot/PivotReady",     m_pivot.atSetpoint());
    SmartDashboard.putBoolean("Shoot/ShooterReady",   m_shooter.atTargetRpm());
    SmartDashboard.putBoolean("Shoot/Firing", m_pivot.atSetpoint() && m_shooter.atTargetRpm());
    
    // m_pivot.setAngle(0.5);
    // m_shooter.shoot(2500);
    // m_feeder.prepareShoot(4000);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_feeder.stop();
    m_pivot.setAngle(Constants.PivotConstants.kMinAngleRad);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  // ── Interpolación lineal reutilizable para ángulos y RPMs ─────────────────
  private double interpolate(double distance, double[] values) {
    if (distance <= kDistances[0])                         return values[0];
    if (distance >= kDistances[kDistances.length - 1])     return values[values.length - 1];

    for (int i = 0; i < kDistances.length - 1; i++) {
      if (distance >= kDistances[i] && distance <= kDistances[i + 1]) {
        double t = (distance - kDistances[i]) / (kDistances[i + 1] - kDistances[i]);
        return values[i] + t * (values[i + 1] - values[i]);
      }
    }
    return values[0];
  }
}
