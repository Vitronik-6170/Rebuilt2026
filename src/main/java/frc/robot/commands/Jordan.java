// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.IntakeExtension;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Jordan extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Shooter m_shooter;
  private final Feeder m_feeder;
  private final Pivot m_pivot;
  private final IntakeExtension m_intakeExtension;

  private final Timer m_firingTimer = new Timer();
  private boolean m_firingStarted  = false;
  private boolean m_intakeRetracted = false;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Jordan(Shooter shooter, Feeder feeder, Pivot pivot, IntakeExtension intakeExtension) {
    m_shooter = shooter;
    m_feeder = feeder;
    m_pivot = pivot;
    m_intakeExtension = intakeExtension;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter, m_feeder, m_pivot, m_intakeExtension);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_firingStarted   = false;
    m_intakeRetracted = false;
    m_firingTimer.reset();
    m_firingTimer.stop();
    m_intakeExtension.setSpeed(0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pivot.setAngle(2.7);
    m_shooter.shoot(4000);

    // 4. El feeder solo alimenta si el pivot Y el shooter están listos
    //    Esto evita disparos imprecisos mientras el sistema está ajustando
    if (m_pivot.atSetpoint() && m_shooter.atTargetRpm()) {
      m_feeder.prepareShoot(4000);
      if (!m_firingStarted) {
        m_firingStarted = true;
        m_firingTimer.reset();
        m_firingTimer.start();
      }
      if (!m_intakeRetracted && m_firingTimer.hasElapsed(0.2)) {
        m_intakeExtension.setExtensionPosition(0);
        m_intakeRetracted = true;
      }
    } else {
      m_feeder.stop(); // Esperamos a que todo esté listo
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feeder.stop();
    m_shooter.stop();
    m_pivot.setAngle(0);
    m_intakeExtension.setSpeed(0.9);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
