// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Shoot extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Shooter m_shooter; 
  private final Feeder m_feeder; 

  private final Pivot m_pivot; 

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Shoot(Shooter shooter, Feeder feeder, Pivot pivot) {
    m_shooter = shooter;
    m_feeder = feeder;
    m_pivot = pivot; 

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter,feeder,pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.shoot(4000);
    m_feeder.prepareShoot(1000);
    m_pivot.setAngle(2.8);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_feeder.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
