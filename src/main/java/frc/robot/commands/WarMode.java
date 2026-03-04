// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeExtension;
import frc.robot.subsystems.Pivot;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class WarMode extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Pivot m_Pivot; 
  private final IntakeExtension m_IntakeExtension; 
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public WarMode(Pivot pivot, IntakeExtension intakeExtension) {
    m_Pivot = pivot; 
    m_IntakeExtension = intakeExtension; 

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot,intakeExtension);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Pivot.setAngle(Constants.PivotConstants.kMinAngleRad);
    m_IntakeExtension.setExtensionPosition(Constants.IntakeConstants.kExtensionPositionRetracted);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
