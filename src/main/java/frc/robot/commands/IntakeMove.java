// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeExtension;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakeMove extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final IntakeExtension m_IntakeExtension;
  private final Intake m_Intake;

  /**
   * Creates a new IntakeMove.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeMove(IntakeExtension intakeExtension, Intake intake) {
    m_IntakeExtension = intakeExtension;
    m_Intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeExtension, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeExtension.setExtensionPosition(Constants.IntakeConstants.kExtensionPositionExtended);
    m_Intake.setIntakePower(Constants.IntakeConstants.kIntakeMotorPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeExtension.setExtensionPosition(Constants.IntakeConstants.kExtensionPositionRetracted);
    m_Intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
