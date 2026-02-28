// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Drive extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_SwerveDrive;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Drive(DriveSubsystem m_SwerveDrive) {
    this.m_SwerveDrive = m_SwerveDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_SwerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double y = -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(), OIConstants.kDriveDeadband);
    double x = -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftX(), OIConstants.kDriveDeadband);
    double z = -MathUtil.applyDeadband(RobotContainer.m_driverController.getRightX(), OIConstants.kDriveDeadband);
    if (Math.abs(x) < 0.15 && Math.abs(y) < 0.15 && Math.abs(z) < 0.15) {
      x = 0; y = 0; z = 0;
  }
    double speed =0.3;
    m_SwerveDrive.drive(y*speed, x*speed, z*speed, true);
    //m_SwerveDrive.drive(0.1, 0, 0, true);
    //m_SwerveDrive.drive(0, 0, 0.1, true);
  }

  // Called once the commpeeand ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
