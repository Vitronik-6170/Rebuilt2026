// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldPositions;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.GoToPose;
import frc.robot.commands.IntakeMove;
import frc.robot.commands.OutTakeMove;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeExtension;
import frc.robot.subsystems.LimelightSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  
  public final DriveSubsystem m_DriveSubsystem;
   private final LimelightSubsystem m_limelight; // ← agrega esto


  public final IntakeExtension m_IntakeExtension;
  public final Intake m_Intake;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  public static CommandXboxController m_mechanismController =
      new CommandXboxController(OperatorConstants.kMechanismControllerPort);

 /* Path follower */
    private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_DriveSubsystem = new DriveSubsystem();
     m_limelight = new LimelightSubsystem(m_DriveSubsystem); 
    m_IntakeExtension = new IntakeExtension();
    m_Intake = new Intake();

    //SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
    //m_driverController.rightBumper().whileTrue(new IntakeMove(m_IntakeExtension, m_Intake));
    
    m_driverController.leftBumper().whileTrue(new GoToPose(FieldPositions.kPosition1, m_DriveSubsystem)); // Trinchera derecha 
    m_driverController.rightBumper().whileTrue(new GoToPose(FieldPositions.kPosition2, m_DriveSubsystem)); // Trincera izquierda
    m_driverController.x().whileTrue(new GoToPose(FieldPositions.kPosition3, m_DriveSubsystem)); // Cerca de trinchera izquierda)
    m_driverController.a().whileTrue(new GoToPose(FieldPositions.kPosition4, m_DriveSubsystem)); // Centro 
    m_driverController.b().whileTrue(new GoToPose(FieldPositions.kPosition5, m_DriveSubsystem)); // Cerca de trinchera derecha
    m_mechanismController.rightBumper().whileTrue(new IntakeMove(m_IntakeExtension, m_Intake));
    m_mechanismController.leftBumper().whileTrue(new OutTakeMove(m_IntakeExtension, m_Intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
