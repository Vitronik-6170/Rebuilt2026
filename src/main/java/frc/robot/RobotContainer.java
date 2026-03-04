// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.FieldPositions;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.GoToPose;
import frc.robot.commands.IntakeMove;
import frc.robot.commands.OutTakeMove;
import frc.robot.commands.Shoot;
import frc.robot.commands.WarMode;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeExtension;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer {

  // ================= SUBSYSTEMS =================
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final DriveSubsystem m_robotDrive;
  private final LimelightSubsystem m_limelight;

  public final IntakeExtension m_IntakeExtension;
  public final Intake m_Intake;
  public final Shooter m_Shooter;
  public final Feeder m_Feeder;
  public final Pivot m_Pivot;

  // ================= CONTROLLERS =================
  public static CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public static CommandXboxController m_mechanismController =
      new CommandXboxController(OperatorConstants.kMechanismControllerPort);

  // ================= AUTO =================
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    // Inicializar subsistemas
    m_robotDrive = new DriveSubsystem();
    m_limelight = new LimelightSubsystem(m_robotDrive);

    m_IntakeExtension = new IntakeExtension();
    m_Intake = new Intake();
    m_Shooter = new Shooter();
    m_Feeder = new Feeder();
    m_Pivot = new Pivot();

    // Configurar botones
    configureBindings();

    // Default drive command
    m_robotDrive.setDefaultCommand(
        new RunCommand(
                () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
    // Auto chooser con PathPlanner
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  // ================= BUTTON BINDINGS =================
  private void configureBindings() {

    // Posiciones predefinidas
    m_driverController.leftBumper()
        .whileTrue(new GoToPose(FieldPositions.kPosition1, m_robotDrive));

    m_driverController.rightBumper()
        .whileTrue(new GoToPose(FieldPositions.kPosition2, m_robotDrive));

    m_driverController.x()
        .whileTrue(new GoToPose(FieldPositions.kPosition3, m_robotDrive));

    m_driverController.a()
        .whileTrue(new GoToPose(FieldPositions.kPosition4, m_robotDrive));

    m_driverController.b()
        .whileTrue(new GoToPose(FieldPositions.kPosition5, m_robotDrive));

    //Botón human 
    m_driverController.y()
        .whileTrue(new GoToPose(FieldPositions.kHuman, m_robotDrive));

    //Boton endgame 
    m_driverController.povUp()
        .whileTrue(new GoToPose(FieldPositions.kEndGame, m_robotDrive));
    
    //Botón modo guerra (se viene defensa padrísima amiguitos :O -> terror llamen a Ozuna)   
    m_mechanismController.x()
        .whileTrue(new WarMode(m_Pivot, m_IntakeExtension));

    // Intake
    m_mechanismController.rightBumper()
        .whileTrue(new IntakeMove(m_IntakeExtension, m_Intake));

    m_mechanismController.leftBumper()
        .whileTrue(new OutTakeMove(m_IntakeExtension, m_Intake));

    // Lock wheels
    m_driverController.y()
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

    // Shoot
    m_mechanismController.a()
        .whileTrue(new Shoot(m_Shooter, m_Feeder, m_Pivot, m_robotDrive,m_IntakeExtension));
  
}  

  // ================= AUTO COMMAND =================
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}