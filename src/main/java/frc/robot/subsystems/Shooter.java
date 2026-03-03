// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SparkMaxConfig ShooterConfig; 
  private final SparkMax shooterBackMotor;
  private final SparkMax shooterFrontMotor;
  private final RelativeEncoder shooterBackEncoder;
  private final RelativeEncoder shooterFrontEncoder; 
  private final SparkClosedLoopController shooterBackController; 
  private final SparkClosedLoopController shooterFrontController; 

  private double m_targetRpm = 0.0;

  public Shooter() {
    ShooterConfig = new SparkMaxConfig(); 
    ShooterConfig.idleMode(IdleMode.kCoast); 
    ShooterConfig.voltageCompensation(Constants.ShooterConstants.kVoltage); 
    ShooterConfig.closedLoop.feedForward.kV(Constants.ShooterConstants.kV);
    ShooterConfig.closedLoop.pid(Constants.ShooterConstants.kPShooter,Constants.ShooterConstants.kIShooter, Constants.ShooterConstants.kDShooter); 
    ShooterConfig.closedLoop.iZone(Constants.ShooterConstants.iZone);
    ShooterConfig.closedLoop.outputRange(-Constants.ShooterConstants.powerShooter, Constants.ShooterConstants.powerShooter);
    ShooterConfig.inverted(false); 

    shooterBackMotor = new SparkMax(Constants.ShooterConstants.kIDShooterBackMotor, MotorType.kBrushless); 
    shooterFrontMotor = new SparkMax(Constants.ShooterConstants.kIDShooterFrontMotor, MotorType.kBrushless); 
    shooterBackEncoder = shooterBackMotor.getEncoder(); 
    shooterFrontEncoder = shooterFrontMotor.getEncoder(); 

    shooterBackController = shooterBackMotor.getClosedLoopController();
    shooterFrontController = shooterFrontMotor.getClosedLoopController(); 

    shooterBackMotor.configure(ShooterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    shooterFrontMotor.configure(ShooterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  public void shoot(double velocity) {
    // Clamp para nunca salir del rango seguro
    m_targetRpm = Math.max(Constants.ShooterConstants.kMinRpm, Math.min(Constants.ShooterConstants.kMaxRpm, velocity));
    shooterBackController.setSetpoint(m_targetRpm,  ControlType.kVelocity);
    shooterFrontController.setSetpoint(m_targetRpm, ControlType.kVelocity);
  }

  public void stop() {
    m_targetRpm = 0.0;
    shooterBackMotor.set(0);
    shooterFrontMotor.set(0);
  }

  /** Retorna true si ambos motores alcanzaron las RPM objetivo. */
  public boolean atTargetRpm() {
    boolean backOk  = Math.abs(shooterBackEncoder.getVelocity()  - m_targetRpm) < Constants.ShooterConstants.kRpmTolerance;
    boolean frontOk = Math.abs(shooterFrontEncoder.getVelocity() - m_targetRpm) < Constants.ShooterConstants.kRpmTolerance;
    return backOk && frontOk;
  }
  public double getBackVelocity()  { return shooterBackEncoder.getVelocity();  }
  public double getFrontVelocity() { return shooterFrontEncoder.getVelocity(); }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter/BackRPM",   getBackVelocity());
    SmartDashboard.putNumber("Shooter/FrontRPM",  getFrontVelocity());
    SmartDashboard.putNumber("Shooter/TargetRPM", m_targetRpm);
    SmartDashboard.putBoolean("Shooter/AtSpeed",  atTargetRpm());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
