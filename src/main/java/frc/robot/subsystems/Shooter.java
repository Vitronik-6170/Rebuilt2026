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


  public Shooter() {
    ShooterConfig = new SparkMaxConfig(); 
    ShooterConfig.idleMode(IdleMode.kCoast); 
    ShooterConfig.voltageCompensation(Constants.Shooter.kVoltage); 
    ShooterConfig.closedLoop.feedForward.kV(Constants.Shooter.kV);
    ShooterConfig.closedLoop.pid(Constants.Shooter.kPShooter,Constants.Shooter.kIShooter, Constants.Shooter.kDShooter); 
    ShooterConfig.closedLoop.iZone(Constants.Shooter.iZone);
    ShooterConfig.closedLoop.outputRange(-Constants.Shooter.powerShooter, Constants.Shooter.powerShooter);
    ShooterConfig.inverted(false); 

    shooterBackMotor = new SparkMax(Constants.Shooter.kIDShooterBackMotor, MotorType.kBrushless); 
    shooterFrontMotor = new SparkMax(Constants.Shooter.kIDShooterFrontMotor, MotorType.kBrushless); 

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

  public void shoot(double velocity){
    shooterBackController.setSetpoint(velocity, ControlType.kVelocity); 
    shooterFrontController.setSetpoint(velocity, ControlType.kVelocity); 
  }
  public void stop(){
    shooterBackMotor.set(0);
    shooterFrontMotor.set(0);
  }

  //Metodo get velocidades 


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
