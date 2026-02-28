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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  private final SparkMaxConfig feederConfig; 
  private final SparkMax feederMotor;
  private final RelativeEncoder feederEncoder;
  private final SparkClosedLoopController feederController; 


  /** Creates a new ExampleSubsystem. */
  public Feeder() {
    feederConfig = new SparkMaxConfig(); 
    feederConfig.idleMode(IdleMode.kCoast);     
    feederConfig.voltageCompensation(Constants.Shooter.kVoltage); 
    feederConfig.closedLoop.feedForward.kV(Constants.Shooter.kV);
    feederConfig.closedLoop.pid(Constants.Shooter.kPShooter,Constants.Shooter.kIShooter, Constants.Shooter.kDShooter); 
    feederConfig.closedLoop.iZone(Constants.Shooter.iZone);
    feederConfig.closedLoop.outputRange(-Constants.Shooter.powerShooter, Constants.Shooter.powerShooter);
    feederConfig.inverted(false); 

    feederMotor = new SparkMax(Constants.Shooter.kIDFeederMotor, null); 
    feederEncoder = feederMotor.getEncoder(); 
    feederController = feederMotor.getClosedLoopController();

    feederMotor.configure(feederConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
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

  public void prepareShoot(double velocity){
    feederController.setSetpoint(velocity, ControlType.kVelocity); 
  }
  public void stop(){
    feederMotor.set(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
