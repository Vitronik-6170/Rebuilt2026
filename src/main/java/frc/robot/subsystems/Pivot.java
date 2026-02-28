// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

  package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
  private final SparkMaxConfig pivotConfig; 
  private final SparkMax pivotMotor;
  private final AbsoluteEncoder pivotEncoder;
  private final SparkClosedLoopController pivotController; 
 
  /** Creates a new ExampleSubsystem. */
  public Pivot() {
    pivotConfig = new SparkMaxConfig();
    pivotConfig.idleMode(IdleMode.kBrake); 
    pivotConfig.inverted(false); 
    pivotConfig.absoluteEncoder.inverted(true); 
    pivotConfig.absoluteEncoder.positionConversionFactor(Math.PI*2); 
    pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder); 
    pivotConfig.closedLoop.pid(1, 0, 0);
    pivotConfig.closedLoop.outputRange(-1, 1); 
    pivotConfig.closedLoop.positionWrappingEnabled(true); 
    pivotConfig.closedLoop.positionWrappingInputRange(0, Math.PI*2);
   
    pivotMotor = new SparkMax(11, MotorType.kBrushless);
    pivotEncoder = pivotMotor.getAbsoluteEncoder();
    pivotController  = pivotMotor.getClosedLoopController(); 

    pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

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

  public void setAngle(double angle){
    pivotController.setSetpoint(angle, ControlType.kPosition);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
