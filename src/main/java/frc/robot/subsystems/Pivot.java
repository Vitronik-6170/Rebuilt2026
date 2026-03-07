// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

  package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {

  private double m_setpointRad = 0.0;

  private final SparkMaxConfig pivotConfig; 
  private final SparkMax pivotMotor;
  private final AbsoluteEncoder pivotEncoder;
  private final SparkClosedLoopController pivotController; 
 
  /** Creates a new ExampleSubsystem. */
  public Pivot() {
    pivotConfig = new SparkMaxConfig();
    pivotConfig.idleMode(IdleMode.kBrake); 
    pivotConfig.inverted(Constants.PivotConstants.kPivotMotorInverted); 
    pivotConfig.absoluteEncoder.inverted(Constants.PivotConstants.kPivotEncoderInverted); 
    pivotConfig.absoluteEncoder.positionConversionFactor(Constants.PivotConstants.kPivotEncoderPositionConversionFactor);
    pivotConfig.absoluteEncoder.zeroOffset(Constants.PivotConstants.kPivotEncoderZeroOffset);
    pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder); 
    pivotConfig.closedLoop.pid(Constants.PivotConstants.kPPivot, Constants.PivotConstants.kIPivot, Constants.PivotConstants.kDPivot);
    pivotConfig.closedLoop.outputRange(-Constants.PivotConstants.kPivotPower, Constants.PivotConstants.kPivotPower); 
    pivotConfig.closedLoop.positionWrappingEnabled(Constants.PivotConstants.kPivotPositionWrappingEnabled); 
    pivotConfig.closedLoop.positionWrappingInputRange(0, Math.PI*2);
   
    pivotMotor = new SparkMax(Constants.PivotConstants.kIDpivotMotor, MotorType.kBrushless);
    pivotEncoder = pivotMotor.getAbsoluteEncoder();
    pivotController  = pivotMotor.getClosedLoopController(); 

    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
   * Mueve el pivot al ángulo deseado en radianes.
   * Aplica clamp automático para proteger los límites mecánicos.
   */
  public void setAngle(double angleRad) {
    m_setpointRad = MathUtil.clamp(angleRad, Constants.PivotConstants.kMinAngleRad, Constants.PivotConstants.kMaxAngleRad);
    pivotController.setSetpoint(m_setpointRad, ControlType.kPosition);
  }
  /** Retorna el ángulo actual del encoder absoluto en radianes. */
  public double getAngle() {
    return pivotEncoder.getPosition();
  }
  /** Retorna true si el pivot está dentro de la tolerancia del setpoint. */
  public boolean atSetpoint() {
    return Math.abs(getAngle() - m_setpointRad) < Constants.PivotConstants.kAngleToleranceRad;
  }
  public void stop() {
    pivotMotor.stopMotor();
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
    SmartDashboard.putNumber("Pivot/Angle_rad",    getAngle());
    SmartDashboard.putNumber("Pivot/Angle_deg",    Math.toDegrees(getAngle()));
    SmartDashboard.putNumber("Pivot/Setpoint_rad", m_setpointRad);
    SmartDashboard.putBoolean("Pivot/AtSetpoint",  atSetpoint());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
