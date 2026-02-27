// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeExtension extends SubsystemBase {
  /** Creates a new IntakeExtension. */
  private final DigitalInput limInput;
  private final DigitalInput limOInput;

  private final SparkMaxConfig extensionConfig;
  private final SparkMax extensionMotor;
  private final RelativeEncoder extensionEncoder;
  private final SparkClosedLoopController extensionController;

  public IntakeExtension() {
    limInput = new DigitalInput(Constants.IntakeConstants.kExtensionLimitInputChannel);
    limOInput = new DigitalInput(Constants.IntakeConstants.kExtensionLimitOInputChannel);

    extensionConfig = new SparkMaxConfig();
    extensionConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    extensionConfig.inverted(Constants.IntakeConstants.kExtensionMotorInverted);
    extensionConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    extensionConfig.closedLoop.pid(Constants.IntakeConstants.kPextensionMotor, Constants.IntakeConstants.kIextensionMotor, Constants.IntakeConstants.kDextensionMotor);
    extensionConfig.closedLoop.iZone(Constants.IntakeConstants.kExtensionIZone);
    extensionConfig.closedLoop.outputRange(-Constants.IntakeConstants.kExtensionMotorPower, Constants.IntakeConstants.kExtensionMotorPower);
    extensionMotor = new SparkMax(Constants.IntakeConstants.kIDextensionMotor, MotorType.kBrushless);
    extensionEncoder = extensionMotor.getEncoder();
    extensionController = extensionMotor.getClosedLoopController();

    extensionMotor.configure(extensionConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setExtensionPosition(double position) {
    extensionController.setSetpoint(position, ControlType.kPosition);
  }
  public void setEncoderPosition(double encoderPosition) {
    extensionEncoder.setPosition(encoderPosition);
  }
  public boolean getLimitInput() {
    return !limInput.get();
  }
  public boolean getLimitOInput() {
    return !limOInput.get();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
