package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {

  private static final int kMotorCanId = 14;

  public static final double kMinAngleRad   = 0.0;
  public static final double kMaxAngleRad   = Math.PI;
  public static final double kAngleToleranceRad = 0.05;

  private final SparkMax m_motor;
  private final AbsoluteEncoder m_encoder;
  private final SparkClosedLoopController m_controller;

  private double m_setpointRad = 0.0;

  public PivotSubsystem() {
    m_motor      = new SparkMax(kMotorCanId, MotorType.kBrushless);
    m_encoder    = m_motor.getAbsoluteEncoder();
    m_controller = m_motor.getClosedLoopController();

    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    config.absoluteEncoder.inverted(true);
    config.absoluteEncoder.positionConversionFactor(Math.PI * 2);
    config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    config.absoluteEncoder.zeroOffset(0);
    config.closedLoop.pid(1, 0.0, 0.0);
    config.closedLoop.outputRange(-1, 1);
    config.closedLoop.positionWrappingEnabled(true);
    config.closedLoop.positionWrappingInputRange(0, Math.PI * 2);

    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot/Angle_rad",  getAngle());
    SmartDashboard.putNumber("Pivot/Angle_deg",  Math.toDegrees(getAngle()));
    SmartDashboard.putNumber("Pivot/Setpoint_rad", m_setpointRad);
    SmartDashboard.putBoolean("Pivot/AtSetpoint", atSetpoint());
  }

  public void setAngle(double angleRad) {
    m_setpointRad = MathUtil.clamp(angleRad, kMinAngleRad, kMaxAngleRad);
    m_controller.setSetpoint(m_setpointRad, ControlType.kPosition);
  }

  public double getAngle() {
    return m_encoder.getPosition();
  }

  public boolean atSetpoint() {
    return Math.abs(getAngle() - m_setpointRad) < kAngleToleranceRad;
  }

  public void stop() {
    m_motor.stopMotor();
  }
}