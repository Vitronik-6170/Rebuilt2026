package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private static final int kFrontCanId = 3;
  private static final int kBackCanId  = 19;

  public static final double kRpmTolerance = 100.0;

  private final SparkMax m_shooterFront;
  private final SparkMax m_shooterBack;
  private final RelativeEncoder m_frontEncoder;
  private final RelativeEncoder m_backEncoder;
  private final SparkClosedLoopController m_frontController;
  private final SparkClosedLoopController m_backController;

  private double m_targetRpm = 0.0;

  public ShooterSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);
    config.inverted(false);
    config.closedLoop.pid(0.1, 0, 0);
    config.closedLoop.outputRange(-0.8, 0.8);

    m_shooterFront = new SparkMax(kFrontCanId, MotorType.kBrushless);
    m_shooterBack  = new SparkMax(kBackCanId,  MotorType.kBrushless);
    m_frontEncoder = m_shooterFront.getEncoder();
    m_backEncoder  = m_shooterBack.getEncoder();
    m_frontController = m_shooterFront.getClosedLoopController();
    m_backController  = m_shooterBack.getClosedLoopController();

    m_shooterFront.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_shooterBack.configure(config,  ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/FrontRPM",  m_frontEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter/BackRPM",   m_backEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter/TargetRPM", m_targetRpm);
    SmartDashboard.putBoolean("Shooter/AtSpeed",  atTargetRpm());
  }

  public void setRpm(double rpm) {
    m_targetRpm = rpm;
    m_frontController.setSetpoint(rpm, ControlType.kVelocity);
    m_backController.setSetpoint(rpm,  ControlType.kVelocity);
  }

  public boolean atTargetRpm() {
    boolean frontOk = Math.abs(m_frontEncoder.getVelocity() - m_targetRpm) < kRpmTolerance;
    boolean backOk  = Math.abs(m_backEncoder.getVelocity()  - m_targetRpm) < kRpmTolerance;
    return frontOk && backOk;
  }

  public void stop() {
    m_targetRpm = 0.0;
    m_shooterFront.stopMotor();
    m_shooterBack.stopMotor();
  }
}