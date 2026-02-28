// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kMechanismControllerPort = 1;
  }
  public static class IntakeConstants {
    //INTAKE
    public static final int kIDintakeMotor = 15; //ID of the intake motor
    public static final boolean kIntakeMotorInverted = true; //inverted or not
    public static final double kIntakeMotorPower = 0.9; //Power

    //INTAKE EXTENSION
    public static final int kExtensionLimitInputChannel = 0; //limit 1
    public static final int kExtensionLimitOInputChannel = 1; //limit 2

    public static final int kIDextensionMotor = 16; //ID of the extension motor
    public static final boolean kExtensionMotorInverted = true; //inverted or not
    public static final double kPextensionMotor = 1; //kP
    public static final double kIextensionMotor = 0; //kI
    public static final double kDextensionMotor = 0; //kD
    public static final double kExtensionIZone = 0.0002; //iZone
    public static final double kExtensionMotorPower = 0.9; //Power

    public static final double kExtensionPositionRetracted = 5; // min position
    public static final double kExtensionPositionExtended = 120; // max position

  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 10;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = 0;// -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = 0;//Math.PI;
    public static final double kBackRightChassisAngularOffset = 0;//Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 5;
    public static final int kRearLeftDrivingCanId = 7;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 9;

    public static final int kFrontLeftTurningCanId = 4;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 2;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // SDS MK4n L2+ configuration with NEO V1.0/V1.1 motors
    // No configurable pinion like MAXSwerve - gear ratio is fixed by module version
    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60; // NEO = 5820 RPM
    public static final double kWheelDiameterMeters = 0.1016; // 4 inch wheel (MK4n uses 4", MAXSwerve uses 3")
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    // 3-stage gear reduction for MK4n L2+:
    // First stage:  16 driving / 50 driven
    // Second stage: 27 driving / 17 driven
    // Third stage:  15 driving / 45 driven
    public static final double kDrivingMotorReduction =
        (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0); // = 5.90:1

    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)/ kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.2;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  //Constantes de las posiciones de la cancha 
  public static final class FieldPositions {
    public static final Pose2d kPosition1 = new Pose2d(3.292, 7.220, Rotation2d.fromDegrees(113.869)); // Trinchera izquierda
    public static final Pose2d kPosition2 = new Pose2d( 3.292, 0.780, Rotation2d.fromDegrees(-113.869)); // Trinchera derecha
    public static final Pose2d kPosition3 = new Pose2d(2.0, 6.0, Rotation2d.fromDegrees(145.543)); // Cerca de la trinchera izquierda 
    public static final Pose2d kPosition4 = new Pose2d(2.0, 4, Rotation2d.fromDegrees(180)); // En medio
    public static final Pose2d kPosition5 = new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(-143.543)); // Cerca de la trinchera derecha
}

}