package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {

    private final DriveSubsystem m_drive;
    private final String m_limelightName = "limelight"; // si le cambiaste el nombre en la GUI cámbialo aquí

    public LimelightSubsystem(DriveSubsystem drive) {
        m_drive = drive;
    }

    @Override
    public void periodic() {
        updateVisionPose();
    }
    
    private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
}

    private void updateVisionPose() {

        // Le mandamos el heading del NavX 2 a la Limelight cada ciclo
        // Esto es lo que necesita para calcular MT2 correctamente
        LimelightHelpers.SetRobotOrientation(
            m_limelightName,
            m_drive.getHeading(), // viene del NavX 2 en DriveSubsystem
            0, 0, 0, 0, 0        // rates los dejamos en 0, no son necesarios
        );

        // Obtenemos la pose de MT1 (más estable en tu caso)
        /*LimelightHelpers.PoseEstimate mt1 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue(m_limelightName);*/

        LimelightHelpers.PoseEstimate mt1 = isRedAlliance()
        ? LimelightHelpers.getBotPoseEstimate_wpiRed(m_limelightName)
        : LimelightHelpers.getBotPoseEstimate_wpiBlue(m_limelightName);

        // ── Filtros para no meter basura al estimador ──────────────────

        // Si no llegó nada
        if (mt1 == null) {
            SmartDashboard.putBoolean("Vision/Activa", false);
            return;
        }

        // Si no ve ningún tag
        if (mt1.tagCount == 0) {
            SmartDashboard.putBoolean("Vision/Activa", false);
            return;
        }

        // Si solo ve 1 tag y está muy lejos, no es confiable
        if (mt1.tagCount == 1 && mt1.avgTagDist > 3.5) {
            SmartDashboard.putBoolean("Vision/Activa", false);
            return;
        }

        // Si el robot está girando muy rápido, hay motion blur
        if (Math.abs(m_drive.getYawRateDegPerSec()) > 540) {
            SmartDashboard.putBoolean("Vision/Activa", false);
            return;
        }

        // ── Confianza dinámica según distancia y cantidad de tags ──────
        // Más lejos o menos tags = std dev más alto = menos confianza
        double xyStdDev = 0.3 * (mt1.avgTagDist * mt1.avgTagDist) / mt1.tagCount;

        // ── Mandamos la medición al PoseEstimator ──────────────────────
        m_drive.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds,
            VecBuilder.fill(xyStdDev, xyStdDev, 9999)
            // 9999 en rotación = el NavX 2 es más confiable que la visión para heading
        );

        // ── Debug en SmartDashboard ────────────────────────────────────
        SmartDashboard.putBoolean("Vision/Activa", true);
        SmartDashboard.putNumber("Vision/TagCount", mt1.tagCount);
        SmartDashboard.putNumber("Vision/AvgTagDist", mt1.avgTagDist);
        SmartDashboard.putNumber("Vision/XYStdDev", xyStdDev);
        SmartDashboard.putNumber("Vision/PoseX", mt1.pose.getX());
        SmartDashboard.putNumber("Vision/PoseY", mt1.pose.getY());
    }
}