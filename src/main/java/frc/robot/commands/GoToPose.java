package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class GoToPose extends Command {

    private final Pose2d m_targetPose;
    private final DriveSubsystem m_drive;
    private Command m_pathfindingCommand;

    // Ajusta estas constraints a tu robot
    private static final PathConstraints kConstraints = new PathConstraints(
        2.0,  // maxVelocityMps
        2.0,  // maxAccelerationMpsSq
        Units.degreesToRadians(360),  // maxAngularVelocityRps
        Units.degreesToRadians(360)   // maxAngularAccelerationRpsSq
    );

    public GoToPose(Pose2d targetPose, DriveSubsystem drive) {
        m_targetPose = targetPose;
        m_drive = drive;
        // OJO: NO pongas addRequirements aquí
        // el comando interno de PathPlanner ya toma el subsistema
    }

    @Override
    public void initialize() {
        m_pathfindingCommand = AutoBuilder.pathfindToPose(m_targetPose, kConstraints);
        m_pathfindingCommand.initialize();
    }

    @Override
    public void execute() {
        m_pathfindingCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        m_pathfindingCommand.end(interrupted);
        // Frena el robot al terminar o si se interrumpe
        m_drive.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        return m_pathfindingCommand.isFinished();
    }
}