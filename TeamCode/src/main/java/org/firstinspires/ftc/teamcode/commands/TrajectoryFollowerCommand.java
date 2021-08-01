package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.additions.HolonomicDriveController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.Arrays;
import java.util.List;
import java.util.TreeMap;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANGULAR_VELOCITY;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VELOCITY;

public class TrajectoryFollowerCommand extends CommandGroupBase {

    private DriveSubsystem m_driveSubsystem;
    private HolonomicDriveController m_driveController;
    private Trajectory m_trajectory;
    private ElapsedTime m_time;
    private SequentialCommandGroup m_sequentialCommandGroup;
    TreeMap<Double, List<Command>> commandMap;

    private boolean isStarted;

    public TrajectoryFollowerCommand(DriveSubsystem driveSubsystem,
                                     HolonomicDriveController holonomicDriveController,
                                     Trajectory trajectory) {
        m_driveSubsystem = driveSubsystem;
        m_driveController = holonomicDriveController;
        m_trajectory = trajectory;
        m_time = new ElapsedTime();
        m_sequentialCommandGroup = new SequentialCommandGroup();

        addRequirements(driveSubsystem);
    }

    public void scheduleCommandAt(double time, Command... commands) {
        commandMap = new TreeMap<>();
        commandMap.put(time, Arrays.asList(commands.clone()));
    }

    @Override
    public void initialize() {
        isStarted = false;
    }

    @Override
    public void execute() {
        double time;
        if (!isStarted) {
            m_time.reset();
            isStarted = true;
            time = 0;
        } else time = m_time.seconds();

        if (!commandMap.isEmpty()) {
            double smallestKey = commandMap.firstKey();
            if (smallestKey <= time) {
                List<Command> commandsToSchedule = commandMap.remove(smallestKey);
                for (Command command : commandsToSchedule)
                    command.schedule();
            }
        }

        Trajectory.State currentSample = m_trajectory.sample(time);
        Pose2d currentPose = m_driveSubsystem.getPose();
        ChassisSpeeds speeds = m_driveController.calculate(currentPose, currentSample, currentPose.getRotation());
        Translation2d robotTrans = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).div(MAX_VELOCITY);
        double turnSpeed = speeds.omegaRadiansPerSecond / MAX_ANGULAR_VELOCITY;
        m_driveSubsystem.drive(-robotTrans.getY(), robotTrans.getX(), turnSpeed);
    }

    @Override
    public boolean isFinished() {
        return isStarted && m_time.seconds() >= m_trajectory.getTotalTimeSeconds();
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.stop();
        m_sequentialCommandGroup.schedule();
    }

    @Override
    public void addCommands(Command... commands) {
        m_sequentialCommandGroup.addCommands(commands);
    }

}
