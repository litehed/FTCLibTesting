package org.firstinspires.ftc.teamcode.FTCLibTesting.trajectories;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.additions.HolonomicDriveController;

public class MecanumTrajectoryFollower {

    private MecanumDrive m_mecanumDrive;
    private Odometry m_odom;
    private HolonomicDriveController m_driveController;
    private ElapsedTime elapsedTime;

    private final double MAX_ANGULAR_VELOCITY;
    private final double MAX_VELOCITY;

    public MecanumTrajectoryFollower(MecanumDrive mecanumDrive, Odometry odometry,
                                     HolonomicDriveController holonomicDriveController) {
        m_mecanumDrive = mecanumDrive;
        m_odom = odometry;
        m_driveController = holonomicDriveController;
        MAX_ANGULAR_VELOCITY = Math.toRadians(180.0);
        MAX_VELOCITY = 1.5;
    }

    //Jackson is a sussy baka
    public void followTrajectory(Trajectory trajectory) {
        double t = 0.0;
        elapsedTime.reset();
        while (t < trajectory.getTotalTimeSeconds()) {
            Trajectory.State currentSample = trajectory.sample(t);
            Pose2d currentPose = m_odom.getPose();
            ChassisSpeeds speeds = m_driveController.calculate(currentPose, currentSample, currentPose.getRotation());
            Translation2d robotTrans = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).div(MAX_VELOCITY);
            double turnSpeed = speeds.omegaRadiansPerSecond / MAX_ANGULAR_VELOCITY;
            m_mecanumDrive.driveRobotCentric(robotTrans.getY(), robotTrans.getX(), turnSpeed);
            m_odom.updatePose();
            t = elapsedTime.seconds();
        }
        m_mecanumDrive.stop();
    }
}
