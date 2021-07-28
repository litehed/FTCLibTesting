package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.Odometry;

public class DriveSubsystem extends SubsystemBase {

    private RobotDrive m_drive;
    private Odometry m_odom;

    public DriveSubsystem(RobotDrive drive, Odometry odometry) {
        m_drive = drive;
        m_odom = odometry;
    }

    @Override
    public void periodic() {
        m_odom.updatePose();
    }

    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        if (m_drive instanceof MecanumDrive) {
            ((MecanumDrive) m_drive).driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
        } else if (m_drive instanceof DifferentialDrive) {
            ((DifferentialDrive) m_drive).arcadeDrive(forwardSpeed, turnSpeed);
        } else throw new IllegalArgumentException("Unsupported drive type.");
    }

    public void stop() {
        m_drive.stop();
    }

    public Pose2d getPose() {
        return m_odom.getPose();
    }

}
