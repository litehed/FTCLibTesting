package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.Odometry;

import org.firstinspires.ftc.teamcode.additions.MecanumDrive;

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

    public void drive(double frontLeftSpeed, double frontRightSpeed,
                      double backLeftSpeed, double backRightSpeed) {
        if (m_drive instanceof MecanumDrive) {
            ((MecanumDrive) m_drive).driveWithMotorPowers(frontLeftSpeed, frontRightSpeed,
                    backLeftSpeed, backRightSpeed);
        }
    }

    public void stop() {
        m_drive.stop();
    }

    public Pose2d getPose() {
        return m_odom.getPose();
    }

}
