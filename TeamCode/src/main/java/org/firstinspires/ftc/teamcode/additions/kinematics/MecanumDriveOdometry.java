package org.firstinspires.ftc.teamcode.additions.kinematics;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;

import java.util.function.DoubleSupplier;

public class MecanumDriveOdometry extends Odometry {
    private final MecanumDriveKinematics m_kinematics;
    private Pose2d m_poseMeters;
    private double m_prevTimeSeconds = -1;

    private Rotation2d m_gyroOffset;
    private Rotation2d m_previousAngle;

    /**
     * Constructs a MecanumDriveOdometry object.
     *
     * @param kinematics        The mecanum drive kinematics for your drivetrain.
     * @param gyroAngle         The angle reported by the gyroscope.
     * @param initialPoseMeters The starting position of the robot on the field.
     */
    public MecanumDriveOdometry(MecanumDriveKinematics kinematics, Rotation2d gyroAngle,
                                Pose2d initialPoseMeters) {
        super(initialPoseMeters);
        m_kinematics = kinematics;
        m_poseMeters = initialPoseMeters;
        m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
        m_previousAngle = initialPoseMeters.getRotation();
    }

    /**
     * Constructs a MecanumDriveOdometry object with the default pose at the origin.
     *
     * @param kinematics The mecanum drive kinematics for your drivetrain.
     * @param gyroAngle  The angle reported by the gyroscope.
     */
    public MecanumDriveOdometry(MecanumDriveKinematics kinematics, Rotation2d gyroAngle) {
        this(kinematics, gyroAngle, new Pose2d());
    }

    /**
     * Resets the robot's position on the field.
     *
     * <p>The gyroscope angle does not need to be reset here on the user's robot code.
     * The library automatically takes care of offsetting the gyro angle.
     *
     * @param poseMeters The position on the field that your robot is at.
     * @param gyroAngle  The angle reported by the gyroscope.
     */
    public void resetPosition(Pose2d poseMeters, Rotation2d gyroAngle) {
        m_poseMeters = poseMeters;
        m_previousAngle = poseMeters.getRotation();
        m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
    }

    /**
     * Returns the position of the robot on the field.
     *
     * @return The pose of the robot (x and y are in meters).
     */
    public Pose2d getPoseMeters() {
        return m_poseMeters;
    }

    /**
     * Updates the robot's position on the field using forward kinematics and
     * integration of the pose over time. This method takes in the current time as
     * a parameter to calculate period (difference between two timestamps). The
     * period is used to calculate the change in distance from a velocity. This
     * also takes in an angle parameter which is used instead of the
     * angular rate that is calculated from forward kinematics.
     *
     * @param currentTimeSeconds The current time in seconds.
     * @param gyroAngle          The angle reported by the gyroscope.
     * @param wheelSpeeds        The current wheel speeds.
     * @return The new pose of the robot.
     */
    public Pose2d updateWithTime(double currentTimeSeconds, Rotation2d gyroAngle,
                                 MecanumDriveWheelSpeeds wheelSpeeds) {
        double period = m_prevTimeSeconds >= 0 ? currentTimeSeconds - m_prevTimeSeconds : 0.0;
        m_prevTimeSeconds = currentTimeSeconds;

        Rotation2d angle = gyroAngle.plus(m_gyroOffset);

        ChassisSpeeds chassisState = m_kinematics.toChassisSpeeds(wheelSpeeds);
        Pose2d newPose = m_poseMeters.exp(
                new Twist2d(chassisState.vxMetersPerSecond * period,
                        chassisState.vyMetersPerSecond * period,
                        angle.minus(m_previousAngle).getRadians()));

        m_previousAngle = angle;
        m_poseMeters = new Pose2d(newPose.getTranslation(), angle);
        return m_poseMeters;
    }

    @Override
    public void updatePose(Pose2d newPose) {
        m_poseMeters = newPose;
    }

    //Do not use
    @Override
    public void updatePose() {
        m_poseMeters = new Pose2d();
    }

    @Override
    public Pose2d getPose() {
        return getPoseMeters();
    }

}