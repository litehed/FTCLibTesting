package org.firstinspires.ftc.teamcode.additions.kinematics;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import com.arcrobotics.ftclib.kinematics.Odometry;

public class DifferentialDriveOdometry extends Odometry {
    private Pose2d m_poseMeters;

    private Rotation2d m_gyroOffset;
    private Rotation2d m_previousAngle;

    private double m_prevLeftDistance;
    private double m_prevRightDistance;

    /**
     * Constructs a DifferentialDriveOdometry object.
     *
     * @param gyroAngle         The angle reported by the gyroscope.
     * @param initialPoseMeters The starting position of the robot on the field.
     */
    public DifferentialDriveOdometry(Rotation2d gyroAngle,
                                     Pose2d initialPoseMeters) {
        super(initialPoseMeters);
        m_poseMeters = initialPoseMeters;
        m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
        m_previousAngle = initialPoseMeters.getRotation();
    }

    /**
     * Constructs a DifferentialDriveOdometry object with the default pose at the origin.
     *
     * @param gyroAngle The angle reported by the gyroscope.
     */
    public DifferentialDriveOdometry(Rotation2d gyroAngle) {
        this(gyroAngle, new Pose2d());
    }

    /**
     * Resets the robot's position on the field.
     *
     * <p>You NEED to reset your encoders (to zero) when calling this method.
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

        m_prevLeftDistance = 0.0;
        m_prevRightDistance = 0.0;
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
     * Updates the robot position on the field using distance measurements from encoders. This
     * method is more numerically accurate than using velocities to integrate the pose and
     * is also advantageous for teams that are using lower CPR encoders.
     *
     * @param gyroAngle           The angle reported by the gyroscope.
     * @param leftDistanceMeters  The distance traveled by the left encoder.
     * @param rightDistanceMeters The distance traveled by the right encoder.
     * @return The new pose of the robot.
     */
    public Pose2d update(Rotation2d gyroAngle, double leftDistanceMeters,
                         double rightDistanceMeters) {
        double deltaLeftDistance = leftDistanceMeters - m_prevLeftDistance;
        double deltaRightDistance = rightDistanceMeters - m_prevRightDistance;

        m_prevLeftDistance = leftDistanceMeters;
        m_prevRightDistance = rightDistanceMeters;

        double averageDeltaDistance = (deltaLeftDistance + deltaRightDistance) / 2.0;
        Rotation2d angle = gyroAngle.plus(m_gyroOffset);

        Pose2d newPose = m_poseMeters.exp(
                new Twist2d(averageDeltaDistance, 0.0, angle.minus(m_previousAngle).getRadians()));

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