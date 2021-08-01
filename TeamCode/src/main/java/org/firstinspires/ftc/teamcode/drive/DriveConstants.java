package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;

@Config
public class DriveConstants {

    public static double LATERAL_DISTANCE = 0.38227;
    public static double CENTER_WHEEL_OFFSET = -0.172212;
    public static double WHEEL_DIAMETER = 0.0350;
    public static double TICKS_PER_REV = 8192;
    public static double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    public static double HEADING_KP = 1.0, HEADING_KI = 0.0, HEADING_KD = 0.0;
    public static double TRANSLATION_KP = 1.0, TRANSLATION_KI = 0.0, TRANSLATION_KD = 0.0;

    public static double MAX_ANGULAR_VELOCITY = Math.toRadians(180.0);
    public static double MAX_VELOCITY = 1.5;

    public static double TRACK_WIDTH = 0.44069;
    public static double WHEEL_BASE = 0.34036;

    public static final MecanumDriveKinematics DRIVE_KINEMATICS =
            new MecanumDriveKinematics(
                    new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                    new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                    new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                    new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
            );

}
