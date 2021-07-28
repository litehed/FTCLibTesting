package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DriveConstants {

    public static double TRACK_WIDTH = 0.38227;
    public static double CENTER_WHEEL_OFFSET = -0.172212;
    public static double WHEEL_DIAMETER = 0.0350;
    public static double TICKS_PER_REV = 8192;
    public static double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    public static double HEADING_KP = 1.0, HEADING_KI = 0.0, HEADING_KD = 0.0;
    public static double TRANSLATION_KP = 1.0, TRANSLATION_KI = 0.0, TRANSLATION_KD = 0.0;

    public static double MAX_ANGULAR_VELOCITY = Math.toRadians(180.0);
    public static double MAX_VELOCITY = 1.5;

}
