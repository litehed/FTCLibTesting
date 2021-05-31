package org.firstinspires.ftc.teamcode.FTCLibTesting.trajectories;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;

import java.util.ArrayList;

public class TestTrajectory {
    public static Trajectory generateTrajectory() {

        // 2018 cross scale auto waypoints.
        Pose2d sideStart = new Pose2d(11.0, 12.0,
                Rotation2d.fromDegrees(-180));
        Pose2d crossScale = new Pose2d(12.0, 11.5,
                Rotation2d.fromDegrees(-160));

        ArrayList interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(11.5, 12.0));
        interiorWaypoints.add(new Translation2d(11.5, 11.5));

        TrajectoryConfig config = new TrajectoryConfig(1.5, 1.5);
        config.setReversed(true);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                sideStart,
                interiorWaypoints,
                crossScale,
                config);
        return trajectory;
    }
}
