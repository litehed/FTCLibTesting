package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FTCLibTesting.trajectories.MecanumTrajectoryFollower;
import org.firstinspires.ftc.teamcode.FTCLibTesting.trajectories.TestTrajectory;
import org.firstinspires.ftc.teamcode.additions.HolonomicDriveController;

@Autonomous
public class TestOpmode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Motor frontLeft = new Motor(hardwareMap, "fL");
        Motor frontRight = new Motor(hardwareMap, "fR");
        Motor backLeft = new Motor(hardwareMap, "bL");
        Motor backRight = new Motor(hardwareMap, "bR");
        PIDController translationController = new PIDController(1, 0, 0);
        ProfiledPIDController headingController = new ProfiledPIDController(1, 0, 0,
                new TrapezoidProfile.Constraints(Math.toRadians(180), Math.toRadians(180.0)));
        HolonomicDriveController holonomicDriveController =
                new HolonomicDriveController(translationController, translationController, headingController);
        MecanumDrive mecanumDrive = new MecanumDrive(
                frontLeft,
                frontRight,
                backLeft,
                backRight
        );
        HolonomicOdometry holonomicOdometry = new HolonomicOdometry(
                backLeft::getCurrentPosition,
                backRight::getCurrentPosition,
                frontLeft::getCurrentPosition,
                15.05,
                -6.78
        );
        MecanumTrajectoryFollower trajectoryFollower = new MecanumTrajectoryFollower(
                mecanumDrive,
                holonomicOdometry,
                holonomicDriveController
        );
        Trajectory traj = TestTrajectory.generateTrajectory();
        telemetry.addData("Total Time Seconds", traj.getTotalTimeSeconds());
        telemetry.update();
        waitForStart();
        trajectoryFollower.followTrajectory(traj);

    }
}

