package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.additions.MecanumDrive;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.TestTrajectory;
import org.firstinspires.ftc.teamcode.additions.HolonomicDriveController;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.CENTER_WHEEL_OFFSET;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.DISTANCE_PER_PULSE;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.HEADING_KD;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.HEADING_KI;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.HEADING_KP;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.LATERAL_DISTANCE;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRANSLATION_KD;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRANSLATION_KI;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRANSLATION_KP;

@Config
@Autonomous
public class TestOpmode extends CommandOpMode {


    private Motor.Encoder leftOdometer, rightOdometer, midOdometer;

    @Override
    public void initialize() {
        Motor frontLeft = new Motor(hardwareMap, "fL");
        Motor frontRight = new Motor(hardwareMap, "fR");
        Motor backLeft = new Motor(hardwareMap, "bL");
        Motor backRight = new Motor(hardwareMap, "bR");

        Motor flywheel = new Motor(hardwareMap, "shoot");

        backLeft.setInverted(true);
        frontLeft.setInverted(true);

        leftOdometer = frontRight.encoder;
        rightOdometer = backLeft.encoder;
        midOdometer = frontLeft.encoder;

        leftOdometer.setDirection(Motor.Direction.REVERSE);
        midOdometer.setDirection(Motor.Direction.REVERSE);

        leftOdometer.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdometer.setDistancePerPulse(DISTANCE_PER_PULSE);
        midOdometer.setDistancePerPulse(DISTANCE_PER_PULSE);

        leftOdometer.reset();
        rightOdometer.reset();
        midOdometer.reset();

        PIDController translationController = new PIDController(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD);
        ProfiledPIDController headingController = new ProfiledPIDController(HEADING_KP, HEADING_KI, HEADING_KD,
                new TrapezoidProfile.Constraints(DriveConstants.MAX_ANGULAR_VELOCITY, Math.toRadians(90.0)));
        HolonomicDriveController holonomicDriveController =
                new HolonomicDriveController(translationController, translationController, headingController);
        MecanumDrive mecanumDrive = new MecanumDrive(
                false,
                frontLeft,
                frontRight,
                backLeft,
                backRight
        );

        HolonomicOdometry holonomicOdometry = new HolonomicOdometry(
                frontRight::getDistance,
                backLeft::getDistance,
                frontLeft::getDistance,
                LATERAL_DISTANCE,
                CENTER_WHEEL_OFFSET
        );
        DriveSubsystem driveSubsystem = new DriveSubsystem(mecanumDrive, holonomicOdometry);
        Trajectory traj = TestTrajectory.generateTrajectory();
        TrajectoryFollowerCommand trajectoryFollowerCommand =
                new TrajectoryFollowerCommand(driveSubsystem, holonomicDriveController, traj, telemetry);
        trajectoryFollowerCommand.scheduleCommandAt(1.0, new InstantCommand(() -> flywheel.set(1.0)));
        trajectoryFollowerCommand.addCommands(new InstantCommand(() -> flywheel.stopMotor()));

        schedule(trajectoryFollowerCommand);
    }

}
