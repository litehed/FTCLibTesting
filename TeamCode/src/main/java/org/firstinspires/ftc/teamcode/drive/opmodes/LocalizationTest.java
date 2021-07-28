package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.additions.HolonomicDriveController;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.CENTER_WHEEL_OFFSET;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.DISTANCE_PER_PULSE;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.HEADING_KD;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.HEADING_KI;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.HEADING_KP;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRANSLATION_KD;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRANSLATION_KI;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRANSLATION_KP;

@Config
@TeleOp
public class LocalizationTest extends LinearOpMode {


    private GamepadEx driverOp;

    private Motor.Encoder leftOdometer, rightOdometer, midOdometer;

    @Override
    public void runOpMode() throws InterruptedException {

        Motor frontLeft = new Motor(hardwareMap, "fL");
        Motor frontRight = new Motor(hardwareMap, "fR");
        Motor backLeft = new Motor(hardwareMap, "bL");
        Motor backRight = new Motor(hardwareMap, "bR");

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

        PIDController translationController = new PIDController(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD);
        ProfiledPIDController headingController = new ProfiledPIDController(HEADING_KP, HEADING_KI, HEADING_KD,
                new TrapezoidProfile.Constraints(Math.toRadians(180), Math.toRadians(180.0)));
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
                TRACK_WIDTH,
                CENTER_WHEEL_OFFSET
        );
        driverOp = new GamepadEx(gamepad1);

        backLeft.resetEncoder();
        frontRight.resetEncoder();
        frontLeft.resetEncoder();

        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            mecanumDrive.driveRobotCentric(-driverOp.getLeftX(), -driverOp.getLeftY(), -driverOp.getRightX());
            telemetry.addData("Robot Position: ", holonomicOdometry.getPose());
            telemetry.update();
            holonomicOdometry.updatePose();
        }
    }
}
