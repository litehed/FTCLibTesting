package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FTCLibTesting.trajectories.MecanumTrajectoryFollower;
import org.firstinspires.ftc.teamcode.additions.HolonomicDriveController;

@TeleOp
public class LocalizationTest extends LinearOpMode {

    public static double TRACKWIDTH = 0.38227;
    public static double CENTER_WHEEL_OFFSET = -0.172212;
    public static double WHEEL_DIAMETER = (0.688976 * 2)/39.37;
    public static double TICKS_PER_REV = 8192;
    public static double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

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

        PIDController translationController = new PIDController(1, 0, 0);
        ProfiledPIDController headingController = new ProfiledPIDController(1, 0, 0,
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
                TRACKWIDTH,
                CENTER_WHEEL_OFFSET
        );
        MecanumTrajectoryFollower trajectoryFollower = new MecanumTrajectoryFollower(
                mecanumDrive,
                holonomicOdometry,
                holonomicDriveController
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
