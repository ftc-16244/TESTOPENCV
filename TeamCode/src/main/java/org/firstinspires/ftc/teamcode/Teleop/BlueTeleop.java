package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Subsystems.CarouselTurnerThingy;
import org.firstinspires.ftc.teamcode.Subsystems.Felipe;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.Enums.Alliance.BLUE;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class BlueTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Felipe felipe = new Felipe(); // instantiate Felipe (the main implement)
        CarouselTurnerThingy carousel = new CarouselTurnerThingy(hardwareMap, BLUE);
        // init and setup
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        felipe.init(hardwareMap);
        //carousel.init(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
