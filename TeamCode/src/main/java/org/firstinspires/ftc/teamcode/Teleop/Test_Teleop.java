package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.DrivetrainRR;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

@TeleOp(name="GunShow Teleop", group="Teleop")
//@Disabled
public class Test_Teleop extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Instantiate subsytems components here
        DrivetrainRR drivetrain = new DrivetrainRR(hardwareMap); // this is the roadrunner derivative of Simple Mechanum Drive we copied
        StandardTrackingWheelLocalizer localizer = new StandardTrackingWheelLocalizer(hardwareMap);      // Add servos or other motors here as needed.


        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double lf, lr, rf, rr;

            lf = (y + x + rx); // forward + turn right + strafe right
            lr = (y - x + rx);
            rf = (y - x - rx);
            rr = (y + x - rx);

            if (Math.abs(lf) > 1 || Math.abs(lr) > 1 || Math.abs(rf) > 1 || Math.abs(rr) > 1) {
                double max = 0;
                max = Math.max(Math.abs(lf), Math.abs(lr));
                max = Math.max(Math.abs(rf), max);
                max = Math.max(Math.abs(rr), max);

                // scales output if y + x + rx >1
                lf /= max;
                lr /= max;
                rf /= max;
                rr /= max;

            }


            drivetrain. leftFront.setPower(lf);
            drivetrain.leftRear.setPower(lr);
            drivetrain.rightFront.setPower(rf);
            drivetrain.rightRear.setPower(rr);

           //telemetry.addData("Left Stick Y-Fwd", "%5.2f", y);
            // telemetry.addData("Right  Stick X-Turn", "%5.2f", x);
            //telemetry.addData("Left Stick RX-Rotation", "%5.2f", rx);
            //telemetry.update();


            Pose2d poseEstimate = drivetrain.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.update();


        }
    }

}