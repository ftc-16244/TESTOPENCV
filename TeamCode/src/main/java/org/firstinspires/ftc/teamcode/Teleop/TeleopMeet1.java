package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.Enums.Alliance.BLUE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.LiftPosition;
import org.firstinspires.ftc.teamcode.Enums.PatrickState;
import org.firstinspires.ftc.teamcode.Subsystems.CarouselTurnerThingy;
import org.firstinspires.ftc.teamcode.Subsystems.Felipe;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "Test")
public class TeleopMeet1 extends LinearOpMode {

    Felipe felipe = new Felipe(this);
    //Felipe felipe = new Felipe(); // instantiate Felipe (the main implement)
    CarouselTurnerThingy carousel = new CarouselTurnerThingy();
    // init and setup
    ElapsedTime runtime = new ElapsedTime();



    // ENUMS
    //DriveSpeedState  currDriveState;
    PatrickState patrickState = PatrickState.OFF;
    LiftPosition liftPosition = LiftPosition.DOWN;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); // this has to be here inside the runopmode. The others go above as class variables
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // initialize the other subsystems
        felipe.init(hardwareMap);
        carousel.init(hardwareMap,  BLUE);

        ////////////////////////////////////////////////////////////////////////////////////////////
        // WAIT FOR MATCH TO START
        ///////////////////////////////////////////////////////////////////////////////////////////
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

            // Gampepad 1 Functions

            /**
             *
             * Gamepad #1 Buttons -
             *
             **/


            /**
             *
             * Gamepad #1 Bumpers
             *
             **/

        
            if (gamepad1.left_bumper && patrickState == PatrickState.OFF) {
                felipe.intakeOn();
                felipe.homieCenter();
                patrickState = PatrickState.COLLECT;
                telemetry.addData("Collector State", patrickState);
                debounce(175); // need to pause for a few ms to let drive release the button

            }
            if (gamepad1.left_bumper && patrickState == PatrickState.COLLECT) {

                felipe.intakeOff();
                patrickState = PatrickState.OFF;
                telemetry.addData("Collector State", patrickState);
                debounce(175);
            }


            if (gamepad1.right_bumper && patrickState == PatrickState.OFF) {

                felipe.intakeEject();
                felipe.homieCenter();
                patrickState = PatrickState.EJECT;

                telemetry.addData("Collector State", patrickState);
                debounce(175);

            }

            if (gamepad1.right_bumper && patrickState == PatrickState.EJECT) {
                felipe.intakeOff();
                patrickState = PatrickState.OFF;
                telemetry.addData("Collector State",patrickState);
                debounce(175);

            }


            /**
             *
             * Gamepad #1 DPAD Julio COntrols
             *
             **/


            /**
             *
             * Gamepad #1 Triggers - Homie Controls
             *
             **/

            if (gamepad1.left_trigger > 0.25) {
                felipe.homieLeft();
                //debounce(400);
                telemetry.addData("Homie Left", "Complete ");

                //debounce(400);
            }
            if (gamepad1.right_trigger > 0.25) {
                felipe.homieRight();
                telemetry.addData("Homie Right", "Complete ");
            }

            //Complex method buttons
            /**
             *
             * Gamepad #2  - Complex Methods             *
             **/
            if (gamepad2.a) {
                felipe.reset();
                telemetry.addData("Reset", "Complete ");
            }

            if (gamepad2.dpad_left) {
                felipe.highGoalLeft();
                telemetry.addData("High Goal", "Complete ");
            }

            if (gamepad2.dpad_right) {
                felipe.highGoalRight();;
                telemetry.addData("High Goal", "Complete ");
            }

            if (gamepad2.dpad_down) {
                felipe.liftToTargetHeight(Felipe.JUANLIFTDOWN, 3);
                telemetry.addData("lower elevator", "Complete ");
            }


            // Carosel Functions
            if (gamepad2.x) {
                carousel.carouselTurnCCW();
                telemetry.addData("Turning CCW", "Complete ");
            }
            if (gamepad2.b) {
                carousel.carouselTurnCW();
                telemetry.addData("Turning CW", "Complete ");
            }
            if (gamepad2.y) {
                carousel.carouselTurnOff();
                telemetry.addData("Turning Off", "Complete ");
            }


            telemetry.update();

        }


    }

    void debounce(long debounceTime){
        try {
            Thread.sleep(debounceTime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }


}
