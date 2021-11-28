package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.LiftPosition;
import org.firstinspires.ftc.teamcode.Enums.PatrickState;
import org.firstinspires.ftc.teamcode.Subsystems.CarouselTurnerThingy;
import org.firstinspires.ftc.teamcode.Subsystems.Felipe;
import org.firstinspires.ftc.teamcode.Subsystems.FuerteFelipe;
import org.firstinspires.ftc.teamcode.Subsystems.NewFelipe;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.Enums.Alliance.BLUE;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "Test")
//@Disabled
public class NewFelipeTest extends LinearOpMode {

    NewFelipe felipe = new NewFelipe(this);
    FuerteFelipe fuerteFelipe = new FuerteFelipe(this);
    ElapsedTime runtime = new ElapsedTime();

    LiftPosition liftPosition = LiftPosition.LOAD;



    @Override
    public void runOpMode() throws InterruptedException {
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); // this has to be here inside the runopmode. The others go above as class variables
        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // initialize the other subsystems
        felipe.init(hardwareMap);
        fuerteFelipe.init(hardwareMap);


        ////////////////////////////////////////////////////////////////////////////////////////////
        // WAIT FOR MATCH TO START
        ///////////////////////////////////////////////////////////////////////////////////////////
        waitForStart();
        fuerteFelipe.liftLoad();
        while (!isStopRequested()) {

            // Gampepad 1 Functions

            /**
             *
             * Gamepad #1 Buttons -
             *
             **/

            if (gamepad1.x) {
              felipe.julioLeft90();


                }
            if (gamepad1.y) {
                felipe.julioRight90();
            }

            if (gamepad1.a) {
                felipe.julioCenter();
            }
            if (gamepad1.b) {


            }
            /**
             *
             * Gamepad #1 Bumpers
             *
             **/

        
            if (gamepad1.left_bumper) {


            }
            if (gamepad1.left_bumper ) {


            }


            if (gamepad1.right_bumper) {


            }

            if (gamepad1.right_bumper) {


            }

            //right bumper once to turn intake lifon, right bumper to collect, left bumper to eject, left bumper again
            /**
             *
             * Gamepad #1 DPAD Julio COntrols
             *
             **/

            if (gamepad1.dpad_left) { //does not work

            }

            if (gamepad1.dpad_up){
                fuerteFelipe.liftRise();
            }
            if (gamepad1.dpad_right) {
                fuerteFelipe.liftPartial();
            }
            if (gamepad1.dpad_down) {//this one works
                fuerteFelipe.liftLoad();

            }
            /**
             *
             * Gamepad #1 Triggers - Homie Controls
             *
             **/

            if (gamepad1.left_trigger > 0.25) { //no purpose


                //debounce(400);
            }
            if (gamepad1.right_trigger > 0.25) { //no purpose

            }

            //Complex method buttons
            /**
             *
             * Gamepad #2  - Complex Methods             *
             **/
            if (gamepad2.a) {

            }

            if (gamepad2.dpad_left) {

            }

            if (gamepad2.dpad_right) {

            }

            if (gamepad2.dpad_up) {

            }

            if (gamepad2.dpad_down) {

            }

            // Carosel Functions
            if (gamepad2.x) {

            }
            if (gamepad2.b) {

            }
            if (gamepad2.y) {

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
