package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.JulioPosition;
import org.firstinspires.ftc.teamcode.Enums.LiftPosition;
import org.firstinspires.ftc.teamcode.Subsystems.FuerteFelipe;
import org.firstinspires.ftc.teamcode.Subsystems.NewFelipe;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "Test")
//@Disabled
public class StateMachineFelipe extends LinearOpMode {

    NewFelipe felipe = new NewFelipe(this);
    FuerteFelipe fuerteFelipe = new FuerteFelipe(this);
    ElapsedTime runtime = new ElapsedTime();
    LiftPosition liftPosition = LiftPosition.DOWN; // states for Juan the lift
    JulioPosition julioPosition = JulioPosition.CENTER; // states for Julio the Arm



    @Override
    public void runOpMode() throws InterruptedException {

        // initialize the other subsystems
        felipe.init(hardwareMap);
        fuerteFelipe.init(hardwareMap);
        // need to get out of Encoder Mode for this to work. Maybe put this in the function?
        fuerteFelipe.linearActuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fuerteFelipe.juanMechanicalReset();

       ////////////////////////////////////////////////////////////////////////////////////////////
        // WAIT FOR MATCH TO START
        ///////////////////////////////////////////////////////////////////////////////////////////
        waitForStart();
        fuerteFelipe.liftLoad();// put here becase opmode is acitve is a condition in the method that does this

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



            //right bumper once to turn intake lifon, right bumper to collect, left bumper to eject, left bumper again
            /**
             *
             * Gamepad #1 DPAD Julio COntrols
             *
             **/

            if (gamepad1.dpad_up){
                liftPosition = LiftPosition.UP;
                julioPosition = JulioPosition.CENTER;
                telemetry.addData("Lift State",  liftPosition);
                telemetry.addData("Arm State",  julioPosition);
                fuerteFelipe.liftRise();
            }
            if (gamepad1.dpad_right) {
                liftPosition = LiftPosition.PARTIAL;
                julioPosition = JulioPosition.RIGHT90;

            }

            if (gamepad1.dpad_left) {
                liftPosition = LiftPosition.PARTIAL;
                julioPosition = JulioPosition.LEFT90;
                fuerteFelipe.getJuanPosition();
                telemetry.addData("Lift State",  liftPosition);
                telemetry.addData("Arm State",  julioPosition);
                telemetry.addData("Juan Start Position",  fuerteFelipe.getJuanPosition());
                telemetry.update();
                fuerteFelipe.liftPartial();
                felipe.julioLeft90();
            }
            if (gamepad1.dpad_down) {//this one works
                liftPosition = LiftPosition.DOWN;
                julioPosition = JulioPosition.CENTER;
                fuerteFelipe.getJuanPosition();
                telemetry.addData("Lift State",  liftPosition);
                telemetry.addData("Arm State",  julioPosition);
                telemetry.addData("Juan Start Position",  fuerteFelipe.getJuanPosition());
                telemetry.update();
                felipe.julioCenter();
                fuerteFelipe.liftLoad();

            }
            // STATE HANDLING to ALLOW FOR SILMULTANEOUS MOTIONS
            if  (liftPosition == LiftPosition.PARTIAL && julioPosition == JulioPosition.RIGHT90) {// where we need to go
                telemetry.addData("Going to Lift PARTIAL and Right 90 Degrees", "Done");
                // this is messy just make a function to call instead
                //this sets the target when the button is pressed as the new STATE is set
                fuerteFelipe.linearActuator.setTargetPosition( (int)(fuerteFelipe. linearActuatorPARTIAL *  fuerteFelipe.TICKS_PER_LIFT_IN));
                felipe.julioArm.setTargetPosition((int)(felipe. JULIOARMRIGHT  * felipe.TICKS_PER_DEGREE));
                if( fuerteFelipe.getJuanPosition() < fuerteFelipe.linearActuatorPARTIAL ){

                    fuerteFelipe.linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    fuerteFelipe.linearActuator.setPower(Math.abs(fuerteFelipe.linearActuatorSPEED));
                }
                if( fuerteFelipe.getJuanPosition() >= fuerteFelipe.linearActuatorPARTIAL ){

                    felipe.julioArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    felipe.julioArm.setPower(Math.abs(felipe.JULIOTURNSPEED));
                }

                telemetry.addData("Lift State",  liftPosition);
                telemetry.addData("Lift Position (inches)", fuerteFelipe.getJuanPosition());
                telemetry.update();


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
