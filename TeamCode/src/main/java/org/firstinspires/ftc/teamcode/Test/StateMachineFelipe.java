package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.JulioPosition;
import org.firstinspires.ftc.teamcode.Enums.LiftPosition;
import org.firstinspires.ftc.teamcode.Subsystems.FelipeDeux;


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

    FelipeDeux felipe = new FelipeDeux(this);
    ElapsedTime runtime = new ElapsedTime();
    LiftPosition liftPosition = LiftPosition.UNKNOWN; // don't know where it is until initi
    JulioPosition julioPosition = JulioPosition.CENTER; // states for Julio the Arm


    @Override
    public void runOpMode() throws InterruptedException {

        // initialize subsystems
        felipe.init(hardwareMap);
        // felipe.linearActuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); redundant code
        felipe.juanMechanicalReset();

        waitForStart();

        felipe.liftLoad();// put here becase opmode is acitve is a condition in the method that does this
        felipe.armInit();
        felipe.thumbClose();

        while (!isStopRequested()) {

            /**
             Gamepad #1 Buttons
             **/

            if (gamepad1.x) {
              felipe.julioLeft90();
                }

            if (gamepad1.y) {
                felipe.liftPartial();
                felipe.julioRight90();
            }

            if (gamepad1.a) {
                felipe.julioCenter();
            }

            if (gamepad1.b) {
                felipe.thumbOpen();
            }


            //right bumper once to turn intake lifon, right bumper to collect, left bumper to eject, left bumper again

            /**
             Gamepad #1 DPAD Julio Controls
             **/

            //reset function, if dpad down is pressed sets the state of lift and julio to down and center
            if (gamepad1.dpad_down) {//this one works
                julioPosition = JulioPosition.CENTER;
            }

            //if the dpad down is pressed, the code below gets executed
            if  (liftPosition == LiftPosition.LOAD && julioPosition == JulioPosition.CENTER) {// where we need to go
                telemetry.addData("Going to Lift PARTIAL and Right 90 Degrees", "Done");

                //this sets the target when the button is pressed as the new STATE is set
                felipe.linearActuator.setTargetPosition( (int)(felipe.JUANLIFTPARTIAL *  felipe.TICKS_PER_LIFT_IN));
                felipe.julioArm.setTargetPosition((int)(felipe. JULIOARMCENTER  * felipe.TICKS_PER_DEGREE));
<<<<<<< HEAD
                if(felipe.getJuanPosition() < felipe.JUANLIFTLOAD){

=======
                if( felipe.getJuanPosition() < felipe.JUANLIFTPARTIAL){

                    felipe.linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    felipe.linearActuator.setPower(Math.abs(felipe.JUANLIFTSPEED));
                }
                felipe.linearActuator.setTargetPosition( (int)(felipe.JUANLIFTLOAD *  felipe.TICKS_PER_LIFT_IN));
                if( felipe.getJuanPosition() < felipe.JUANLIFTLOAD){
>>>>>>> aaf4f868d64bffc52c08eeb1147a26a468a9d2d2

                    felipe.linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    felipe.linearActuator.setPower(Math.abs(felipe.JUANLIFTSPEED));
                }
                //only after linear actuator reaches target height does the julio start moving
<<<<<<< HEAD
                if( felipe.getJuanPosition() >= felipe.JUANLIFTPARTIAL ){

                if( felipe.getJuanPosition() < felipe.JUANLIFTPARTIAL ){


                    felipe.linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    felipe.linearActuator.setPower(Math.abs(felipe.JUANLIFTSPEED));
                }
                //only after linear actuator reaches target height does the julio start moving

                if( felipe.getJuanPosition() >= felipe.JUANLIFTPARTIAL ){


                    felipe.julioArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    felipe.julioArm.setPower(Math.abs(felipe.JULIOTURNSPEED));
                }

                telemetry.addData("Lift State",  liftPosition);
                telemetry.addData("Lift Position (inches)", felipe.getJuanPosition());
                telemetry.update();
            }

            //------------------------------------------------------------------------------------

            //when you press dpad left, it changes the state of lift and julio to partial and left90
            if (gamepad1.dpad_left) {
                liftPosition = LiftPosition.PARTIAL;
                julioPosition = JulioPosition.LEFT90;
            }

            //if the dpad left is pressed, the code below gets executed
            if  (liftPosition == LiftPosition.PARTIAL && julioPosition == JulioPosition.LEFT90) {// where we need to go
                telemetry.addData("Going to Lift PARTIAL and Right 90 Degrees", "Done");

                //this sets the target when the button is pressed as the new STATE is set
                felipe.linearActuator.setTargetPosition( (int)(felipe.JUANLIFTPARTIAL *  felipe.TICKS_PER_LIFT_IN));
                felipe.julioArm.setTargetPosition((int)(felipe. JULIOARMLEFT  * felipe.TICKS_PER_DEGREE));
                if( felipe.getJuanPosition() < felipe.JUANLIFTPARTIAL ){

                    felipe.linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    felipe.linearActuator.setPower(Math.abs(felipe.JUANLIFTSPEED));
                }
                //only after linear actuator reaches target height does the julio start moving
<<<<<<< HEAD
                if( felipe.getJuanPosition() >= felipe.JUANLIFTPARTIAL){
=======
                if( felipe.getJuanPosition() >= felipe.JUANLIFTPARTIAL ){
>>>>>>> aaf4f868d64bffc52c08eeb1147a26a468a9d2d2

                    felipe.julioArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    felipe.julioArm.setPower(Math.abs(felipe.JULIOTURNSPEED));
                }

                telemetry.addData("Lift State",  liftPosition);
                telemetry.addData("Lift Position (inches)", felipe.getJuanPosition());
                telemetry.update();
            }


            /**
             Gamepad #1 Triggers - Homie Controls
             **/

            if (gamepad1.left_trigger > 0.25) {
                //no purpose
            }

            if (gamepad1.right_trigger > 0.25) {
                //no purpose
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
