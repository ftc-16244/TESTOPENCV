package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class NewFelipe {

    public DcMotor julioArm = null;


    public static final double      JULIOARMLEFT          = 90.0;
    public static final double      JULIOARMCENTER         = 0.0;
    public static final double      JULIOARMRIGHT        = -90.0;
    public static final double      JULIOTURNSPEED       =   0.3;
    public static final int         TICKS_PER_REV         = 500;
    public static final int         TICKS_PER_DEGREE = TICKS_PER_REV/360;
    ElapsedTime runtime = new ElapsedTime();

    LinearOpMode opmode;

    public NewFelipe(LinearOpMode opmode) {
        this.opmode = opmode;
    }

    public void init(HardwareMap hwMap)  {
        julioArm = hwMap.get(DcMotor.class,"julioArm");

        julioArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        julioArm.setDirection(DcMotor.Direction.FORWARD);
        julioArm.setDirection(DcMotor.Direction.FORWARD);
        julioArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }


    public void julioLeft90(){
        rotateToTargetAngle(JULIOARMLEFT,2);
    }

    public void julioRight90(){
        rotateToTargetAngle(JULIOARMRIGHT  ,2);
    }

    public void julioCenter(){
        rotateToTargetAngle(JULIOARMCENTER  ,2);
        julioArm.setPower(Math.abs(0)); // cut power to motor when it is at the center to it can be guided into the frame
    }

    public void rotateToTargetAngle(double angle, double timeoutS){

        int newTargetAngle; // ok to leave an an int unless we want really fine tuning


        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {

            // Determine new target lift height in ticks based on the current position.
            // When the match starts the current position should be reset to zero.

            newTargetAngle = (int)(angle *  TICKS_PER_DEGREE);
            // Set the target now that is has been calculated
            julioArm.setTargetPosition(newTargetAngle); //
            // Turn On RUN_TO_POSITION
            julioArm.setPower(Math.abs( JULIOTURNSPEED ));
            // reset the timeout time and start motion.
            runtime.reset();
            julioArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // keep looping while we are still active, and there is time left, and thr motor is running.
            // Note: We use (isBusy() in the loop test, which means that when the motor hits
            // its target position, motion will stop.

            while (opmode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) && julioArm.isBusy()) {

                // Display it for the driver.
                //  telemetry.addData("Moving to New Lift Height",  "Running to %7d", newTargetHeight);

                // telemetry.update();
            }

        }
    }
    }

