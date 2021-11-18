package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import org.firstinspires.ftc.teamcode.Enums.LiftPosition;

public class FuerteFelipe {

    //Define Hardware Objects

    ///private ElapsedTime     runtime = new ElapsedTime();

    public DcMotor  linearActuator = null;

    // Need some features from the Linear Opmode to make the lift work

    LinearOpMode myOpMode;
    ElapsedTime runtime = new ElapsedTime();

    //Constants Lift
    public static final double      linearActuatorSPEED   =   0.6; // if this is too fast you cannot reset without hitting the framwe
    public static final int         linearActuatorDOWN        =   0;
    public static final int         linearActuatorPARTIAL     =   6;
    public static final int         linearActuatorLOW         =   4;
    public static final int         linearActuatorUP          =   4; //Number is in inches

    public static final int        TICKS_PER_LIFT_IN = 460; // determined experimentally 96 mm into inches = 3.73 = 360/3.73= 95
    private static final int        LIFT_HEIGHT_HIGH = (int) (linearActuatorUP * TICKS_PER_LIFT_IN); // converts to ticks

    LinearOpMode opmode;

    public FuerteFelipe(LinearOpMode opmode) {
        this.opmode = opmode;
    }


    public void init(HardwareMap hwMap)  {
        linearActuator = hwMap.get(DcMotor.class,"juanLift");
        linearActuator.setDirection(DcMotor.Direction.FORWARD);
        linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    //// Single operation methods - see below for methods to be called in Opmodes


    //Juan the lift's methods
    public void liftRise() {
        liftToTargetHeight(4,3);
    }


    // high goal is for the alliance hub so need LH and RH. The shared hub is only a low goal
    public void highGoalLeft() {
        //reset(); you will need to manually reset for noe. it messes up the flow
        liftToTargetHeight(linearActuatorPARTIAL, 3);
        liftToTargetHeight(linearActuatorUP, 3);

    }


    public void liftToTargetHeight(double height, double timeoutS){

        int newTargetHeight;


        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {

            // Determine new target lift height in ticks based on the current position.
            // When the match starts the current position should be reset to zero.

            newTargetHeight = (int)(height *  TICKS_PER_LIFT_IN);
            // Set the target now that is has been calculated
            linearActuator.setTargetPosition(newTargetHeight); //1000 ticks extends lift from 295mm to 530 mm which is 9.25 in per 1000 ticks or 108 ticks per in
            // Turn On RUN_TO_POSITION
            linearActuator.setPower(Math.abs(linearActuatorSPEED));
            // reset the timeout time and start motion.
            runtime.reset();
            linearActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // keep looping while we are still active, and there is time left, and thr motor is running.
            // Note: We use (isBusy() in the loop test, which means that when the motor hits
            // its target position, motion will stop.

            while (opmode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) && linearActuator.isBusy()) {

                // Display it for the driver.
                //  telemetry.addData("Moving to New Lift Height",  "Running to %7d", newTargetHeight);

                // telemetry.update();
            }

            // Stop all motion after exiting the while loop
            linearActuator.setPower(.25); // puts a low power to help hold the lift in place. There is a better way
            //liftPosition = LiftPosition.HOLD;
            // Turn off RUN_TO_POSITION
            //felipe.juanLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }


}




