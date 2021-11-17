package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import org.firstinspires.ftc.teamcode.Enums.LiftPosition;

public class Felipe {

    //Define Hardware Objects

    ///private ElapsedTime     runtime = new ElapsedTime();

    public DcMotor  juanLift = null;
    public DcMotor  patrickIntake = null; // no longer used
    public Servo    julioPivot = null;
    public Servo    homieBox = null;

    // Need some features from the Linear Opmode to make the lift work

    LinearOpMode myOpMode;
    ElapsedTime runtime = new ElapsedTime();

    //Constants Lift
    public static final double      JUANLIFTSPEED       =   0.6; // if this is too fast you cannot reset without hitting the framwe
    public static final int         JUANLIFTDOWN        =   0;
    public static final int         JUANLIFTPARTIAL     =   6;
    public static final int         JUANLIFTLOW         =   4;
    public static final int         JUANLIFTUP          =   11; //Number is in inches

    public static final int        TICKS_PER_LIFT_IN = 80; // determined experimentally
    private static final int        LIFT_HEIGHT_HIGH = (int) (JUANLIFTUP * TICKS_PER_LIFT_IN); // converts to ticks

    //Constants for robot arm
    public static final double      JULIOPIVOTLEFT          = 0.1;
    public static final double      JULIOPIVOTRIGHT         = 0.8;
    public static final double      JULIOPIVOTCENTER        = 0.445;
    public static final double      JULIODELAY              = 0.5;

    //Constants for robot home box
    public static final double      HOMIEBOXPIVOTLEFT       = 1;
    public static final double      HOMIEBOXPIVOTRIGHT      = 0.3;
    public static final double      HOMIEBOXPIVOTCENTER     = 0.66;
    public static final double      HOMIEDELAY              = 0.15;

    //Constants for robot intake
    public static final double      PATRICKINTAKESLOW = .3;//use this while lifting juan
    public static final double      PATRIKINTAKECOFF = 0;
    public static final double      PATRICKINTAKEON = 0.7;

    LinearOpMode opmode;

    public Felipe(LinearOpMode opmode) {
        this.opmode = opmode;
    }


    public void init(HardwareMap hwMap)  {
        juanLift = hwMap.get(DcMotor.class,"juanLift");
        juanLift.setDirection(DcMotor.Direction.FORWARD);
        juanLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        juanLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        patrickIntake = hwMap.get(DcMotor.class,"patrickIntake");
        patrickIntake.setDirection(DcMotor.Direction.FORWARD);
        patrickIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        julioPivot = hwMap.get(Servo.class,"julioPivot");
        homieBox = hwMap.get(Servo.class,"homieBox");

        //Positive=up and Negative=down

        julioPivot.setPosition(JULIOPIVOTCENTER);
        homieBox.setPosition(HOMIEBOXPIVOTCENTER);
    }

    //// Single operation methods - see below for methods to be called in Opmodes


    //Juan the lift's methods
    public void liftRise() {
        juanLift.setTargetPosition(JUANLIFTUP);// value is in ticks from above calculation
        juanLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        juanLift.setPower(JUANLIFTSPEED);
        juanLift.setZeroPowerBehavior(BRAKE);
    }
    public void liftLower() {
        juanLift.setTargetPosition(JUANLIFTDOWN);// value is in ticks from above calculation
        juanLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        juanLift.setPower(JUANLIFTSPEED);
    }

    public void liftPartial() {
        juanLift.setTargetPosition(JUANLIFTPARTIAL);// value is in ticks from above calculation
        juanLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        juanLift.setPower(JUANLIFTSPEED);

    }


    //Patrick the intake's methods
    public void intakeOff() {
        patrickIntake.setPower(PATRIKINTAKECOFF);
    }
    public void intakeOn() {
        patrickIntake.setPower(PATRICKINTAKEON);
    }
    public void intakeEject() {
        patrickIntake.setPower(-PATRICKINTAKEON);
    }

    //Homie the box's methods
    public void homieRight() {
        homieBox.setPosition(HOMIEBOXPIVOTRIGHT);
    }
    public void homieLeft() {
        homieBox.setPosition(HOMIEBOXPIVOTLEFT);
    }
    public void homieCenter() {
        homieBox.setPosition(HOMIEBOXPIVOTCENTER);
    }


    //Julio the arm's methods
    public void julioRight() {
        julioPivot.setPosition(JULIOPIVOTRIGHT);
    }
    public void julioLeft() {
        julioPivot.setPosition(JULIOPIVOTLEFT);
    }
    public void julioCenter() {
        julioPivot.setPosition(JULIOPIVOTCENTER);
    }


    public void reset() {
        intakeOff();
        //liftPartial(); this one will not work becasue there is no while loop to let the motor compplete ite rotations
        liftToTargetHeight(JUANLIFTUP, 3);
        homieCenter();
        runtime.reset();
        while (runtime.seconds() < HOMIEDELAY) {

        }
        runtime.reset();
        julioCenter();
        while (runtime.seconds() < JULIODELAY){

        }
        //liftLower(); no while loop - remove
        liftToTargetHeight(JUANLIFTDOWN, 3);
    }

    // high goal is for the alliance hub so need LH and RH. The shared hub is only a low goal
    public void highGoalLeft() {
        //reset(); you will need to manually reset for noe. it messes up the flow
        //intakeOn();
        liftToTargetHeight(JUANLIFTPARTIAL, 3);
        intakeOff();
        homieLeft();
        liftToTargetHeight(JUANLIFTUP, 3);
        julioLeft();
        //homieRight(); need to wait to drop the cargo if you leave it her it drops immediatley

    }

    // high goal is for the alliance hub so need LH and RH. The shared hub is only a low goal
    public void highGoalRight() {
        //reset(); you will need to manually reset for noe. it messes up the flow
        //intakeOn();
        liftToTargetHeight(JUANLIFTPARTIAL, 3);
        intakeOff();
        homieRight();
        liftToTargetHeight(JUANLIFTUP, 3);
        julioRight();
        //homieRight(); need to wait to drop the cargo if you leave it her it drops immediatley

    }
    // need LH and RH here too depending on which side yoy are approaching from
    // blue you need to swing to the left, red swing to th right
    public void sharedHubBlue() {
        reset();

        //intakeOn();
        liftToTargetHeight(JUANLIFTLOW, 3);
        intakeOff();
        homieRight();
        julioRight();


    }

    public void sharedHubRed() {
        reset();
        //intakeOn();
        liftToTargetHeight(JUANLIFTLOW, 3);
        intakeOff();
        homieLeft();
        julioLeft();
    }

    public void liftToTargetHeight(double height, double timeoutS){

        int newTargetHeight;


        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {

            // Determine new target lift height in ticks based on the current position.
            // When the match starts the current position should be reset to zero.

            newTargetHeight = (int)(height *  TICKS_PER_LIFT_IN);
            // Set the target now that is has been calculated
            juanLift.setTargetPosition(newTargetHeight); //1000 ticks extends lift from 295mm to 530 mm which is 9.25 in per 1000 ticks or 108 ticks per in
            // Turn On RUN_TO_POSITION
            juanLift.setPower(Math.abs(JUANLIFTSPEED));
            // reset the timeout time and start motion.
            runtime.reset();
            juanLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // keep looping while we are still active, and there is time left, and thr motor is running.
            // Note: We use (isBusy() in the loop test, which means that when the motor hits
            // its target position, motion will stop.

            while (opmode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) && juanLift.isBusy()) {

                // Display it for the driver.
              //  telemetry.addData("Moving to New Lift Height",  "Running to %7d", newTargetHeight);

               // telemetry.update();
            }

            // Stop all motion after exiting the while loop
            juanLift.setPower(.25); // puts a low power to help hold the lift in place. There is a better way
            //liftPosition = LiftPosition.HOLD;
            // Turn off RUN_TO_POSITION
            //felipe.juanLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }


}




