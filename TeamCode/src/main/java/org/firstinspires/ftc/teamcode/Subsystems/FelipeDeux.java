package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.LiftPosition;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

public class FelipeDeux {

    //Define Hardware Objects

    ///private ElapsedTime     runtime = new ElapsedTime();

    public DcMotor  linearActuator   = null;
    public DcMotor  patrickIntake    = null; // no longer used
    public DcMotor  julioArm         = null; // new
    public Servo    homieBox         = null;
    public Servo    cristianoCodo    = null; //arm
    public Servo    panchoPulgar     = null; //thumb

    // Need some features from the Linear Opmode to make the lift work


    ElapsedTime runtime = new ElapsedTime();

    //Constants Lift
    public static final double      JUANLIFTSPEED       =   0.6; // if this is too fast you cannot reset without hitting the framwe
    //public static final int       JUANLIFTDOWN        =   0;
    public static final double      JUANLIFTPARTIAL     =   4.75;
    public static final int         JUANLIFTLOW         =   2;
    public static final double      JUANLIFTUP          =   7.5; //Number is in inches
    public static final double      JUANLIFTLOAD        =   0.25; //Number is in inches
    private static final double     TICKS_PER_MOTOR_REV         =   145.1; // goBilda 1150 RPM motor
    private static final double     ACTUATOR_DISTANCE_PER_REV   = 8/25.4; // 8mm of travel per rev converted to inches
    public static final double      TICKS_PER_LIFT_IN           = TICKS_PER_MOTOR_REV / ACTUATOR_DISTANCE_PER_REV; // 460 and change


    public static final double      linearActuatorSPEED         =   0.9; // if this is too fast you cannot reset without hitting the framwe
    public static final int         linearActuatorDOWN          =   0;
    public static final int         linearActuatorPARTIAL       =   5;
    public static final int         linearActuatorUP            =   7; //Number is in inches
    public static final double      linearActuatorLOAD          =  .25; //Number is in inches


    public static final double      CRISTIANOCODOINIT =  0.0;
    public static final double      CRISTIANOCODOMID =  0.5;
    public static final double      CRISTIANOCODOLOW =  0.75;

    public static final double      PANCHOPULGAROPEN =  0.2;
    public static final double      PANCHOPULGARCLOSE =  0.4;






    //Constants for servo driven Julio (Now Obsolete)
    //public static final double      JULIOPIVOTLEFT          = 0.1;
    //public static final double      JULIOPIVOTRIGHT         = 0.8;
    //public static final double      JULIOPIVOTCENTER        = 0.445;
    public static final double      JULIODELAY              = 0.5;

    //Constants for new motor version of Julio
    public static final double      JULIOARMLEFT            =   -90.0;
    public static final double      JULIOARMCENTER          =   0.0;
    public static final double      JULIOARMRIGHT           =   90.0;
    public static final double      JULIOTURNSPEED          =   0.3;
    public static final double      TICKS_PER_REV           =   1425.1; // 117 RPM motor 50.9:1 reduction
    public static final double      TICKS_PER_DEGREE         =  TICKS_PER_REV/360;

    //Constants for robot home box
    public static final double      HOMIEBOXPIVOTLEFT       = 1;
    public static final double      HOMIEBOXPIVOTRIGHT      = 0.3;
    public static final double      HOMIEBOXPIVOTCENTER     = 0.66;
    public static final double      HOMIEDELAY              = 0.15;

    //Constants for robot intake
    public static final double      PATRICKINTAKESLOW = .3;//use this while lifting juan
    public static final double      PATRIKINTAKECOFF = 0;
    public static final double      PATRICKINTAKEON = 0.7;


    LiftPosition liftPosition = LiftPosition.UNKNOWN;


    LinearOpMode opmode;
    public FelipeDeux(LinearOpMode opmode) {
        this.opmode = opmode;
    }


    public void init(HardwareMap hwMap)  {

        // Initialize Juan - Linear Actuator type of lift
        linearActuator = hwMap.get(DcMotor.class,"juanLift");
        linearActuator.setDirection(DcMotor.Direction.FORWARD);
        linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Initialize Patrick which is the intake drive motor
        patrickIntake = hwMap.get(DcMotor.class,"patrickIntake");
        patrickIntake.setDirection(DcMotor.Direction.FORWARD);
        patrickIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize Julio the rotating arm to deliver Homie to the sides
        julioArm = hwMap.get(DcMotor.class,"julioArm");
        julioArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        julioArm.setDirection(DcMotor.Direction.FORWARD);
        julioArm.setDirection(DcMotor.Direction.FORWARD);
        julioArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Initialize Homie the servo that rotates the freight capture bucket
        homieBox = hwMap.get(Servo.class,"homieBox");

        // pre-position during init
        homieBox.setPosition(HOMIEBOXPIVOTCENTER);

        cristianoCodo = hwMap.get(Servo.class,"arm");
        cristianoCodo.setPosition(CRISTIANOCODOINIT);

        panchoPulgar = hwMap.get(Servo.class,"thumb");
        panchoPulgar.setPosition(PANCHOPULGARCLOSE);

    }

    //// Single operation methods - see below for methods to be called in Opmodes


    //Juan the lift's methods

    //Juan the lift's methods (these use the while loop so be casrefull)
    public void liftRise() {
        liftToTargetHeight(JUANLIFTUP ,3);
    }
    public void liftPartial() {
        liftToTargetHeight(JUANLIFTPARTIAL ,3);
    }
    public void liftLow() {
        liftToTargetHeight( JUANLIFTLOAD   ,3);
    }

    // use this method to get Juan at the correct height after a mechanical reset
    public void liftLoad() {
        liftToTargetHeight(JUANLIFTLOAD,1);
        liftPosition = LiftPosition.LOAD; // set state accordingly after this is done
    }

    // get Juan's Position need a local variable to do this
    public double getJuanPosition(){
        double juanPositionLocal;
        juanPositionLocal = linearActuator.getCurrentPosition()/ TICKS_PER_LIFT_IN; //returns in inches
        return  juanPositionLocal;
    }


    public void juanMechanicalReset(){
        linearActuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // need to swich off encoder to run with a timer
        linearActuator.setPower(-0.6);
        runtime.reset();
        // opmode is not active during init so take that condition out of the while loop
        while ((runtime.seconds() < 2.0)) {

            //Time wasting loop
        }
        // set everything back the way is was before reset so encoders can be used
        linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftPosition = LiftPosition.MECH_RESET;

    }

    // high goal is for the alliance hub so need LH and RH. The shared hub is only a low goal



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
    public double getJulioPosition(){
        double julioPositionLocal;
        julioPositionLocal = julioArm.getCurrentPosition()/  TICKS_PER_DEGREE; //returns in inches
        return  julioPositionLocal;
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

    public void armInit(){
        cristianoCodo.setPosition(CRISTIANOCODOINIT);
    }

    public void armMid(){
        cristianoCodo.setPosition(CRISTIANOCODOMID);
    }

    public void armLow(){
        cristianoCodo.setPosition(CRISTIANOCODOLOW);
    }
    public void thumbOpen(){
        panchoPulgar.setPosition((PANCHOPULGAROPEN));
    }
    public void thumbClose(){
        panchoPulgar.setPosition((PANCHOPULGARCLOSE));
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


    public void reset() {
        intakeOff();
        //liftPartial(); this one will not work becasue there is no while loop to let the motor compplete ite rotations
        liftToTargetHeight(JUANLIFTPARTIAL, 3);
        homieCenter();
        runtime.reset();

        julioCenter();
        liftToTargetHeight(JUANLIFTLOAD, 3);
    }

    // high goal is for the alliance hub so need LH and RH. The shared hub is only a low goal
    public void highGoalLeft() {
        //reset(); you will need to manually reset for noe. it messes up the flow
        //intakeOn();
        liftToTargetHeight(JUANLIFTPARTIAL, 3);
        intakeOff();
        homieLeft();

        julioLeft90();
        //homieRight(); need to wait to drop the cargo if you leave it her it drops immediatley

    }

    // high goal is for the alliance hub so need LH and RH. The shared hub is only a low goal
    public void highGoalRight() {
        //reset(); you will need to manually reset for noe. it messes up the flow
        //intakeOn();
        liftToTargetHeight(JUANLIFTPARTIAL, 3);
        intakeOff();
        homieRight();
        julioRight90();
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
        julioRight90();


    }

    public void sharedHubRed() {
        reset();
        //intakeOn();
        liftToTargetHeight(JUANLIFTLOW, 3);
        intakeOff();
        homieLeft();
        julioLeft90();
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
            linearActuator.setPower(Math.abs(JUANLIFTSPEED));
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

        }
    }


}




