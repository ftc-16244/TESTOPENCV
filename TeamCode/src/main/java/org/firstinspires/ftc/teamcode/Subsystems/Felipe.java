package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Felipe {

    //Define Hardware Objects
    public DcMotor juanLift = null;
    public DcMotor  patrickIntake = null; // no longer used
    public Servo julioPivot = null;
    public Servo    homieBox = null;

    //Constants Lift
    private static final double     JUANLIFTSPEED       =   0.65;
    private static final int     JUANLIFTUP          =   14 ; //Number is in inches
    private static final int        JUANLIFTDOWN        =   0;
    private static final int        JUANLIFTPARTIAL        = 8;
    private static final int        TICKS_PER_LIFT_IN = 76; // determined experimentally
    private static final int        LIFT_HEIGHT_HIGH = (int) (JUANLIFTUP * TICKS_PER_LIFT_IN); // converts to ticks

    //Constants for robot arm
    public static final double      JULIOPIVOTLEFT      = 0;
    public static final double      JULIOPIVOTRIGHT     = 1;
    public static final double      JULIOPIVOTCENTER    = 0.5;

    //Constants for robot home box
    public static final double      HOMIEBOXPIVOTLEFT      = 0;
    public static final double      HOMIEBOXPIVOTRIGHT     = 1;
    public static final double      HOMIEBOXPIVOTCENTER    = 0.5;

    //Constants for robot intake
    public static final double PATRICKINTAKESPEED = .5;
    public static final double PATRIKINTAKECOFF = 0;
    public static final double PATRICKINTAKEON = 0.9;

    public void init(HardwareMap hwMap)  {
        juanLift = hwMap.get(DcMotor.class,"juanLift");
        patrickIntake = hwMap.get(DcMotor.class,"patrickIntake");
        julioPivot = hwMap.get(Servo.class,"julioPivot");
        homieBox = hwMap.get(Servo.class,"homieBox");

        //Positive=up and Negative=down
        juanLift.setDirection(DcMotor.Direction.FORWARD);
        juanLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        patrickIntake.setDirection(DcMotor.Direction.FORWARD);
        patrickIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        julioPivot.setPosition(JULIOPIVOTCENTER);
        homieBox.setPosition(HOMIEBOXPIVOTCENTER);
    }

    //// Single operation methods - see below for methods to be called in Opmodes


    //Juan the lift's methods
    public void liftRise() {
        juanLift.setTargetPosition(JUANLIFTUP);// value is in ticks from above calculation
        juanLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        juanLift.setPower(JUANLIFTSPEED);
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

    //Homie the box's methods
    public void homieClockwise() {
        homieBox.setPosition(HOMIEBOXPIVOTRIGHT);
    }
    public void homieCounterClockwise() {
        homieBox.setPosition(HOMIEBOXPIVOTLEFT);
    }
    public void homieCenter() {
        homieBox.setPosition(HOMIEBOXPIVOTCENTER);
    }


    //Julio the arm's methods
    public void julioClockwise() {
        julioPivot.setPosition(JULIOPIVOTRIGHT);
    }
    public void julioCounterClockwise() {
        julioPivot.setPosition(JULIOPIVOTLEFT);
    }
    public void julioCenter() {
        julioPivot.setPosition(JULIOPIVOTCENTER);
    }
    /*public void ArmCounterClockwise() {

        WobbleExtend.setTargetPosition(ARMEXTEND);
        WobbleExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        WobbleExtend.setPower(EXTENDSPEED);


    }
    public void ArmClockwise() {
        raiseWobbleClamp();
        WobbleExtend.setTargetPosition(ARMCONTRACT);
        WobbleExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        WobbleExtend.setPower(EXTENDSPEED);
        GripperOpen();
    }*/
    public void ArmCarryWobble() {
        raiseWobbleClamp();
        WobbleExtend.setTargetPosition(ARMCARRY);
        WobbleExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        WobbleExtend.setPower(EXTENDSPEED);
    }

    public void raiseWobbleClamp() {

        WobbleBaseClamp.setPosition(BASECLAMPUP);

    }

    public void lowerWobbleClamp() {

        WobbleBaseClamp.setPosition(BASECLAMPDOWN);

    }

    public void wobbleWristDown()  {

        WobbleWrist.setPosition(WRISTDOWN);

    }

    public void wobbleWristUp() {

        WobbleWrist.setPosition(WRISTUP);

    }
    public void wobbleWristStart() {

        WobbleWrist.setPosition(WRISTSTART);

    }

    ///// Multi Function methods to be called by the Opmodes

    public void resetWobble() {
        GripperClose();
        ArmContract();
        LiftLower();
    }

    public void readyToGrabWobble() {
        //LiftRise();
        //ArmExtend();
        GripperOpen();
        LiftLower();

        lowerWobbleClamp();
    }

    public void grabAndLift() {
        GripperClose();
        //LiftRise();
        raiseWobbleClamp();
    }

    public void lowerAndRelease() {
        LiftLower();
        GripperOpen();
    }
}



