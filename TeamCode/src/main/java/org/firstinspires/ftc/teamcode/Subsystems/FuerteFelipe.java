package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FuerteFelipe {

    //Define Hardware Objects

    ///private ElapsedTime     runtime = new ElapsedTime();

    public DcMotor  linearActuator = null;

    // Need some features from the Linear Opmode to make the lift work
    Telemetry telemetry;
    LinearOpMode myOpMode;
    ElapsedTime runtime = new ElapsedTime();

    //Constants Lift
    public static final double      linearActuatorSPEED         =   0.9; // if this is too fast you cannot reset without hitting the framwe
    public static final int         linearActuatorDOWN          =   0;
    public static final int         linearActuatorPARTIAL       =   5;
    public static final int         linearActuatorUP            =   7; //Number is in inches
    public static final double      linearActuatorLOAD          =  .25; //Number is in inches
    private static final double     TICKS_PER_MOTOR_REV         =   145.1; // goBilda 1150 RPM motor
    private static final double     ACTUATOR_DISTANCE_PER_REV   = 8/25.4; // 8mm of travel per rev converted to inches
    public static final double      TICKS_PER_LIFT_IN           = TICKS_PER_MOTOR_REV / ACTUATOR_DISTANCE_PER_REV; // 460 and change

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
        liftToTargetHeight(linearActuatorUP,3);
    }
    public void liftPartial() {
        liftToTargetHeight(linearActuatorPARTIAL,3);
    }
    public void liftLow() {
        liftToTargetHeight(linearActuatorDOWN,3);
    }
    public void liftLoad() {liftToTargetHeight(linearActuatorLOAD,1);}

    // get Juan's Position need a local variable to do this
    public double getJuanPosition(){
        double juanPositionLocal;
        juanPositionLocal = linearActuator.getCurrentPosition()/ TICKS_PER_LIFT_IN; //returns in inches
        return  juanPositionLocal;
    }


    public void juanMechanicalReset(){
        linearActuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearActuator.setPower(-0.6);
        runtime.reset();
        // opmode is not active during init so take that condition out of the while loop
        while ((runtime.seconds() < 2.0)) {

        }
        linearActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearActuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // high goal is for the alliance hub so need LH and RH. The shared hub is only a low goal


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


        }
    }


}




