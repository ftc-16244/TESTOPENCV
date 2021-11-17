package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

class NewFelipe {

    public DcMotor julioArm = null;


    public static final double      JULIOARMLEFT          = 90.0;
    public static final double      JULIOARMCENTER         = 0.0;
    public static final double      JULIOARMRIGHT        = -90.0;
    public static final double      JULIOTURNSPEED       =   0.6;
    public static final int        TICKS_PER_REV         = 500;
    public static final int        TICKS_PER_DEGREE = TICKS_PER_REV/360;
    ElapsedTime runtime = new ElapsedTime();

    LinearOpMode opmode;

    public NewFelipe(LinearOpMode opmode) {
        this.opmode = opmode;
    }

    public void rotateToTargetAngle(double angle, double timeoutS){

        int newTargetAngle;

        if (opmode.opModeIsActive()) {

            newTargetAngle = (int) (angle * TICKS_PER_DEGREE);
            julioArm.setTargetPosition(newTargetAngle);
        }
            julioArm.setPower(Math.abs(JULIOTURNSPEED));

            runtime.reset();
            julioArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            while (opmode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) && julioArm.isBusy()) {

            }
        }
    }

    public void work(){
        rotateToTargetAngle(JULIOARMLEFT, 3);
    }
}
