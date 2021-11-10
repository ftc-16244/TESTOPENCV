package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.Alliance;

import static org.firstinspires.ftc.teamcode.Enums.Alliance.BLUE;

public class CarouselTurnerThingy {

    //Define Hardware Objects
    public DcMotor carousel = null;

    public static final double      CAROUSELOFF = 0;
    public static final double      CAROUSELON = 0.40;

    // create a constructor to add in the special requirements



    public void init(HardwareMap hwMap, Alliance Color)  {
        carousel = hwMap.get(DcMotor.class,"carousel");
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(Color == BLUE) {
            carousel.setDirection(DcMotor.Direction.FORWARD);
        }
        else {
            carousel.setDirection(DcMotor.Direction.REVERSE);
        }

    }

    public void carouselTurnOff() {
        carousel.setPower(CAROUSELOFF);
    }
    public void carouselTurnCCW() {
        carousel.setPower(CAROUSELON);
    }

    public void carouselTurnCW() {
        carousel.setPower(-CAROUSELON);
    }

    }



