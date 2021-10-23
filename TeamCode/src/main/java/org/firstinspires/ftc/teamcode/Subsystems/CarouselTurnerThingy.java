package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Enums.Alliance;

import static org.firstinspires.ftc.teamcode.Enums.Alliance.BLUE;

public class CarouselTurnerThingy {

    //Define Hardware Objects
    public DcMotor carousel = null;

    public static final double      CAROUSELOFF = 0;
    public static final double      CAROUSELON = 0.5;

    // create a constructor to add in the special requirements
    public CarouselTurnerThingy(HardwareMap hwMap, Alliance Color) {



    //public void init(HardwareMap hwMap, Alliance Alliance)  {
        carousel = hwMap.get(DcMotor.class,"carousel");
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(Color == BLUE) {
            carousel.setDirection(DcMotor.Direction.FORWARD);
        }
        else {
            carousel.setDirection(DcMotor.Direction.REVERSE);
        }

    }

    public void carouselTurnOn() {
        carousel.setPower(CAROUSELOFF);
    }
    public void carouselTurnOff() {
        carousel.setPower(CAROUSELON);
    }

}
