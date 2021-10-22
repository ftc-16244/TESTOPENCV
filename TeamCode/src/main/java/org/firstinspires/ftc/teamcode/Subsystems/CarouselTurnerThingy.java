package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CarouselTurnerThingy {

    //Define Hardware Objects
    public DcMotor carousel = null;

    public static final double      CAROUSELOFF = 0;
    public static final double      CAROUSELON = 0.5;

    public void init(HardwareMap hwMap)  {
        carousel = hwMap.get(DcMotor.class,"carousel");
        carousel.setDirection(DcMotor.Direction.FORWARD);
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void carouselTurnOn() {
        carousel.setPower(CAROUSELOFF);
    }
    public void carouselTurnOff() {
        carousel.setPower(CAROUSELON);
    }

}
