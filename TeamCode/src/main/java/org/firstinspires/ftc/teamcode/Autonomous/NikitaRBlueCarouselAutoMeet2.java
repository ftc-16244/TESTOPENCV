package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.Enums.Barcode;
import org.firstinspires.ftc.teamcode.Enums.GameElement;
import org.firstinspires.ftc.teamcode.Enums.LiftPosition;
import org.firstinspires.ftc.teamcode.Enums.PatrickState;
import org.firstinspires.ftc.teamcode.Subsystems.CarouselTurnerThingy;
import org.firstinspires.ftc.teamcode.Subsystems.FelipeDeux;
import org.firstinspires.ftc.teamcode.Test.SkystonePipeline;
import org.firstinspires.ftc.teamcode.Test.SkystoneAutoMode_webcam;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;


/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "Test")
public class NikitaRBlueCarouselAutoMeet2 extends LinearOpMode {

    OpenCvCamera webcam;

    public static double DISTANCE = 30; // in
    public ElapsedTime   tfTime      = new ElapsedTime(); // timer for tensor flow
    FelipeDeux felipe = new FelipeDeux(this); // instantiate Felipe (the main implement)
    CarouselTurnerThingy carousel = new CarouselTurnerThingy();
    // init and setup
    ElapsedTime runtime = new ElapsedTime();


    // ENUMS
    //DriveSpeedState  currDriveState;
    PatrickState patrickState = PatrickState.OFF;
    LiftPosition liftPosition = LiftPosition.LOAD;

    Barcode barcode = Barcode.RIGHT; // Default target zone
    GameElement gameElement = GameElement.BALL; //set to Ball since this is impossible

    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };



    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        SkystonePipeline detector = new SkystonePipeline(telemetry);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        ElapsedTime timer = new ElapsedTime();
        double ducktime = 2.5; // carosel rotation time
        // initialize the other subsystems
        felipe.init(hardwareMap);
        carousel.init(hardwareMap);
        felipe.juanMechanicalReset();

        ///////////////////////////////////////////////////////////////////////////
        // Trajectories - HIGH GOAL
        ///////////////////////////////////////////////////////////////////////////
        Trajectory  traj_HG_01 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(42,3,Math.toRadians(179)))
                .addTemporalMarker(-.25,()->{felipe.armMid();})
                //.addTemporalMarker(-.25,()->{felipe.liftRise();})
                .build();

        Trajectory  traj_HG_02 = drive.trajectoryBuilder(traj_HG_01.end())
                .lineToLinearHeading(new Pose2d(42,-34,Math.toRadians(179)))
                .addTemporalMarker(-0.8,()->{felipe.thumbOpen();})
                .addTemporalMarker(1,()->{felipe.thumbClose();})
                .addTemporalMarker(1.5,()->{felipe.armInit();})

                .build();

        Trajectory  traj_HG_03 = drive.trajectoryBuilder(traj_HG_02.end())

                .lineToLinearHeading(new Pose2d(15,-34,Math.toRadians(179)))

                .build();
        Trajectory  traj_HG_04 = drive.trajectoryBuilder(traj_HG_03.end())
                // final touch up to engage carousel
                .forward(5)
                .addTemporalMarker(.25,()->{carousel.carouselTurnCCWAuto();})
                .build();
        Trajectory  traj_HG_05 = drive.trajectoryBuilder(traj_HG_04.end())
                //back away but stay out of the wall to make it move better
                .lineToLinearHeading(new Pose2d(30,-30,Math.toRadians(90)))
                .addTemporalMarker(.25,()->{carousel.carouselTurnOff();})
                .build();

        ///////////////////////////////////////////////////////////////////////////
        // Trajectories - MIDDLE GOAL
        ///////////////////////////////////////////////////////////////////////////


        Trajectory  traj_MG_01 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(42,-26,Math.toRadians(179)))
                //.addTemporalMarker(-.25,()->{felipe.armMid();})
                //.addTemporalMarker(-.25,()->{felipe.liftRise();})
                .build();

        Trajectory  traj_MG_02 = drive.trajectoryBuilder(traj_MG_01.end())
                .lineToLinearHeading(new Pose2d(42,3,Math.toRadians(179)))
                .addTemporalMarker(0.25,()->{felipe.armMid();})
                //.addTemporalMarker(-.25,()->{felipe.liftRise();})
                .build();
        Trajectory  traj_MG_03 = drive.trajectoryBuilder(traj_MG_02.end())

                .lineToLinearHeading(new Pose2d(42,-34,Math.toRadians(179)))
                .addTemporalMarker(-0.8,()->{felipe.thumbOpen();})
                .addTemporalMarker(1,()->{felipe.thumbClose();})
                .addTemporalMarker(1.5,()->{felipe.armInit();})
                .build();
        Trajectory  traj_MG_04 = drive.trajectoryBuilder(traj_MG_03.end())
                //.addTemporalMarker(-0.8,()->{felipe.thumbOpen();})
                //.addTemporalMarker(1,()->{felipe.thumbClose();})
                //.addTemporalMarker(1.5,()->{felipe.armInit();})
                .lineToLinearHeading(new Pose2d(15,-34,Math.toRadians(179)))

                .build();
        Trajectory  traj_MG_05 = drive.trajectoryBuilder(traj_MG_04.end())
                // final touch up to engage carousel
                .forward(5)
                .addTemporalMarker(.25,()->{carousel.carouselTurnCCW();})
                .build();
        Trajectory  traj_MG_06 = drive.trajectoryBuilder(traj_MG_05.end())
                //back away but stay out of the wall to make it move better
                .lineToLinearHeading(new Pose2d(28,-31,Math.toRadians(90)))
                .addTemporalMarker(.25,()->{carousel.carouselTurnOff();})
                .build();
        ///////////////////////////////////////////////////////////////////////////
        // Trajectories - LOW GOAL
        ///////////////////////////////////////////////////////////////////////////
        Trajectory  traj_LG_01 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(42,-26,Math.toRadians(179)))
                .build();
        Trajectory  traj_LG_02 = drive.trajectoryBuilder(traj_LG_01.end())
                .lineToLinearHeading(new Pose2d(42,3,Math.toRadians(179)))
                .addTemporalMarker(-.25,()->{felipe.armLow();})
                //.addTemporalMarker(-.25,()->{felipe.liftRise();})
                .build();
        Trajectory traj_LG_03 = traj_MG_03;
        Trajectory traj_LG_04 = traj_MG_04;
        Trajectory traj_LG_05 = traj_MG_05;
        Trajectory traj_LG_06 = traj_MG_06;


        waitForStart();
        felipe.liftLoad();// put here becase opmode is acitve is a condition in the method that does this
        tfTime.reset(); //  reset the TF timer

            switch(detector.getLocation()){
                case LEFT: //
                    drive.followTrajectory(traj_LG_01);
                    drive.followTrajectory(traj_LG_02);
                    drive.followTrajectory(traj_LG_03);
                    drive.followTrajectory(traj_LG_04);
                    felipe.liftLoad();
                    drive.followTrajectory(traj_LG_05);//park

                    //delay to let carousel turn
                    timer.reset();
                    while(timer.seconds() < ducktime) drive.update();

                    drive.followTrajectory(traj_LG_06);

                    break;

                case CENTER: //
                    drive.followTrajectory(traj_MG_01);
                    drive.followTrajectory(traj_MG_02);
                    drive.followTrajectory(traj_MG_03);
                    drive.followTrajectory(traj_MG_04);
                    felipe.liftLoad();
                    drive.followTrajectory(traj_MG_05);
                    timer.reset();
                    while(timer.seconds() < ducktime) drive.update();

                    drive.followTrajectory(traj_MG_06); //park

                    break;

                case RIGHT: //level 3 highest goal
                    felipe.liftRise();
                    drive.followTrajectory(traj_HG_01);
                    drive.followTrajectory(traj_HG_02);
                    drive.followTrajectory(traj_HG_03);
                    felipe.liftLoad();
                    drive.followTrajectory(traj_HG_04);

                    //delay to let carousel turn
                    timer.reset();
                    while(timer.seconds() < ducktime) drive.update();

                    drive.followTrajectory(traj_HG_05);

                    break;
            }
        //}

        if (isStopRequested()) return;


        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;

        webcam.stopStreaming();
    }


}
