package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.LiftPosition;
import org.firstinspires.ftc.teamcode.Enums.PatrickState;
import org.firstinspires.ftc.teamcode.Subsystems.CarouselTurnerThingy;
import org.firstinspires.ftc.teamcode.Subsystems.Felipe;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.Enums.Alliance.BLUE;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "Test")
@Disabled
public class FieldCoordiante_Auto_test_Blue_Carosel_Side extends LinearOpMode {
    public static double DISTANCE = 30; // in
    Felipe felipe = new Felipe(this); // instantiate Felipe (the main implement)
    CarouselTurnerThingy carousel = new CarouselTurnerThingy();
    // init and setup
    ElapsedTime runtime = new ElapsedTime();
    double X_Field_Start_Coord = -30;
    double Y_Field_Start_Coord = 62;
    double Heading_Field_Start = 0;// thisnneeds to be in degrees


    // ENUMS
    //DriveSpeedState  currDriveState;
    PatrickState patrickState = PatrickState.OFF;
    LiftPosition liftPosition = LiftPosition.LOAD;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // initialize the other subsystems
        felipe.init(hardwareMap);
        carousel.init(hardwareMap);

        ///////////////////////////////////////////////////////////////////////////
        // Trajectories Here
        ///////////////////////////////////////////////////////////////////////////
       // Define the robot stating pose in field oordiantes for the start posionn and heading
        Pose2d startPose = new Pose2d(X_Field_Start_Coord, Y_Field_Start_Coord, Math.toRadians(Heading_Field_Start));
        Trajectory  traj1 = drive.trajectoryBuilder(startPose)
                .forward(15)
                //.addTemporalMarker(-1,()->{felipe.highGoal();})
                .build();

        Trajectory  traj2 = drive.trajectoryBuilder(traj1.end().plus(new Pose2d(0,0,Math.toRadians(-90))))
                .back(18)
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeLeft(9)
                .addDisplacementMarker(()->{felipe.homieRight();})
                .build();
        // Move away from the alliance shipping hub so the arm can be retracted without hitting the hub
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .strafeRight(8)
                .addDisplacementMarker(()->{felipe.reset();})
                .build();

        Trajectory junk = drive.trajectoryBuilder(traj1.end().plus(new Pose2d(0,0,Math.toRadians(-90))))
                .lineToLinearHeading(new Pose2d(10, 10, Math.toRadians(0)))
                //.splineTo(new Vector2d(20,14),Math.toRadians(-90))

                .build();



        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traj1);
        drive.turn(Math.toRadians(-90));


        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
