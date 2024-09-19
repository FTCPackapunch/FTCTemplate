package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
/*
* This is a simple routine to test turning capabilities.
*/
@Config
//@Disabled
@Autonomous(name = "Auto Test", group = "Concept")


public class MyOpMode extends LinearOpMode {

    Pose2d initPose, thirdPose; // Starting Pose
    Vector2d secondPose, fourthPose, fivePose;
    TrajectorySequence trajectoryParking;
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
      //  StandardTrackingWheelLocalizer drive = new StandardTrackingWheelLocalizer(hardwareMap);


        waitForStart();

        if(isStopRequested()) return;


        initPose = new Pose2d(0, 0, Math.toRadians(0));//Starting pose
        secondPose = new Vector2d(30,0);
        thirdPose = new Pose2d(28,0, Math.toRadians(90));
        fourthPose = new Vector2d(28, 80);
        fivePose= new Vector2d(28, -15);


        trajectoryParking  = drive.trajectorySequenceBuilder(initPose)
                .lineTo(secondPose)
                .lineToLinearHeading(thirdPose)
                .lineTo(fourthPose)
                .lineTo(fivePose)
                .lineTo(fourthPose)
                .lineTo(fivePose)
                .lineTo(fourthPose)
                .build();


                //   .lineTo(new Vector2d(30,-36))
          //      .lineTo(new Vector2d(30,-36))
            //    .lineToLinearHeading(new Pose2d(50,-85, Math.toRadians(253)))


        drive.followTrajectorySequence(trajectoryParking);
    }
}
