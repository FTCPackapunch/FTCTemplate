/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Auto;


import static org.firstinspires.ftc.teamcode.Auto.HardwarePushbot_TC.intakePrimary;
import static org.firstinspires.ftc.teamcode.Auto.HardwarePushbot_TC.intakeSecondary;
import static org.firstinspires.ftc.teamcode.Auto.HardwarePushbot_TC.stackServo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@Autonomous(name = "BackRight", group = "TEST")
//@Disabled
public class BackTest extends LinearOpMode {
    Pose2d initPose,backboardPose, thirdPose, dropPurplePose,b34dropPurplePose,b45dropPurplePose,b6dropYellowPose ;// Starting Pose
    Vector2d wallPose,secondPose, fourthPose,backPose, fivePose, sixPose,thirdPoseV;

    Vector2d b4dropPurpleVector,b4backupVector,b6dropYellowVector,b1pickLWhiteVector,c5lineupRWhiteVector,c1pickRWhiteVector,c1backupToWallVector,c6parkVector;
    TrajectorySequence trajectoryParking;
    private ElapsedTime runtime = new ElapsedTime();
    private static int DESIRED_TAG_ID = -1;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagDetection desiredTag = null;
    static final double angleUp     =  0.95;
    static final double bucketIntakePosition    =  0.41;//0.74
    static final double bucketHalfPosition     =  0.69; //0.85
    static final double bucketDropfPosition     =  0.68;//0.61
    static final double armIntakePosition     =  1;
    static final double armOuttakePosition     =  0.62; //0 as highest
    static public final double smallServoOpen     =  0.55;
    static public final double smallServoClosed     =  0.12;
    static final double angleDown     =  0.51;


    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    private boolean startAprilTagCamera = false;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    OpenCvWebcam webcam;

    HardwarePushbot_TC  robot;

    double newTarget;

    @Override
    public void runOpMode() {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        boolean targetFound = false;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot = new HardwarePushbot_TC(hardwareMap);
        robot.leftFront.setDirection(DcMotorEx.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        robot.rightFront.setDirection(DcMotorEx.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        robot.leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        robot.rightRear.setDirection(DcMotorEx.Direction.FORWARD);


        robot.leftLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ;
        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();



        /*
         * Wait for the user to press start on the Driver Station
         */

        waitForStart();
        if (opModeIsActive() && !targetFound) {
            initPose = new Pose2d(0, 0, Math.toRadians(0));//Starting pose
            trajectoryParking = drive.trajectorySequenceBuilder(initPose)
                    .back(10)
                    .UNSTABLE_addTemporalMarkerOffset(0.01, () ->
                            {
                                stackServo.setPosition(0.40);
                            }
                    )
                    .waitSeconds(1)
                    .back(10,SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () ->
                            {
                                stackServo.setPosition(0.25);
                            }
                    )
                    .waitSeconds(0.2)
                    .forward(10, SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                    .UNSTABLE_addTemporalMarkerOffset(0.01, () ->
                            {
                                intakePrimary.setPower(1);
                                intakeSecondary.setPower(1);
                                stackServo.setPosition(0.18);
                            }
                    )
                    .back(8, SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                    .waitSeconds(0.5)
                    .UNSTABLE_addTemporalMarkerOffset(0.7, () ->
                            {
                                intakePrimary.setPower(-1);
                                intakeSecondary.setPower(-1);
                                robot.angleServo.setPosition(angleDown);
                            }
                    )
                    .forward(10)
                    .build();
            drive.followTrajectorySequence(trajectoryParking);
        }
    }
}