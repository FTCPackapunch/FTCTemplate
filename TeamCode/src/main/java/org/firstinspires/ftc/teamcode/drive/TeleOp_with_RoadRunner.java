/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
@TeleOp(name="Teleop with Road Runner", group = "Concept")
@Disabled
public class TeleOp_with_RoadRunner extends LinearOpMode {

    private DcMotor leftFrontDrive = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive = null;  //  Used to control the right back drive wheel
    Pose2d initPose, thirdPose; // Starting Pose
    TrajectorySequence trajectoryParking;
    double strafe = 0;        // Desired strafe power/speed (-1 to +1)
    double turn = 0;
    double driveX = 0;

    public void runOpMode() {
        // Insert whatever initialization your own code does
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "frontRight");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        // Assuming you're using StandardTrackingWheelLocalizer.java
        // Switch this class to something else (Like TwoWheeTrackingLocalizer.java) if your configuration is different
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set your initial pose to x: 0, y: 0, facing 90 degrees
         drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        waitForStart();

        while (opModeIsActive()) {
            driveX = -gamepad1.left_stick_y; // Reduce drive rate to 50%.
            strafe = -gamepad1.left_stick_x;  // Reduce strafe rate to 50%.
            turn = -gamepad1.right_stick_x * 0.65;  // Reduce turn rate to 33%.
            // Make sure to call drive.update() on *every* loop
            // Increasing loop time by utilizing bulk reads and minimizing writes will increase your odometry accuracy
               drive.update();

            // Retrieve your pose
                 Pose2d myPose = drive.getPoseEstimate();

              telemetry.addData("x", myPose.getX());
               telemetry.addData("y", myPose.getY());
               telemetry.addData("heading", myPose.getHeading());


            if (gamepad1.dpad_left) {
                // Insert whatever teleop code you're using
                //initPose = new Pose2d(0, 0, Math.toRadians(0));//Starting pose
                trajectoryParking = drive.trajectorySequenceBuilder(myPose)
                        .strafeLeft(5)
                        .build();
                drive.followTrajectorySequence(trajectoryParking);

            }
            if (gamepad1.dpad_right) {
                // Insert whatever teleop code you're using
               // initPose = new Pose2d(0, 0, Math.toRadians(0));//Starting pose
                trajectoryParking = drive.trajectorySequenceBuilder(myPose)
                        .strafeRight(5)
                        .build();
                drive.followTrajectorySequence(trajectoryParking);
            }
            // Apply desired axes motions to the drivetrain.
            moveRobot(driveX, strafe, turn);

        }

    }


    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;


        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));


        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;

        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);


    }
}
