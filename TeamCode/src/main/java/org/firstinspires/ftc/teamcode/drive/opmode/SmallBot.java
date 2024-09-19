package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp

public class SmallBot extends LinearOpMode {
    private DcMotor MiddleDrive;
    private DcMotor RightDrive;


    @Override
    public void runOpMode() {
        MiddleDrive = hardwareMap.get(DcMotor.class, "MiddleDrive");
        RightDrive = hardwareMap.get(DcMotor.class, "RightDrive");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //DRIVETRAIN CODE
            double middleAxis = -gamepad1.right_stick_x;
            double rightAxis = -gamepad1.left_stick_y;

            double middlePower = middleAxis;
            double rightPower = rightAxis;

            MiddleDrive.setPower(middlePower);
            RightDrive.setPower(rightPower);

            telemetry.update();
        }
    }
}



