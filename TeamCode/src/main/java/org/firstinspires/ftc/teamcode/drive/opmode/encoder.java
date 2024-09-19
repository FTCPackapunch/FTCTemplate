package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "Presets ")
public class encoder extends OpMode {
    DcMotor motor;
    DcMotor motor2;
    double ticks = 537.7;
    double newTarget;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "rightLinear");
        motor2 = hardwareMap.get(DcMotor.class, "leftLinear");
        telemetry.addData("Hardware: ", "Initialized");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            lineOne(1);
        }
        if(gamepad1.y){
            lineTwo(1);
        }
        if(gamepad1.x){
            bottom(1);
        }
        telemetry.addData("Motor Ticks: ", motor.getCurrentPosition());
        telemetry.addData("Motor2 Ticks: ", motor2.getCurrentPosition());
        if(gamepad1.b){
            reset();
        }
        if(gamepad1.dpad_up){
            hangUp(1);
        }
        if(gamepad1.dpad_down){
            hangDown(1);
        }

    }
    public void lineOne(int turnage){
        newTarget = ticks;
        motor.setTargetPosition(-1935);
        motor.setPower(1);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setTargetPosition(1935);
        motor2.setPower(1);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void lineTwo(int turnage){
        newTarget = ticks;
        motor.setTargetPosition(-3091);
        motor.setPower(0.3);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setTargetPosition(3091);
        motor2.setPower(0.3);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void bottom(int turnage){
        newTarget = ticks;
        motor.setTargetPosition(0);
        motor.setPower(0.7);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setTargetPosition(0);
        motor2.setPower(0.7);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void hangUp(int turnage){
        newTarget = ticks;
        motor.setTargetPosition(-1983);
        motor.setPower(1);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setTargetPosition(1983);
        motor2.setPower(1);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void hangDown(int turnage){
        newTarget = ticks;
        motor.setTargetPosition(-805);
        motor.setPower(0.5);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setTargetPosition(805);
        motor2.setPower(0.5);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void reset(){

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}