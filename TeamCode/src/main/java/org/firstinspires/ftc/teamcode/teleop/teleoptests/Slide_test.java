package org.firstinspires.ftc.teamcode.teleop.teleoptests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class Slide_test extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        DcMotor slideMotor = null;
        Servo leftClaw = null;
        Servo rightClaw = null;

        slideMotor = hardwareMap.get(DcMotor.class, "SM");
        leftClaw = hardwareMap.get(Servo.class, "LC");
        rightClaw = hardwareMap.get(Servo.class, "RC");

            waitForStart();

            while (opModeIsActive()) {
                double slidePower = gamepad2.left_stick_y * 0.7;
                slideMotor.setPower(slidePower);

                if (gamepad2.left_bumper){
                    leftClaw.setPosition(.5);
                }
                if (gamepad2.right_bumper){
                    rightClaw.setPosition(.5);
            }

    }
}
}