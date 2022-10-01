package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class SlideTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor slideMotor = null;
        Servo leftClaw = null;
        Servo rightClaw = null;
        double slidePower = gamepad2.left_stick_y * 0.7;
        slideMotor = hardwareMap.get(DcMotor.class,"SM");
        waitForStart();

        while (opModeIsActive()) {
            slideMotor.setPower(slidePower);
            if (gamepad2.left_bumper );

            if (gamepad2.right_bumper);
        }
    }
}
