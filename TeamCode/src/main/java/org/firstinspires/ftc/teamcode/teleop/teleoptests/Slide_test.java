package org.firstinspires.ftc.teamcode.teleop.teleoptests;

import static org.firstinspires.ftc.teamcode.teleop.teleoptests.SlideConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.teleop.teleoptests.SlideConstants.maxTargetPosition;
import static org.firstinspires.ftc.teamcode.teleop.teleoptests.SlideConstants.minTargetPosition;
import static org.firstinspires.ftc.teamcode.teleop.teleoptests.SlideConstants.slideTicksPerRev;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Slide_test extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        DcMotor slideMotor = null;
        Servo leftClaw = null;
        Servo rightClaw = null;
        double Kp = 0.1;
        double currentPosition;
        double targetPosition = 0;
        double positionError;
        double slidePower;
        double zeroPos = 0;


        slideMotor = hardwareMap.get(DcMotor.class, "SM");
        leftClaw = hardwareMap.get(Servo.class, "LC");
        rightClaw = hardwareMap.get(Servo.class, "RC");
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        zeroPos = encoderTicksToInches(slideMotor.getCurrentPosition());
        leftClaw.setDirection(Servo.Direction.REVERSE);

            waitForStart();

            while (opModeIsActive()) {
                targetPosition += gamepad2.left_stick_y * -0.2;
                if (targetPosition > maxTargetPosition) {
                    targetPosition = maxTargetPosition;
                }
                if (targetPosition < minTargetPosition) {
                    targetPosition = minTargetPosition;
                }
                currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
                positionError = targetPosition - currentPosition;
                slidePower = Kp * positionError;
                telemetry.addData("target position: ", targetPosition);
                telemetry.addData("position error: " , positionError);
                telemetry.addData("current position", currentPosition);
                telemetry.addData("slide power: " , slidePower);
                telemetry.addData("Zero Position: ", zeroPos);
                telemetry.addData("Right servo: ", rightClaw.getPosition());
                telemetry.addData("Left servo: ", leftClaw.getPosition());
                telemetry.update();

                if (slidePower > 1) {
                    slidePower = 1;
                }
                if (slidePower < -1) {
                    slidePower = -1;
                }
                slideMotor.setPower(slidePower);

                if (gamepad2.left_bumper){
                    leftClaw.setPosition(1);
                    rightClaw.setPosition(0.6);
                }
                if (gamepad2.right_bumper)
                    leftClaw.setPosition(0.6);
                    rightClaw.setPosition(0.2);
            }

    }
}