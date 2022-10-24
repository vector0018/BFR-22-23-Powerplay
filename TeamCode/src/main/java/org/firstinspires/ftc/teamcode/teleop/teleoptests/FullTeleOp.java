package org.firstinspires.ftc.teamcode.teleop.teleoptests;

import static org.firstinspires.ftc.teamcode.teleop.teleoptests.SlideConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.teleop.teleoptests.SlideConstants.maxTargetPosition;
import static org.firstinspires.ftc.teamcode.teleop.teleoptests.SlideConstants.minTargetPosition;
import static org.firstinspires.ftc.teamcode.teleop.teleoptests.SlideConstants.slideTicksPerRev;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Disabled
@TeleOp
public class FullTeleOp extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
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

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            // add 0.2 inches to target position when left stick is pushed up or down

            targetPosition += gamepad2.left_stick_y * -0.2;

            //safeties for slide pos
            if (targetPosition > maxTargetPosition) {
                targetPosition = maxTargetPosition;
            }
            if (targetPosition < minTargetPosition) {
                targetPosition = minTargetPosition;
            }

            //takes starting position and subtracts from current input to get real position
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
            positionError = targetPosition - currentPosition;

            //proportional pid controller
            slidePower = (Kp * positionError) * 7;

            telemetry.addData("target position: ", targetPosition);
            telemetry.addData("position error: ", positionError);
            telemetry.addData("current position", currentPosition);
            telemetry.addData("slide power: ", slidePower);
            telemetry.addData("Zero Position: ", zeroPos);
            telemetry.addData("Right servo: ", rightClaw.getPosition());
            telemetry.addData("Left servo: ", leftClaw.getPosition());
            telemetry.update();

            //power safeties
            if (slidePower > 1) {
                slidePower = 1;
            }
            if (slidePower < -1) {
                slidePower = -1;
            }
            slideMotor.setPower(slidePower);

            //close claw
            if (gamepad2.left_bumper) {
                leftClaw.setPosition(1);
            }
            if (gamepad2.left_bumper) {
                rightClaw.setPosition(0.7);
            }
            //open claw
            if (gamepad2.right_bumper) {
                rightClaw.setPosition(0.3);
            }
            if (gamepad2.right_bumper) {
                leftClaw.setPosition(0.7);
            }

            //binds for specific heights - don't touch left stick, added 2 inches to junction height


            //ground junction
            if (gamepad2.a){
                targetPosition = 2;
                slideMotor.setPower(slidePower);
            }
            //low junction
            if (gamepad2.b){
                targetPosition = 15.5;
                slideMotor.setPower(slidePower);
            }
            //medium junction
            if (gamepad2.x) {
                targetPosition = 25.45;
                slideMotor.setPower(slidePower);
            }
            //high junction
            if (gamepad2.y){
                targetPosition = 35.5;
                slideMotor.setPower(slidePower);
            }
        }
    }
}
