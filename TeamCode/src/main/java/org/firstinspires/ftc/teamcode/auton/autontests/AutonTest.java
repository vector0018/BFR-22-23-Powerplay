package org.firstinspires.ftc.teamcode.auton.autontests;

import static org.firstinspires.ftc.teamcode.teleop.teleoptests.SlideConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.teleop.teleoptests.SlideConstants.maxTargetPosition;
import static org.firstinspires.ftc.teamcode.teleop.teleoptests.SlideConstants.minTargetPosition;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import static org.firstinspires.ftc.teamcode.teleop.teleoptests.SlideConstants.slideTicksPerRev;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name =  "Blue 1" )
public class AutonTest extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotor slideMotor;
        Servo leftClaw;
        Servo rightClaw;
        double currentPosition;
        double targetPosition = 0;
        double zeroPos;
        double xVal;
        double yVal;
        double angleVal;


        slideMotor = hardwareMap.get(DcMotor.class, "SM");
        leftClaw = hardwareMap.get(Servo.class, "LC");
        rightClaw = hardwareMap.get(Servo.class, "RC");
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        zeroPos = encoderTicksToInches(slideMotor.getCurrentPosition());
        leftClaw.setDirection(Servo.Direction.FORWARD);
        rightClaw.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {


            rightClaw.setPosition(0.5);
            leftClaw.setPosition(0.6);
            sleep(5000);

            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
            moveSlide (currentPosition , 2);
            sleep(5000);
            drive.setWeightedDrivePower(new Pose2d(0.1, 0, 0));
            sleep(5000);
            drive.setWeightedDrivePower(new Pose2d(-0.1, 0, 0));
            sleep(5000);
            drive.setWeightedDrivePower(new Pose2d(2, 0, 45));
            sleep(5000);

            /*
            // add 0.2 inches to target position when left stick is pushed up or down
            if (Math.abs(gamepad2.left_stick_y) > .01) {
                targetPosition += gamepad2.left_stick_y * -0.4;
            }
            else if (gamepad2.a){
                targetPosition = 2;
            }
            //low junction
            else if (gamepad2.b){
                targetPosition = 15.5;
            }
            //medium junction
            else if (gamepad2.y) {
                targetPosition = 25.45;
            }
            //high junction
            else if (gamepad2.x){
                targetPosition = 35.5;
            }

            slideMotor.setPower(slidePower);

            //open claw
            if (gamepad2.left_bumper) {
                leftClaw.setPosition(.8);
            }
            if (gamepad2.left_bumper) {
                rightClaw.setPosition(.7);
            }
            //close claw
            if (gamepad2.right_bumper) {
                rightClaw.setPosition(0.5);
            }
            if (gamepad2.right_bumper) {
                leftClaw.setPosition(0.6);
            }
            if (gamepad2.left_trigger > .1 || gamepad2.right_trigger > .1){
                leftClaw.setPosition(1);
                rightClaw.setPosition(1);
            }
            */
        }
    }
    private double moveSlide (double currentPosition , double targetPosition) {
        double Kp = 0.2;
        double positionError;
        double slidePower;

        //safeties for slide pos
        if (targetPosition > maxTargetPosition) {
            targetPosition = maxTargetPosition;
        }
        if (targetPosition < minTargetPosition) {
            targetPosition = minTargetPosition;
        }

        //takes starting position and subtracts from current input to get real position
        positionError = targetPosition - currentPosition;

        //proportional pid controller
        slidePower = Kp * positionError;

        //power safeties
        if (slidePower > 1) {
            slidePower = 1;
        }
        if (slidePower < -1) {
            slidePower = -1;
        }
        return slidePower;
    }
}
