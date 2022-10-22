package org.firstinspires.ftc.teamcode.auton.autontests;

import static org.firstinspires.ftc.teamcode.teleop.teleoptests.SlideConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.teleop.teleoptests.SlideConstants.maxTargetPosition;
import static org.firstinspires.ftc.teamcode.teleop.teleoptests.SlideConstants.minTargetPosition;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name =  "Blue 2" )
public class AutonBlue2 extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotor slideMotor;
        ColorSensor colorSensor;
        Servo leftClaw;
        Servo rightClaw;
        double currentPosition;
        double targetPosition = 0;
        double zeroPos;
        double xVal;
        double yVal;
        double angleVal;
        double slidePower;
        ElapsedTime runTime = new ElapsedTime();


        slideMotor = hardwareMap.get(DcMotor.class, "SM");
        leftClaw = hardwareMap.get(Servo.class, "LC");
        rightClaw = hardwareMap.get(Servo.class, "RC");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "CS");
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        zeroPos = encoderTicksToInches(slideMotor.getCurrentPosition());
        leftClaw.setDirection(Servo.Direction.FORWARD);
        rightClaw.setDirection(Servo.Direction.REVERSE);

        Trajectory moveToG3 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(15)
                .build();
        Trajectory finishG3 = drive.trajectoryBuilder(moveToG3.end())
                .forward(4)
                .build();
        waitForStart();
        Trajectory back2Start = drive.trajectoryBuilder(finishG3.end())
                .strafeLeft(15)
                .build();
        waitForStart(); Trajectory moveToSignal = drive.trajectoryBuilder(back2Start.end())
                .forward(8)
                .build();
        waitForStart();
        rightClaw.setPosition(0.5);
        leftClaw.setPosition(0.6);
        sleep(500);

        telemetry.addData("claw", 0);
        telemetry.addData("Alpha: ", colorSensor.alpha());
        telemetry.addData("Blue: ", colorSensor.blue());
        telemetry.addData("Red: ", colorSensor.red());
        telemetry.addData("Green: ", colorSensor.green());
        telemetry.addData("argb; ", colorSensor.argb());
        telemetry.update();

        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();
        while (currentPosition < 2.0 && runTime.seconds()<0.5) {
            slidePower = moveSlide(currentPosition, 2);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }

        slideMotor.setPower(0);

        drive.followTrajectory(moveToG3);
        drive.followTrajectory(finishG3);
        leftClaw.setPosition(.8);
        rightClaw.setPosition(.7);
        sleep(100);
        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();
        while (currentPosition < 10.0 && runTime.seconds()<1.5) {
            slidePower = moveSlide(currentPosition, 10);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }
        drive.followTrajectory(back2Start);
        drive.followTrajectory(moveToSignal);
        sleep(5000);
        // drive.setWeightedDrivePower(new Pose2d(2, 0, 45));
        // sleep(500);

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
