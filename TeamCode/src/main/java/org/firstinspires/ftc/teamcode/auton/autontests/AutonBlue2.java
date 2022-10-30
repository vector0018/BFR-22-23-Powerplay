package org.firstinspires.ftc.teamcode.auton.autontests;

import static org.firstinspires.ftc.teamcode.hardware.SlideConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.hardware.SlideConstants.maxTargetPosition;
import static org.firstinspires.ftc.teamcode.hardware.SlideConstants.minTargetPosition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name =  "Blue 2" )
public class AutonBlue2 extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        // listing the things
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotor slideMotor;
        ColorSensor colorSensor;
        Servo leftClaw;
        Servo rightClaw;
        double currentPosition;
        double targetPosition = 0;
        double zeroPos;
        int greenValue;
        int redValue;
        int blueValue;
        int alphaValue;
        double slidePower;
        ElapsedTime runTime = new ElapsedTime();

        // naming the things
        slideMotor = hardwareMap.get(DcMotor.class, "SM");
        leftClaw = hardwareMap.get(Servo.class, "LC");
        rightClaw = hardwareMap.get(Servo.class, "RC");
        colorSensor = hardwareMap.get(ColorSensor.class, "CS");
        colorSensor.enableLed(false);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        zeroPos = encoderTicksToInches(slideMotor.getCurrentPosition());
        leftClaw.setDirection(Servo.Direction.FORWARD);
        rightClaw.setDirection(Servo.Direction.REVERSE);
        // IMPORTANT: these are the directions we move and whether we strafe or move forward or turn. They use inches
        Trajectory moveToG3 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(15)
                .build();
        Trajectory finishG3 = drive.trajectoryBuilder(moveToG3.end())
                .forward(4)
                .build();
        Trajectory back2Start = drive.trajectoryBuilder(finishG3.end())
                .strafeLeft(14)
                .build();
        Trajectory moveToSignal = drive.trajectoryBuilder(back2Start.end())
                .forward(11)
                .build();
        Trajectory forwad4Zones = drive.trajectoryBuilder(moveToSignal.end())
                .forward(14)
                .build();
        Trajectory strafeTo1 = drive.trajectoryBuilder(forwad4Zones.end())
                .strafeLeft(27)
                .build();
        Trajectory strafeTo3 = drive.trajectoryBuilder(forwad4Zones.end())
                .strafeRight(27)
                .build();

        waitForStart();

        // closes claw
        rightClaw.setPosition(0.5);
        leftClaw.setPosition(0.4);
        sleep(500);
        // telemetry
        telemetry.addData("Alpha: ", colorSensor.alpha());
        telemetry.addData("Blue: ", colorSensor.blue());
        telemetry.addData("Red: ", colorSensor.red());
        telemetry.addData("Green: ", colorSensor.green());
        telemetry.addData("argb; ", colorSensor.argb());
        telemetry.update();
        // move slide
        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();
        while (currentPosition < 27.0 && runTime.seconds()<0.5) {
            slidePower = moveSlide(currentPosition, 27);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }
        //stop moving slide after while loop
        slideMotor.setPower(0);
        // uses trajectory from earlier to move
        drive.followTrajectory(moveToG3);
        drive.followTrajectory(finishG3);
        // opens claw
        leftClaw.setPosition(.7);
        rightClaw.setPosition(.7);
        sleep(100);
        // raise slide after putting down cone
        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();
        while (currentPosition < 10.0 && runTime.seconds()<1.5) {
            slidePower = moveSlide(currentPosition, 10);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }
        // moves back to start
        drive.followTrajectory(back2Start);
        // Goes to signal sleeve
        drive.followTrajectory(moveToSignal);
        sleep(50000000);
        redValue = colorSensor.red();
        blueValue = colorSensor.blue();
        greenValue = colorSensor.green();
        alphaValue = colorSensor.alpha();
        // prepares for the zones
        drive.followTrajectory(forwad4Zones);

        if (redValue > 80  && blueValue > 80) {
            // color green
            drive.followTrajectory(strafeTo1);
        }
        else if (redValue < 80 && blueValue < 80){
            // color purple
        }
        else if (greenValue > 60) {
            // color white
            drive.followTrajectory(strafeTo3);
        }

        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();
        while (currentPosition > 0.0 && runTime.seconds()<1.5) {
            slidePower = moveSlide(currentPosition, 0);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }

    }
    //slide variable
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
