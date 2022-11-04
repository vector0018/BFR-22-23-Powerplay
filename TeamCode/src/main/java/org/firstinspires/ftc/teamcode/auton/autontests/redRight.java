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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name =  "Red Right" )
public class redRight extends LinearOpMode {

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
        DistanceSensor distanceSensor;
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
        distanceSensor = hardwareMap.get(DistanceSensor.class , "CS");
        colorSensor.enableLed(false);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        zeroPos = encoderTicksToInches(slideMotor.getCurrentPosition());
        leftClaw.setDirection(Servo.Direction.FORWARD);
        rightClaw.setDirection(Servo.Direction.REVERSE);
        // IMPORTANT: these are the directions we move and whether we strafe or move forward or turn. They use inches
        Trajectory moveToSignal = drive.trajectoryBuilder(new Pose2d())
                .forward(15.393701)
                .build();
        Trajectory forwad4Zones = drive.trajectoryBuilder(moveToSignal.end())
                .forward(14)
                .build();
        Trajectory moveToM1 = drive.trajectoryBuilder(forwad4Zones.end())
                .strafeLeft(15)
                .build();
        Trajectory strafeTo1 = drive.trajectoryBuilder(moveToM1.end())
                .strafeLeft(12)
                .build();
        Trajectory backTo2 = drive.trajectoryBuilder(moveToM1.end())
                .strafeRight(15)
                .build();
        Trajectory strafeTo3 = drive.trajectoryBuilder(moveToM1.end())
                .strafeRight(4+
                        2)
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
        telemetry.addData("target position: ", targetPosition);

        telemetry.update();
        // move slide
        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();
        while (currentPosition < 38.0 && runTime.seconds()<2) {
            slidePower = moveSlide(currentPosition, 38);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }
        //stop moving slide after while loop
        slideMotor.setPower(0);

        // Goes to signal sleeve
        drive.followTrajectory(moveToSignal);
        redValue = colorSensor.red();
        blueValue = colorSensor.blue();
        greenValue = colorSensor.green();
        alphaValue = colorSensor.alpha();
        telemetry.addData("Blue: ", colorSensor.blue());
        telemetry.addData("Red: ", colorSensor.red());
        telemetry.addData("Green: ", colorSensor.green());
        telemetry.addData("argb" , colorSensor.argb());
        telemetry.addData("Distance: " , distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.update();
        // prepares for the zones
        drive.followTrajectory(forwad4Zones);
        // uses trajectory from earlier to move
        drive.followTrajectory(moveToM1);
        sleep(500);
        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();

        while (currentPosition > 15.0 && runTime.seconds()<5) {
            slidePower = moveSlide(currentPosition, 15);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }
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

        if (blueValue < 75 && greenValue > 85) {
            // color green
            drive.followTrajectory(strafeTo1);
        }
        else if (blueValue < 75 && greenValue < 90){
            // color brown
            drive.followTrajectory(backTo2);
        }
        else if (blueValue > 100 && redValue > 70) {
            // color pink
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