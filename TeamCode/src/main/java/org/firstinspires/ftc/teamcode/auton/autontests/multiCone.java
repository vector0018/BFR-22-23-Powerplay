package org.firstinspires.ftc.teamcode.auton.autontests;

import static org.firstinspires.ftc.teamcode.hardware.SlideConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.hardware.SlideConstants.maxTargetPosition;
import static org.firstinspires.ftc.teamcode.hardware.SlideConstants.minTargetPosition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
import org.firstinspires.ftc.teamcode.hardware.Pipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name =  "Multi Cone" )
public class multiCone extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        // listing the things
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotor slideMotor;
        ColorSensor colorSensor;
        DistanceSensor distanceSensor;
        Servo leftClaw;
        Servo rightClaw;
        Pipeline pipeline;
        double currentPosition;
        double targetPosition = 0;
        double zeroPos;
        int pipelineValue;
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

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // With live preview
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });
        pipeline = new Pipeline();
        camera.setPipeline(pipeline);
        drive.setPoseEstimate(new Pose2d(-36, -72 , 0));

        // IMPORTANT: these are the directions we move and whether we strafe or move forward or turn. They use inches
        Trajectory moveToH1 = drive.trajectoryBuilder(new Pose2d(-36 , -72 , 0))
                .forward(54)
                .build();
        Trajectory FinishH1 = drive.trajectoryBuilder(moveToH1.end())
                .strafeLeft(15)
                .build();
        Trajectory ToCone = drive.trajectoryBuilder(FinishH1.end())
                .splineTo(new Vector2d(-70, 12 ),Math.toRadians(-90))
                .build();
        waitForStart();

        pipelineValue = pipeline.getMeanCbValue();

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
        while (currentPosition < 42.0 && runTime.seconds()<1) {
            slidePower = moveSlide(currentPosition, 42);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }

        // Goes to signal sleeve
        drive.followTrajectory(moveToH1);
        drive.followTrajectory(FinishH1);
        redValue = colorSensor.red();
        blueValue = colorSensor.blue();
        greenValue = colorSensor.green();
        alphaValue = colorSensor.alpha();
        telemetry.addData("Blue: ", colorSensor.blue());
        telemetry.addData("Red: ", colorSensor.red());
        telemetry.addData("Green: ", colorSensor.green());
        telemetry.addData("argb" , colorSensor.argb());
        telemetry.addData("Distance: " , distanceSensor.getDistance(DistanceUnit.CM));
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();
        sleep(1000);
        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();

        while (currentPosition > 27.0 && runTime.seconds()<.75) {
            slidePower = moveSlide(currentPosition, 27);
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
        while (currentPosition < 42 && runTime.seconds()<1.5) {
            slidePower = moveSlide(currentPosition, 42);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }
        drive.followTrajectory(ToCone);
        /*
        while (currentPosition < 9.0 && runTime.seconds()<1.5) {
            slidePower = moveSlide(currentPosition, 9);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }
        // closes claw
        rightClaw.setPosition(0.5);
        leftClaw.setPosition(0.4);
        sleep(100);
        while (currentPosition < 42.0 && runTime.seconds()<1.5) {
            slidePower = moveSlide(currentPosition, 42);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }

          if (pipelineValue < 130){
            // TODO: Park in zone 1
        }
        else if (pipelineValue >= 130 && pipelineValue < 150){
            // todo: park in zone 2
        }
        else if (pipelineValue >= 150){
            // todo: park in zone 3
      }

        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();
        while (currentPosition > 0.0 && runTime.seconds()<1.5) {
            slidePower = moveSlide(currentPosition, 0);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }

         */

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
