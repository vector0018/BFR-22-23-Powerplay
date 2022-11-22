package org.firstinspires.ftc.teamcode.auton.autontests;

import static org.firstinspires.ftc.teamcode.hardware.SlideConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.hardware.SlideConstants.maxTargetPosition;
import static org.firstinspires.ftc.teamcode.hardware.SlideConstants.minTargetPosition;

import android.app.TaskInfo;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
@Disabled
@Autonomous(name =  "Blue Term" )
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
                .forward(48)
                .splineToLinearHeading(new Pose2d(0,24, Math.toRadians(0)), 0)
                .build();
        Trajectory beginToStack = drive.trajectoryBuilder(moveToH1.end())
                .splineToLinearHeading(new Pose2d(-3, 28.5, Math.toRadians(0)), 0)
                .build();
        Trajectory ToCone = drive.trajectoryBuilder(beginToStack.end())
                .splineToLinearHeading(new Pose2d(-3, -9, Math.toRadians(-85)), 0)
                .build();
        Trajectory BeginL3 = drive.trajectoryBuilder(ToCone.end())
                .splineToLinearHeading(new Pose2d(-3, 28.5, Math.toRadians(-85)), 0)
                .build();
        Trajectory FinishL3 = drive.trajectoryBuilder(BeginL3.end())
                .splineToLinearHeading(new Pose2d(-23, 28.5, Math.toRadians(-85)), 0)
                .build();
        Trajectory BeganH1 = drive.trajectoryBuilder(FinishL3.end())
                .splineToLinearHeading(new Pose2d(24, -12, Math.toRadians(-180)), 0)
                .build();
        waitForStart();
        pipelineValue = pipeline.getMeanCbValue();
        //close claw
            rightClaw.setPosition(0.5);
            leftClaw.setPosition(0.4);
        sleep(300);

        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();
        while (currentPosition < 42.0 && runTime.seconds()<2) {
            slidePower = moveSlide(currentPosition, 42);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }

        drive.followTrajectory(moveToH1);

            leftClaw.setPosition(.6);
            rightClaw.setPosition(.7);

        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();
        while (currentPosition > 15.0 && runTime.seconds()<.75) {
            slidePower = moveSlide(currentPosition, 15);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }

        drive.followTrajectory(beginToStack);

        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();
        while (currentPosition > 10.0 && runTime.seconds()<2) {
            slidePower = moveSlide(currentPosition, 10);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }

        drive.followTrajectory(ToCone);

            rightClaw.setPosition(0.5);
            leftClaw.setPosition(0.4);

        drive.followTrajectory(BeginL3);
        drive.followTrajectory(FinishL3);

        leftClaw.setPosition(.6);
        rightClaw.setPosition(.7);

        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();
        while (currentPosition > 10.0 && runTime.seconds()<2) {
            slidePower = moveSlide(currentPosition, 10);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }

        rightClaw.setPosition(0.5);
        leftClaw.setPosition(0.4);

        drive.followTrajectory(beginToStack);
        drive.followTrajectory(ToCone);

        leftClaw.setPosition(.6);
        rightClaw.setPosition(.7);

        drive.followTrajectory(BeganH1);
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
