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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Pipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name =  "Blue Red Term" )
public class Blue_redterm extends LinearOpMode {

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
        Pose2d poseEstimate;

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
        rightClaw.setDirection(Servo.Direction.FORWARD);
        // camera stuff
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // With live preview
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
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

        // IMPORTANT: these are the directions we move and whether we strafe or move forward or
        // turn. They use inches. For spline to line heading X and Y are inverted from normal
        // coordinate plane
        // in this auton x is negative and y is positive
        Trajectory moveToH3 = drive.trajectoryBuilder(new Pose2d(-63 , 36 , 0))
                .forward(40)
                .splineToLinearHeading(new Pose2d(-14,24), 0)
                .build();
        Trajectory beginToStack = drive.trajectoryBuilder(moveToH3.end())
                .splineToLinearHeading(new Pose2d(-14, 36, Math.toRadians(0)), 0)
                .build();
        Trajectory ToCone = drive.trajectoryBuilder(beginToStack.end())
                .splineToLinearHeading(new Pose2d(-12, 55, Math.toRadians(-90)), 0)
                .build();
        Trajectory RedoH2 = drive.trajectoryBuilder(ToCone.end())
                .splineToLinearHeading(new Pose2d(-14, 36, Math.toRadians(-90)), 0)
                .build();
        Trajectory finishH2PT2 = drive.trajectoryBuilder(RedoH2.end())
                .splineToLinearHeading(new Pose2d(-14,24, Math.toRadians(0)), 0)
                .build();
        Trajectory ToCone2 = drive.trajectoryBuilder(finishH2PT2.end())
                .splineToLinearHeading(new Pose2d(-14, 36, Math.toRadians(0)), 0)
                .build();
        Trajectory headingToStack = drive.trajectoryBuilder(ToCone2.end())
                .splineToLinearHeading(new Pose2d(-12, 55, Math.toRadians(-90)), 0)
                .build();
        Trajectory ToCone3 = drive.trajectoryBuilder(headingToStack.end())
                .splineToLinearHeading(new Pose2d(-12, 55, Math.toRadians(-90)), 0)
                .build();
        Trajectory zone1 = drive.trajectoryBuilder(ToCone3.end())
                .splineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(0)),0)
                .build();
        Trajectory zone2 = drive.trajectoryBuilder(ToCone3.end())
                .splineToLinearHeading(new Pose2d(-36, -36, Math.toRadians(0)),0)
                .build();
        Trajectory zone3 = drive.trajectoryBuilder(ToCone3.end())
                .splineToLinearHeading(new Pose2d(-36, -60, Math.toRadians(0)),0)
                .build();
        waitForStart();
        drive.setPoseEstimate(new Pose2d(-63, -36 , 0));
        // Gets sleeve value
        pipelineValue = pipeline.getMeanCbValue();
        telemetry.addData("Pipleine value", pipelineValue);
        telemetry.update();
        //close claw
        rightClaw.setPosition(1);
        leftClaw.setPosition(0.3);
        sleep(450);
        // Raise slide
        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();
        while (currentPosition < 42.0 && runTime.seconds()<.5) {
            slidePower = moveSlide(currentPosition, 42);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }
        // Goes to the high junction 3
        drive.followTrajectory(moveToH3);
        // Lowers slide
        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();
        while (currentPosition > 29.0 && runTime.seconds()<.75) {
            slidePower = moveSlide(currentPosition, 29);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }
        // opens claw
        leftClaw.setPosition(.6);
        rightClaw.setPosition(.7);
        // We center on the tile so we can turn without hitting anything
        drive.followTrajectory(beginToStack);
        // lowers slide before we turn so we don't hit anything
        // Closes claw slightly so we don't hit the phone
        rightClaw.setPosition(.8);
        leftClaw.setPosition(.45);
        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();
        while (currentPosition > 5.0 && runTime.seconds()<1) {
            slidePower = moveSlide(currentPosition, 5);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }
        // heads to cones
        drive.followTrajectory(ToCone);
        // closes the claw
        rightClaw.setPosition(1);
        leftClaw.setPosition(0.3);
        sleep(500);
        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();
        while (currentPosition < 13 && runTime.seconds()<0.6) {
            slidePower = moveSlide(currentPosition, 13);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }
        // heads to L3
        drive.followTrajectory(BeginL3);
        drive.followTrajectory(FinishL3);
        //Lowers the slide so we can place a cone
        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();
        while (currentPosition > 5.0 && runTime.seconds()<.5) {
            slidePower = moveSlide(currentPosition, 5);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }
        //opens the claw
        leftClaw.setPosition(.6);
        rightClaw.setPosition(.7);
        // lowers the slide to get the cone more consistent
        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();
        while (currentPosition < 7 && runTime.seconds()<.6) {
            slidePower = moveSlide(currentPosition, 7);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }
        // heads back to the cones
        drive.followTrajectory(ToCone2);
        // closes the claw
        leftClaw.setPosition(.3);
        rightClaw.setPosition(1);
        sleep(350);
        // raises the slide all the way
        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();
        while (currentPosition < 42.0 && runTime.seconds()<1) {
            slidePower = moveSlide(currentPosition, 42);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }
        // heads to the high Junction 1
        drive.followTrajectory(BeganH1);
        // finishes heading to the High junction 1
        drive.followTrajectory(finishH1);
        // lowers the slide for half a second
        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();
        while (currentPosition > 15.0 && runTime.seconds()<.5) {
            slidePower = moveSlide(currentPosition, 15);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }
        //opens the claw
        leftClaw.setPosition(.5);
        rightClaw.setPosition(.7);
        // lowers the slide So its not messed up in teleop
        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();
        while (currentPosition > 0 && runTime.seconds()<2) {
            slidePower = moveSlide(currentPosition, 0);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }

        // park stuff
        if  (pipelineValue <= 138){
            // color green
            drive.followTrajectory(zone1);
        }
        else if (pipelineValue >= 147){
            // color Purple
            drive.followTrajectory(zone2);
        }
        else if (pipelineValue < 147 && pipelineValue >138){
            // color pink
            drive.followTrajectory(zone3);
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