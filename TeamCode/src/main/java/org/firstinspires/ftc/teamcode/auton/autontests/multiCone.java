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
        drive.setPoseEstimate(new Pose2d(-63, -36 , 0));
        // IMPORTANT: these are the directions we move and whether we strafe or move forward or
        // turn. They use inches. For spline to line heading X and Y are inverted from normal
        // coordinate plane
        Trajectory forwardToH3 = drive.trajectoryBuilder(new Pose2d(-63 , -36 , 0))
                .forward(40)
                .build();
        Trajectory moveToH3 = drive.trajectoryBuilder(forwardToH3.end())
                .splineToLinearHeading(new Pose2d(-12,-24,0), 0)
                .build();
        Trajectory beginToStack = drive.trajectoryBuilder(moveToH3.end())
                .splineToLinearHeading(new Pose2d(-15, -36, Math.toRadians(0)), 0)
                .build();
//        Trajectory ToCone = drive.trajectoryBuilder(beginToStack.end())
//                .splineToLinearHeading(new Pose2d(-12, -60, Math.toRadians(-85)), 0)
//                .build();
//        Trajectory BeginL3 = drive.trajectoryBuilder(ToCone.end())
//                .splineToLinearHeading(new Pose2d(-12, -48, Math.toRadians(-85)), 0)
//                .build();
//        Trajectory FinishL3 = drive.trajectoryBuilder(BeginL3.end())
//                .splineToLinearHeading(new Pose2d(-12, -40, Math.toRadians(-180)), 0)
//                .build();
//        Trajectory BeganH1 = drive.trajectoryBuilder(ToCone.end())
//                .splineToLinearHeading(new Pose2d(-12, -24, Math.toRadians(90)), 0)
//                .build();
//        Trajectory finishH1 = drive.trajectoryBuilder(BeganH1.end())
//                .splineToLinearHeading(new Pose2d(-24, -12, Math.toRadians(90)),0)
//                .build();
//        Trajectory Backwards = drive.trajectoryBuilder(finishH1.end())
//                .back(20)
//                .build();
//        Trajectory BeginL2 = drive.trajectoryBuilder(ToCone.end())
//                .splineToLinearHeading(new Pose2d(-12, -60, Math.toRadians(90)),0)
//                .build();
//        Trajectory FinishL2 = drive.trajectoryBuilder(BeginL2.end())
//                .splineToLinearHeading(new Pose2d(-48,-30, Math.toRadians(180)),0)
//                .build();
//        Trajectory zone1 = drive.trajectoryBuilder(BeginL2.end())
//                .splineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(0)),0)
//                .build();
//        Trajectory zone2 = drive.trajectoryBuilder(BeginL2.end())
//                .splineToLinearHeading(new Pose2d(-36, -36, Math.toRadians(0)),0)
//                .build();
//        Trajectory zone3 = drive.trajectoryBuilder(BeginL2.end())
//                .splineToLinearHeading(new Pose2d(-36, -60, Math.toRadians(0)),0)
//                .build();
        waitForStart();
        // Gets sleeve value
        pipelineValue = pipeline.getMeanCbValue();
        //close claw
        rightClaw.setPosition(1);
        leftClaw.setPosition(0.3);
        sleep(300);
        // Raise slide
        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();
        while (currentPosition < 42.0 && runTime.seconds()<2) {
            slidePower = moveSlide(currentPosition, 42);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }
        // Goes to the high junction 3
        drive.followTrajectory(forwardToH3);
        drive.followTrajectory(moveToH3);
        // opens claw
        leftClaw.setPosition(.6);
        rightClaw.setPosition(.7);
        // there isn't a sleep here because we open the claw on the way down to save time
        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();
        while (currentPosition > 15.0 && runTime.seconds()<.75) {
            slidePower = moveSlide(currentPosition, 15);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }
        // We center on the tile so we can turn without hiting anything
        drive.followTrajectory(beginToStack);
        // lowers slide before we turn so we don't hit anything
//        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
//        runTime.reset();
//        while (currentPosition > 10.0 && runTime.seconds()<2) {
//            slidePower = moveSlide(currentPosition, 10);
//            slideMotor.setPower(slidePower);
//            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
//        }
//        // heads to cones
//        drive.followTrajectory(ToCone);
//        // closes the claw
//        rightClaw.setPosition(0.5);
//        leftClaw.setPosition(0.4);
//        // heads to L3
//        drive.followTrajectory(BeginL3);
//        drive.followTrajectory(FinishL3);
//        //opens the claw
//        leftClaw.setPosition(.6);
//        rightClaw.setPosition(.7);
//        // lowers the slide to get the cone more consistent
//        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
//        runTime.reset();
//        while (currentPosition > 9.0 && runTime.seconds()<.5) {
//            slidePower = moveSlide(currentPosition, 9);
//            slideMotor.setPower(slidePower);
//            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
//        }
//        // heads back to the cones
//        drive.followTrajectory(beginToStack);
//        drive.followTrajectory(ToCone);
//        // closes the claw
//        leftClaw.setPosition(.5);
//        rightClaw.setPosition(.4);
//        // heads to the high Junction 1
//        drive.followTrajectory(BeganH1);
//        // raises the slide all the way
//        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
//        runTime.reset();
//        while (currentPosition < 42.0 && runTime.seconds()<1) {
//            slidePower = moveSlide(currentPosition, 42);
//            slideMotor.setPower(slidePower);
//            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
//        }
//        // finishes heading to the High junction 1
//        drive.followTrajectory(finishH1);
//        // lowers the slide for half a second
//        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
//        runTime.reset();
//        while (currentPosition > 15.0 && runTime.seconds()<.5) {
//            slidePower = moveSlide(currentPosition, 15);
//            slideMotor.setPower(slidePower);
//            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
//        }
//        // heads back to cones
//        drive.followTrajectory(BeganH1);
//        drive.followTrajectory(ToCone);
//        // lowers the slide so we can pick up the cone
//        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
//        runTime.reset();
//        while (currentPosition > 10 && runTime.seconds()<2) {
//            slidePower = moveSlide(currentPosition, 10);
//            slideMotor.setPower(slidePower);
//            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
//        }
//        // closes claw
//        rightClaw.setPosition(0.5);
//        leftClaw.setPosition(0.4);
//        // raises slide
//        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
//        runTime.reset();
//        while (currentPosition < 15 && runTime.seconds()<.5) {
//            slidePower = moveSlide(currentPosition, 15);
//            slideMotor.setPower(slidePower);
//            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
//        }
//        // heads to L2
//        drive.followTrajectory(Backwards);
//        drive.followTrajectory(BeginL2);
//        drive.followTrajectory(FinishL2);
//        // lowers the slide to 10 inches
//        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
//        runTime.reset();
//        while (currentPosition > 10 && runTime.seconds()<.5) {
//            slidePower = moveSlide(currentPosition, 10);
//            slideMotor.setPower(slidePower);
//            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
//        }
//        // opens the claw
//        leftClaw.setPosition(.6);
//        rightClaw.setPosition(.7);
//        // park stuff
//        if (pipelineValue ==  000){
//            drive.followTrajectory(zone1);
//        }
//        else if (pipelineValue == 000){
//            drive.followTrajectory(zone2);
//        }
//        else if (pipelineValue == 000){
//            drive.followTrajectory(zone3);
//        }
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