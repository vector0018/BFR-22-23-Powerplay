package org.firstinspires.ftc.teamcode.auton.autontests;
import static org.firstinspires.ftc.teamcode.hardware.SlideConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.hardware.SlideConstants.maxTargetPosition;
import static org.firstinspires.ftc.teamcode.hardware.SlideConstants.minTargetPosition;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Pipeline;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.hardware.Subsystems.Slide;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "Blue Right States")
public class StatesBlueRight extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        // listing the things
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotor slideMotor;
        Servo leftClaw;
        Servo rightClaw;
        ColorSensor colorSensor;
        DistanceSensor distanceSensor;
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

        /* IMPORTANT: these are the directions we move and whether we strafe or move forward or
         turn. They use inches. Positive turn angles are counter-clockwise, you can also use
         negative which go clockwise.
         */

        Trajectory forwardToH3 = drive.trajectoryBuilder(new Pose2d())
                .forward(60)
                .build();
        Trajectory backForH3 = drive.trajectoryBuilder(forwardToH3.end())
                .back(8)
                .build();
        Trajectory finishH3 = drive.trajectoryBuilder(backForH3.end())
                .strafeLeft(15)
                .build();
        Trajectory beginToStack = drive.trajectoryBuilder(finishH3.end())
                .strafeRight(15)
                .build();
        Trajectory finishStack = drive.trajectoryBuilder(beginToStack.end().plus(new Pose2d(0, 0, Math.toRadians(-90))), false)
                .forward(20)
                .build();
        Trajectory backTowardsL3 = drive.trajectoryBuilder(finishStack.end())
                .back(19)
                .build();
        Trajectory finishL3 = drive.trajectoryBuilder(backTowardsL3.end())
                .strafeRight(15)
                .build();
        Trajectory backAwayFromL3 = drive.trajectoryBuilder(finishL3.end())
                .back(2)
                .build();
        Trajectory strafeToParkPos = drive.trajectoryBuilder(backAwayFromL3.end())
                .strafeRight(15)
                .build();
        Trajectory zone1 = drive.trajectoryBuilder(strafeToParkPos.end().plus(new Pose2d(0, 0, Math.toRadians(90))), false)
                .strafeLeft(28)
                .build();
        Trajectory zone3 = drive.trajectoryBuilder(strafeToParkPos.end().plus(new Pose2d(0, 0, Math.toRadians(90))), false)
                .strafeRight(28)
                .build();
        waitForStart();

        // Gets sleeve value
        pipelineValue = pipeline.getMeanCbValue();
        telemetry.addData("Pipleine value", pipelineValue);
        telemetry.update();

        // Closes Claw
        rightClaw.setPosition(1);
        leftClaw.setPosition(0.3);
        sleep(450);

        // Raise slide - we only do .01 seconds because starts raising slide, after .01 seconds the robot will move as slide is raised.
        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();
        while (currentPosition < 42.0 && runTime.seconds()<.01) {
            slidePower = moveSlide(currentPosition, 42);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }
        // Goes to H3
        drive.followTrajectory(forwardToH3);
        // Heads back 5 inches because we went to far so we can re-align ourselves
        drive.followTrajectory(backForH3);
        // Goes to right so its in front of H3
        drive.followTrajectory(finishH3);

        // lowers slide for cone placing
        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();
        while (currentPosition > 29.0 && runTime.seconds()<.50) {
            slidePower = moveSlide(currentPosition, 29);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }
        // Open Claw
        leftClaw.setPosition(.6);
        rightClaw.setPosition(.7);

        // Begins to stack
        drive.followTrajectory(beginToStack);

        drive.turn(Math.toRadians(-90));
        // Closes the claw a bit to avoid hitting the claw
        rightClaw.setPosition(.8);
        leftClaw.setPosition(.45);

        // Lowers the slide for the cone
        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();
        while (currentPosition > 6.0 && runTime.seconds()<.75) {
            slidePower = moveSlide(currentPosition, 6);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }

        // Heads to stack
        drive.followTrajectory(finishStack);

        // Closes Claw
        rightClaw.setPosition(1);
        leftClaw.setPosition(0.3);
        sleep(450);

        // Raises the slide for the Junction
        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();
        while (currentPosition < 14.0 && runTime.seconds()<.75) {
            slidePower = moveSlide(currentPosition, 14);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }
        drive.followTrajectory(backTowardsL3);
        drive.followTrajectory(finishL3);
        // Raises the slide for the Junction
        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();
        while (currentPosition > 5 && runTime.seconds()<.75) {
            slidePower = moveSlide(currentPosition, 5);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }

        // Open Claw
        leftClaw.setPosition(.5);
        rightClaw.setPosition(.8);

        drive.followTrajectory(backAwayFromL3);
        drive.followTrajectory(strafeToParkPos);

        // Lowers the slide for the cone
        currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        runTime.reset();
        while (currentPosition > 0 && runTime.seconds()<.01) {
            slidePower = moveSlide(currentPosition, 0);
            slideMotor.setPower(slidePower);
            currentPosition = encoderTicksToInches(slideMotor.getCurrentPosition()) - zeroPos;
        }

        drive.turn(Math.toRadians(90));

        // park stuff
        if  (pipelineValue <= 138){
            // color green
            drive.followTrajectory(zone1);
        }
        else if (pipelineValue >= 153){
            // color Purple
        }
        else if (pipelineValue < 153 && pipelineValue >138){
            // color pink
            drive.followTrajectory(zone3);
        }
        slideMotor.setPower(0);
        sleep(500);
    }



    //calculate slide power
    private double moveSlide (double currentPosition , double targetPosition) {
        double Kp = 0.5;
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
