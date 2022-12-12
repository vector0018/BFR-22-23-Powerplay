
package org.firstinspires.ftc.teamcode.teleop.teleoptests;

import static org.firstinspires.ftc.teamcode.hardware.SlideConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.hardware.SlideConstants.maxTargetPosition;
import static org.firstinspires.ftc.teamcode.hardware.SlideConstants.minTargetPosition;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp (name = "Field Sentric but I can't spell")
public class fieldSentricPart2 extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotor slideMotor;
        Servo leftClaw;
        Servo rightClaw;
        ColorSensor colorSensor;
        double Kp = 0.2;
        double currentPosition;
        double targetPosition = 0;
        double positionError;
        double slidePower;
        double zeroPos;
        double xVal;
        double yVal;
        double angleVal;
        //myLocalizer.setPoseEstimate(org.firstinspires.ftc.teamcode.drive.advanced.TransferPose.currentPose);


        slideMotor = hardwareMap.get(DcMotor.class, "SM");
        leftClaw = hardwareMap.get(Servo.class, "LC");
        rightClaw = hardwareMap.get(Servo.class, "RC");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "CS");
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        zeroPos = encoderTicksToInches(slideMotor.getCurrentPosition());
        leftClaw.setDirection(Servo.Direction.FORWARD);
        rightClaw.setDirection(Servo.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            xVal = -gamepad1.left_stick_y;
            yVal = -gamepad1.left_stick_x;
            angleVal = -gamepad1.right_stick_x;

            if (gamepad1.left_trigger > .1 || gamepad1.right_trigger > .1){
                xVal *= 0.3;
                yVal *= 0.3;
                angleVal *= 0.3;
            }

            Vector2d input = new Vector2d(
                    xVal,
                    yVal)
                    .rotated(-drive.getRawExternalHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            angleVal
                    )
            );
            // add 0.2 inches to target position when left stick is pushed up or down
            if (Math.abs(gamepad2.left_stick_y) > .01) {
                targetPosition += gamepad2.left_stick_y * -1;
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
            slidePower = Kp * positionError;

            telemetry.addData("target position: ", targetPosition);
            telemetry.addData("position error: ", positionError);
            telemetry.addData("current position", currentPosition);
            telemetry.addData("slide power: ", slidePower);
            telemetry.addData("Zero Position: ", zeroPos);
            telemetry.addData("Right servo: ", rightClaw.getPosition());
            telemetry.addData("Left servo: ", leftClaw.getPosition());
            telemetry.addData("Alpha: ", colorSensor.alpha());
            telemetry.addData("Blue: ", colorSensor.blue());
            telemetry.addData("Red: ", colorSensor.red());
            telemetry.addData("Green: ", colorSensor.green());
            telemetry.addData("argb" , colorSensor.argb());
            telemetry.addData("left stick y", gamepad2.left_stick_y);

            telemetry.update();

            //power safeties
            if (slidePower > 1) {
                slidePower = 1;
            }
            if (slidePower < -1) {
                slidePower = -1;
            }
            slideMotor.setPower(slidePower);

            //open claw
            if (gamepad2.left_bumper) {
                leftClaw.setPosition(.6);
            }
            if (gamepad2.left_bumper) {
                rightClaw.setPosition(.7);
            }
            //close claw
            if (gamepad2.right_bumper) {
                rightClaw.setPosition(1);
            }
            if (gamepad2.right_bumper) {
                leftClaw.setPosition(0.3);
            }
            // Open claw all the way
            if (gamepad2.left_trigger > .1 || gamepad2.right_trigger > .1){
                leftClaw.setPosition(.8);
                rightClaw.setPosition(.6);
            }
        }
    }
}