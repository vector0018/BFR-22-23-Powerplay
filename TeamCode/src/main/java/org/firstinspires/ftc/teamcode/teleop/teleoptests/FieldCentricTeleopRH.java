package org.firstinspires.ftc.teamcode.teleop.teleoptests;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.hardware.Subsystems.Slide;
@Disabled
@TeleOp (name = "Field Centric but I can spell")
public class FieldCentricTeleopRH extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = RobotHardware.getInstance();
        SampleMecanumDrive drive = robot.drive;
        Slide slide = robot.slide;
        Claw claw = robot.claw;

        double xVal;
        double yVal;
        double angleVal;

        double cntrlrTargetPosition = 0;

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

            if (Math.abs(gamepad2.left_stick_y) > .01) {
                cntrlrTargetPosition += gamepad2.left_stick_y * -1;
                robot.slide.moveSlideWithoutTime(cntrlrTargetPosition);
            }
            else if (gamepad2.a){
                cntrlrTargetPosition = 2;
                robot.slide.moveSlideWithoutTime(cntrlrTargetPosition);
            }
            //low junction
            else if (gamepad2.b){
                cntrlrTargetPosition = 15.5;
                robot.slide.moveSlideWithoutTime(cntrlrTargetPosition);
            }
            //medium junction
            else if (gamepad2.y) {
                cntrlrTargetPosition = 25.45;
                robot.slide.moveSlideWithoutTime(cntrlrTargetPosition);
            }
            //high junction
            else if (gamepad2.x){
                cntrlrTargetPosition = 35.5;
                robot.slide.moveSlideWithoutTime(cntrlrTargetPosition);
            }

            robot.claw.moveClaw(gamepad2.right_bumper, gamepad2.left_bumper, gamepad1.left_trigger, gamepad2.right_trigger);
        }
    }
}
