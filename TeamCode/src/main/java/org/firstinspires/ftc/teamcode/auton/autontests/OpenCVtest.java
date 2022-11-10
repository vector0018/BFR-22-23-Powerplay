package org.firstinspires.ftc.teamcode.auton.autontests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name =  "OpenCV test" )
public class OpenCVtest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
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
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // With live preview
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

    }
}
