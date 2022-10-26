package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Subsystems.Slide;
import org.firstinspires.ftc.teamcode.hardware.Subsystems.Subsystem;

import java.util.ArrayList;

import javax.xml.parsers.SAXParser;

public class RobotHardware {

    private static RobotHardware instance = new RobotHardware();

    public static RobotHardware getInstance() {
        return instance;
    }

    public SampleMecanumDrive drive = null;
    public Slide slide = null;

    public ArrayList<Subsystem> subsystems = new ArrayList<Subsystem>();

    public RobotHardware(){
        subsystems.clear();
    }

    public void initialize(HardwareMap map, Telemetry telemetry) {

        subsystems.clear();

        drive = new SampleMecanumDrive(map);
        subsystems.add(drive);

        slide = new Slide(map);
        subsystems.add(slide);
    }

    public void sendTelemetry(Telemetry telemetry) {
        for (Subsystem subsystem : subsystems) {
            subsystem.sendTelemetry(telemetry);
        }
    }
}
