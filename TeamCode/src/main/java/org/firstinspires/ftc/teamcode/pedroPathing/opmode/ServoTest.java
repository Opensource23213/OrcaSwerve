package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Servo Test", group="ABC Opmode")

public class ServoTest extends OpMode {
    public static double test_pos = .5;
    public static String name = "blocker";
    public Servo Servo;
    @Override
    public void init(){
        Servo = hardwareMap.get(Servo.class, name);
    }
    @Override
    public void loop(){
        Servo.setPosition(test_pos);
        telemetry.addData("Servo Position", test_pos);
        telemetry.update();
    }

}
