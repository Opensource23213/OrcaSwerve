package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Turret Servo Test", group="ABC Opmode")

public class TurretServoTest extends DecodeLibrary{
    public static double test_pos = .5;
    public static double offset1 = 0;
    public static double offset2 = 0;
    public Servo turret_servo_1;
    public Servo turret_servo_2;
    public AnalogInput servo_pose;
    @Override
    public void init(){
        turret_servo_1 = hardwareMap.get(Servo.class, "turret_servo_1");
        turret_servo_2 = hardwareMap.get(Servo.class, "turret_servo_2");
        servo_pose = hardwareMap.get(AnalogInput.class, "servo_pose");
        turret_servo_1.setDirection(Servo.Direction.REVERSE);
        turret_servo_2.setDirection(Servo.Direction.REVERSE);

    }
    @Override
    public void loop(){
        turret_servo_1.setPosition(test_pos + offset1);
        turret_servo_2.setPosition(test_pos + offset2);
        telemetry.addData("Servo Position", test_pos);
        telemetry.addData("servo_pose", (1.632 - servo_pose.getVoltage()) / .517 * 90);
        telemetry.update();
    }

}
