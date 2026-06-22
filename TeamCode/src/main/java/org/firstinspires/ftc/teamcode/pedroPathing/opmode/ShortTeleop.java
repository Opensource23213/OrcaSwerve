package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name="ShortTeleop", group="ABC Opmode")
public class ShortTeleop extends DecodeLibrary {
    public boolean start = false;
    double times = 0;
    boolean back = false;
    public static double amps = 2000;
    public static double loop_time = 100;


    @Override
    public void init(){
        teleop = true;
        initialize();
        drive.pinpoint.setPosition(auto_pose);
        short_match();
    }

    @Override
    public void loop(){
        teleop_loop();

    }
}
