package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import static java.lang.Math.abs;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.BlueSwerveConst;

@Autonomous(name="SafeAuto", group="ABC Opmode", preselectTeleOp = "DecodeTeleop")
public class SafeAuto extends DecodeLibrary{
    public double forward = 0.25;
    public Path first_pick;
    public Path pick_after_stuff;
    public Path first_shoot;
    public Path second_shoot;
    public Path third_shoot;
    public Path fourth_shoot;
    public Path fifth_shoot;
    public Path second_pick;
    public Path third_pick;
    public Path park;
    public Path gate;
    public double shot = 0;
    public double old_turns = 0;
    public double steps = 0;
    public ElapsedTime gate_open = new ElapsedTime();
    public ElapsedTime shooting_time = new ElapsedTime();
    public double sort_pos = 0;
    public double old_pattern = 2;
    public double old_color = 1;
    public boolean shooting = false;
    public double index_steps = 0;
    public static double first_dump_time = 7000;
    public static double second_dump_time = 800;
    public static double third_dump_time = 800;
    public static double auto_shoot_speed = 1400;
    public double sort_times = 0;
    @Override
    public void init(){
        color = 1;
        teleop = false;
        if(color == 1){
            auto_pose =new Pose(122,-25, Math.toRadians(-90));
        }else{
            auto_pose = new Pose(122,25, Math.toRadians(90));
        }
        initialize();
        follower.setPose(auto_pose);
        if(color == 0) {
            blue_init();
        }else{
            red_init();
        }
        follower.setMaxPower(1);
        follower.setMaxPowerScaling(1);
        follower.followPath(first_shoot);
        intake_system.flippy_pos = flippy_hold;
    }
    @Override
    public void init_loop(){
        for(LynxModule hub : hubs){
            hub.clearBulkCache();
        }
        x_mod = -8;
        turret.zero = true;
        turret.turret_move();
        cam.look();
        if(cam.tags.contains(21.0)){
            pattern = 1;
        }else if(cam.tags.contains(22.0)){
            pattern = 2;
        }else if(cam.tags.contains(23.0)){
            pattern = 3;
        }
        follower.drivetrain.setYVelocity(0);
        follower.update();
        if(gamepad1.right_bumper) {
            color = 0;
        }else if(gamepad1.left_bumper){
            color = 1;
        }
        if(old_color != color || gamepad1.a) {
            initialize();
            if(color == 1){
                auto_pose =new Pose(122,-25, Math.toRadians(-90));
            }else{
                auto_pose = new Pose(122,25, Math.toRadians(90));
            }
            follower.setMaxPower(1);
            follower.setMaxPowerScaling(1);
            follower.setPose(auto_pose);
            if (color == 0) {
                blue_init();
                old_color = 0;
            } else {
                red_init();
                old_color = 1;
            }

            follower.followPath(first_shoot);
        }
        auto_pose = follower.getPose();
        telemetry.addData("color", color);
        telemetry.addData("pattern", pattern);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }

    @Override
    public void loop() {
        for(LynxModule hub : hubs){
            hub.clearBulkCache();
        }
        follower.update();
        turret.turret_move();
        shooter.set_speed = shoot_multiplier * ((dead_distance * .0254) - 1.6) + shoot_power_offset + (speed_increase * abs(turret.target_angle - .5));
        shooter.set_speed = Math.round(shooter.set_speed / 20) * 20 + 80;
        shooter.shooting();
        intake_system.auto();
        intake_system.sort();
        intake_system.sort_twice();
        auto_pose = follower.getPose();
        sort();
        if(follower.atParametricEnd() || !follower.isBusy() || follower.isRobotStuck()) {
            if (forward == .25) {
                shoot();
                shooting = true;
                if(steps == 2){
                    shooting = false;
                    if(pattern == 2){
                        sort_times = 1;
                    }else if(pattern == 3){
                        sort_times = 2;
                    }
                    follower.followPath(second_pick);
                    forward = .5;
                    steps = 0;
                }
            }else if (forward == .5) {
                follower.constants.automaticHoldEnd = true;
                follower.update();
                follower.followPath(gate);
                forward = .75;
            }else if(forward == .75){
                if(steps == 0){
                    gate_open.reset();
                    steps = 1;
                }else if(steps == 1 && gate_open.milliseconds() > first_dump_time) {
                    follower.constants.automaticHoldEnd = false;
                    follower.update();
                    follower.followPath(third_shoot);
                    forward = .8;
                    steps = 0;
                }
            }else if (forward == .8) {
                shoot();
                shooting = true;
                if(steps == 2){
                    shooting = false;
                    if(pattern == 3){
                        sort_times = 1;
                    }else if(pattern == 1){
                        sort_times = 2;
                    }
                    follower.followPath(first_pick);
                    forward = 1;
                    steps = 0;
                }
            }else if (forward == 1) {
                follower.followPath(second_shoot);
                forward = 2;
            }else if(forward == 2){
                shoot();
                shooting = true;
                if(steps == 2) {
                    shooting = false;
                    if(pattern == 2){
                        sort_times = 1;
                    }else if(pattern == 1){
                        sort_times = 2;
                    }
                    follower.followPath(third_pick);
                    forward = 4;
                    steps = 0;
                }
            }else if(forward == 4) {
                follower.followPath(fourth_shoot);
                forward = 5;
                steps = 0;
            }else if(forward == 5){
                shooting_time.reset();
                forward = 6;
            }else if(forward == 6 && shooting_time.milliseconds() > 500){
                intake_system.spindexer.setPower(-.7);
                intake_system.flippy_pos = flippy_up;
                intake_system.blocking = false;
                forward = 9;
            }
        }
    }
    public Path gate2;
    public Path gate3;

    public void red_init() {
        first_shoot = new Path(new BezierLine(new Pose(123, -24), new Pose(78, -4)));
        first_shoot.setConstantHeadingInterpolation(Math.toRadians(-90));
        second_pick = new Path(new BezierCurve(first_shoot.getLastControlPoint(), new Pose(70, -38)));
        second_pick.setConstantHeadingInterpolation(Math.toRadians(-90));
        gate = new Path(new BezierCurve(second_pick.getLastControlPoint(), new Pose(71, -33), new Pose(64, -33),new Pose(64, -42)));
        gate.setConstantHeadingInterpolation(Math.toRadians(0));
        third_shoot = new Path(new BezierLine(gate.getLastControlPoint(), new Pose(72, -8)));
        third_shoot.setLinearHeadingInterpolation(0,Math.toRadians(-90));
        first_pick = new Path(new BezierCurve(third_shoot.getLastControlPoint(), new Pose(49, -3), new Pose(47, -44)));
        first_pick.setConstantHeadingInterpolation(Math.toRadians(-90));
        second_shoot = new Path(new BezierCurve(first_pick.getLastControlPoint(), new Pose(47, -36), new Pose(72, -10)));
        second_shoot.setConstantHeadingInterpolation(Math.toRadians(-90));
        third_pick = new Path(new BezierCurve(second_shoot.getLastControlPoint(), new Pose(26, 0), new Pose(24, -44)));
        third_pick.setConstantHeadingInterpolation(Math.toRadians(-90));
        fourth_shoot = new Path(new BezierCurve(third_pick.getLastControlPoint(), new Pose(80, -10)));
        fourth_shoot.setConstantHeadingInterpolation(Math.toRadians(-90));
    }
    public void blue_init(){
        first_shoot = new Path(new BezierLine(new Pose(123, 24), new Pose(78, 4)));
        first_shoot.setConstantHeadingInterpolation(Math.toRadians(90));
        second_pick = new Path(new BezierCurve(first_shoot.getLastControlPoint(), new Pose(72, 38)));
        second_pick.setConstantHeadingInterpolation(Math.toRadians(90));
        gate = new Path(new BezierCurve(second_pick.getLastControlPoint(), new Pose(71, 33), new Pose(66, 33),new Pose(66, 43)));
        gate.setConstantHeadingInterpolation(Math.toRadians(0));
        third_shoot = new Path(new BezierLine(gate.getLastControlPoint(), new Pose(72, 8)));
        third_shoot.setLinearHeadingInterpolation(0,Math.toRadians(90));
        first_pick = new Path(new BezierCurve(third_shoot.getLastControlPoint(), new Pose(49, 3), new Pose(47, 48)));
        first_pick.setConstantHeadingInterpolation(Math.toRadians(90));
        second_shoot = new Path(new BezierCurve(first_pick.getLastControlPoint(), new Pose(47, 36), new Pose(72, 10)));
        second_shoot.setConstantHeadingInterpolation(Math.toRadians(90));
        third_pick = new Path(new BezierCurve(second_shoot.getLastControlPoint(), new Pose(26, 0), new Pose(24, 48)));
        third_pick.setConstantHeadingInterpolation(Math.toRadians(90));
        fourth_shoot = new Path(new BezierCurve(third_pick.getLastControlPoint(), new Pose(90, 0)));
        fourth_shoot.setConstantHeadingInterpolation(Math.toRadians(90));
    }
    public void shoot(){
        if(shooting) {
            if (steps == 0 && abs(shooter.shoot1.getVelocity() - shooter.set_speed) <= 20 && intake_system.sort_time == 0 && ! intake_system.sorting) {
                shooting_time.reset();
                steps = .5;
            } else if(steps == .5 & shooting_time.milliseconds() > 300){
                shooting_time.reset();
                intake_system.spindexer.setPower(-.7);
                intake_system.spindex_shooting = true;
                intake_system.flippy_pos = flippy_up;
                intake_system.blocking = false;
                steps = 1;
            }else if (steps == 1 && (shooting_time.milliseconds() > 2000 || shooter.balls_shot >= intake_system.balls.size() )) {
                intake_system.flippy_pos = flippy_down;
                intake_system.spindexer.setPower(1);
                intake_system.spindex_shooting = false;
                intake_system.blocking = true;
                steps = 2;
            }
        }
    }
    public boolean sorted = false;
    public double sort_steps = 0;
    public ElapsedTime sort_time = new ElapsedTime();
    public void sort(){
        if(!sorted && intake_system.blocking && sort_times > 0) {
            if (intake_system.balls.size() == 3) {
                if(sort_steps == 0){
                    sort_time.reset();
                    sort_steps = 1;
                }else if(sort_steps == 1 && sort_time.milliseconds() > 500){
                    sorted = true;
                    intake_system.recount();
                    intake_system.sort_time = sort_times;
                    sort_times = 0;
                    sort_steps = 0;
                }
            }

        }else if(!intake_system.blocking){
            sorted = false;
        }
    }



}
