package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Config
@Autonomous(name="Lame Auto", group="ABC Opmode", preselectTeleOp = "DecodeTeleop")
public class LameAuto extends DecodeLibrary{
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
    public static double first_dump_time = 1250;
    public static double second_dump_time = 2500;
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
        if(color == 0){
            y_mod = -2;
            x_mod = -5;
        }else{
            x_mod = -6;
        }

        turret.zero = true;
        cam.look();
        if(cam.tags.contains(21.0)){
            pattern = 1;
        }else if(cam.tags.contains(22.0)){
            pattern = 2;
        }else if(cam.tags.contains(23.0)){
            pattern = 3;
        }
        turret.turret_move();
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
        telemetry.addData("pattern", pattern);
        telemetry.addData("color", color);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }

    @Override
    public void loop() {
        for(LynxModule hub : hubs){
            hub.clearBulkCache();
        }
        if(forward == .25) {
            follower.update();
            turret.turret_move();
            shooter.set_speed = shoot_multiplier * ((dead_distance * .0254) - 1.6) + shoot_power_offset + (speed_increase * abs(turret.target_angle - .5));
            if (forward < 1) {
                shooter.set_speed = Math.round(shooter.set_speed / 20) * 20 + 60;
            } else {
                shooter.set_speed = Math.round(shooter.set_speed / 20) * 20;
            }
            shooter.shooting();
            intake_system.auto();
            auto_sort();
            intake_system.sort();
            auto_pose = follower.getPose();
            if (follower.atParametricEnd() || !follower.isBusy() || follower.isRobotStuck()) {
                if (forward == .25) {
                    intake_system.outtake = false;
                    shoot();
                    shooting = true;
                    if (steps == 2) {
                        shooting = false;
                        forward = .5;
                        steps = 0;
                    }
                }
            }
        }else{
            shooter.shoot1.setVelocity(0);
            shooter.shoot2.setVelocity(0);
            turret.turret_servo_1.setPosition(.5);
            turret.turret_servo_2.setPosition(.5);
            intake_system.intake.setPower(0);
            intake_system.spindexer.setPower(0);
        }
    }
    public Path gate2;
    public Path gate3;
    public Path to_gate;

    public void red_init(){
        first_shoot = new Path(new BezierLine(new Pose(122,-25), new Pose(124, -10)));
        first_shoot.setConstantHeadingInterpolation(Math.toRadians(-90));
    }
    public void blue_init(){
        first_shoot = new Path(new BezierLine(new Pose(122, -25), new Pose(124, 10)));
        first_shoot.setConstantHeadingInterpolation(Math.toRadians(90));
    }
    public void shoot(){
        if(shooting) {
            if (steps == 0 && abs(shooter.shoot1.getVelocity() - shooter.set_speed) <= 20 && intake_system.sort_time == 0 && !intake_system.sorting) {
                shooting_time.reset();
                steps = .5;
            } else if(steps == .5){
                shooting_time.reset();
                if(forward <= .75){
                    intake_system.spindexer.setPower(-.7);
                }else {
                    intake_system.spindexer.setPower(-.7);
                }
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
    public int sort_count(){
        intake_system.recount();
        List<String> unsort = new ArrayList<>();
        if(intake_system.balls.size() == 1){
            unsort.add(intake_system.balls.get(0));
        }else if(intake_system.balls.size() == 2){
            unsort.add(intake_system.balls.get(1));
            unsort.add(intake_system.balls.get(0));
        }else if(intake_system.balls.size() == 3){
            unsort.add(intake_system.balls.get(2));
            unsort.add(intake_system.balls.get(1));
            unsort.add(intake_system.balls.get(0));
        }
        List<String> sort = new ArrayList<>();
        int num = 0;
        if(pattern == 1){
            sort.add("Green");
            sort.add("Purple");
            sort.add("Purple");
        }else if(pattern == 2){
            sort.add("Purple");
            sort.add("Green");
            sort.add("Purple");
        }else if(pattern == 3){
            sort.add("Purple");
            sort.add("Purple");
            sort.add("Green");
        }
        if(unsort.size() == 3) {
            if (unsort == sort) {
                num = 0;
            } else if (!unsort.contains("Green")) {
                num = 0;
            } else if (!unsort.contains("Purple")) {
                num = 0;
            } else {
                if (Collections.frequency(unsort, "Green") > 1) {
                    num = sort.indexOf("Purple") - unsort.indexOf("Purple");
                    if (num < 0) {
                        num += 3;
                    }
                } else {
                    num = sort.indexOf("Green") - unsort.indexOf("Green");
                    if (num < 0) {
                        num += 3;
                    }
                }
            }
        }else{
            if(unsort.contains("Green")){
                if(sort.indexOf("Green") != unsort.indexOf("Green")){
                    num = 1;
                }else {
                    num = 0;
                }
            }else{
                num = 0;
            }
        }
        return num;
    }
    public boolean sorted = false;
    public double sort_steps = 0;
    public ElapsedTime sort_timer = new ElapsedTime();
    public boolean sort_balls = false;
    public boolean override = false;
    public void auto_sort() {
        if (!sorted && intake_system.blocking && sort_balls) {
            if (intake_system.balls.size() == 3 || override) {
                if (sort_steps == 0) {
                    sort_timer.reset();
                    sort_steps = 1;
                } else if (sort_steps == 1 && sort_timer.milliseconds() > 500) {
                    sorted = true;
                    override = false;
                    intake_system.sort_time = sort_count();
                    sort_balls = false;
                    sort_steps = 0;
                }
            }

        } else if (!intake_system.blocking) {
            sorted = false;
        }
    }



}
