package org.firstinspires.ftc.teamcode;

import android.view.InputDevice;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.api.Robot;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class MouseTest extends LinearOpMode {

    private InputDevice mouse;
    private String inputDescriptor;
    private Robot bot;

    private void listAvailableDevices(){
        int[] ids = InputDevice.getDeviceIds();
        List<InputDevice> devices = new ArrayList<>();

        for(int id : ids){
            InputDevice device = InputDevice.getDevice(id);
            devices.add(device);
        }
    }

    public void runOpMode(){
        int[] deviceIds = InputDevice.getDeviceIds();

        waitForStart();
    }
}
