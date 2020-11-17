package org.firstinspires.ftc.teamcode;

import android.view.InputDevice;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.api.Robot;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class MouseTest extends LinearOpMode {

    private InputDevice mouse = null;
    // Get this by looking at descriptors for available devices with listAvailableDevices
    private String inputDescriptor = "c891512364778fa5d984526e6b86d606dfab294e";
    private Robot bot;

    private void listAvailableDevices(){
        int[] ids = InputDevice.getDeviceIds();
        List<InputDevice> devices = new ArrayList<>();

        for(int id : ids){
            InputDevice device = InputDevice.getDevice(id);
            telemetry.addData("Descriptor for " + device.getName() + ":", device.getDescriptor());
        }
        telemetry.update();
    }

    private void getMouse(){
        InputDevice mouse = null;
        int[] ids = InputDevice.getDeviceIds();

        for(int id : ids){
            InputDevice device = InputDevice.getDevice(id);
            if(device.getDescriptor().equals(inputDescriptor)){
                mouse = device;
                break;
            }
        }
    }

    public void runOpMode(){
        try {
            getMouse();

            if(mouse != null){
                waitForStart();

            }else{
                telemetry.addData("Initialization Failed!", "Mouse is not plugged in correctly");
                telemetry.update();
            }

        }catch(Exception e){

        }
    }
}
