package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

public class Camera{
    private int width; // width resolution
    private int height; //height resolution
    private int exposure; // the exposure 
    private int framerateFront; // framerate of the front camera
    //private int framerateBack; // framerate of the back camera
    private UsbCamera frontCam; // the front camera
//    private UsbCamera backCam; // the back camera

    public Camera(int newWidth, int newHeight, int newExposure){
        width = newWidth;
        height = newHeight;
        exposure = newExposure; 
        framerateFront = 10;
        //framerateBack = 20;
    }
    public Camera(){ // width, height, exposure
        this(160, 120, 2);
    }

    public void setExposure(int value){
        frontCam.setExposureManual(value); // exposure value of the front camera
    //    backCam.setExposureManual(value); // exposure value of the back camera
    }
    public void init(){ //initializing
        frontCam = CameraServer.getInstance().startAutomaticCapture(0); // 
        frontCam.setResolution(width, height); // setting the resolution
        frontCam.setExposureManual(exposure); // manually setting the exposure
        frontCam.setFPS(framerateFront); // setting the framerate
        frontCam.setWhiteBalanceHoldCurrent(); // white balance of the camera

    //    backCam = CameraServer.getInstance().startAutomaticCapture(1);
    //    backCam.setResolution(width, height); // setting the width and height
    //    backCam.setExposureManual(exposure); // setting the exposure
    //    backCam.setFPS(framerateBack); // setting the framerate
    //    backCam.setWhiteBalanceHoldCurrent(); // white balance of the camera



    }
}
