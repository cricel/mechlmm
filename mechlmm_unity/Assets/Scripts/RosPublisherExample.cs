using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

using System.Collections;
using System.Collections.Generic;
using UnityEngine.UI;
using RosMessageTypes.Sensor;
using RosMessageTypes.BuiltinInterfaces;
using System;


/// <summary>
///
/// </summary>
public class RosPublisherExample : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "pos_rot";

    // Publish the cube's position and rotation every N seconds
    public float publishMessageFrequency = 0.5f;

    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;


    int currentCamIdx = 0;
    // WebCamTexture tex;
    Texture2D texRos;
    public RawImage display;
    CompressedImageMsg img_msg;
    string webcamiagetopic = "/camera/rgb/image_raw";

    void Start()
    {
        // start the ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(topicName);
        
        ros.Subscribe<ImageMsg>(webcamiagetopic, StartStopCam_Clicked);
    }

    private void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            StringMsg cubePos = new StringMsg("Hi");

            // Finally send the message to server_endpoint.py running in ROS
            ros.Publish(topicName, cubePos);

            timeElapsed = 0;
        }
    }

   
    public void StartStopCam_Clicked(ImageMsg img) {
        // stopping the prev output and clearing the texture
        // if (texRos != null) {
        //     display.texture = null;
        //     // texRos.Stop();
        //     texRos = null;
        // } else {
        // RenderTexture rendtextRos = new RenderTexture(640, 480, 0, UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8A8_UNorm);
        // rendtextRos.Create();
        // rendtextRos.
        texRos = new Texture2D((int) img.width, (int) img.height, TextureFormat.RGB24, false); // , TextureFormat.RGB24
        // BgrToRgb(img.data);
        texRos.LoadRawTextureData(img.data);

        texRos.Apply();
        display.texture = texRos;       
    }

    public void BgrToRgb(byte[] data) {
        for (int i = 0; i < data.Length; i += 3)
        {
            byte dummy = data[i];
            data[i] = data[i + 2];
            data[i + 2] = dummy;
        }
    }
}