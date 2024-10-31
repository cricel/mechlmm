
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;

public class ROSController : MonoBehaviour
{
    private ROSConnection ros;
    
    [SerializeField]
    private string cameraImgTopicName = "/camera/rgb/image_raw";
    [SerializeField]
    private string dummyPublisherStringTopicName = "dummy";
    [SerializeField]
    private float publishMessageFrequency = 0.5f;

    [Space]
    [SerializeField]
    private RawImage display;

    private float timeElapsed;

    private Texture2D texRos;
    

    void Start()
    {
        // start the ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(dummyPublisherStringTopicName);
        
        ros.Subscribe<ImageMsg>(cameraImgTopicName, RemoteRobotBaseCam);
    }

    private void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            StringMsg cubePos = new StringMsg("Hi");

            ros.Publish(dummyPublisherStringTopicName, cubePos);

            timeElapsed = 0;
        }
    }

    public void RemoteRobotBaseCam(ImageMsg img) {
        texRos = new Texture2D((int) img.width, (int) img.height, TextureFormat.RGB24, false); // , TextureFormat.RGB24
        BgrToRgb(img.data);
        texRos.LoadRawTextureData(img.data);
        // FlipTextureX(texRos); 

        texRos.Apply();

        RectTransform rectTransform = display.GetComponent<RectTransform>();
        rectTransform.sizeDelta = new Vector2(texRos.width, texRos.height);


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

    void FlipTextureX(Texture2D texture)
    {
        int width = texture.width;
        int height = texture.height;

        // Loop through each row and flip the pixels horizontally
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width / 2; x++)
            {
                // Get the pixel on the left
                Color leftPixel = texture.GetPixel(x, y);
                
                // Get the pixel on the right
                Color rightPixel = texture.GetPixel(width - x - 1, y);

                // Swap the left and right pixels
                texture.SetPixel(x, y, rightPixel);
                texture.SetPixel(width - x - 1, y, leftPixel);
            }
        }
    }
}