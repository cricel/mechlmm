using UnityEngine;
using RosSharp.RosBridgeClient;

public class ROSTest : MonoBehaviour
{
    public RosSharp.RosBridgeClient.MessageTypes.Std.String message;
    public UnityPublisher<RosSharp.RosBridgeClient.MessageTypes.Std.String> testPub;

    void Start()
    {
        
        message = new RosSharp.RosBridgeClient.MessageTypes.Std.String();
        message.data = "hello";
    }

    // Update is called once per frame
    void Update()
    {
        // testPub.Publish(message);
    }
}
