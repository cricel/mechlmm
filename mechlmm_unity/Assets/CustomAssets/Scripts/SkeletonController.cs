
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

// using Unity.Robotics.ROSTCPConnector;
// using RosMessageTypes.Std;

public class SkeletonController : MonoBehaviour
{
    // private ROSConnection ros;
    
    [SerializeField]
    private List<Bone> boneList = new List<Bone>();

    [Header("ROS")]
    public string skeletonTopicName = "skeleton_joints";
    [SerializeField]
    private float publishMessageFrequency = 0.5f;

    private float timeElapsed;

    void Start()
    {
        // ros = ROSConnection.GetOrCreateInstance();
        // ros.RegisterPublisher<Float32MultiArrayMsg>(skeletonTopicName);

        boneList = transform.GetComponentsInChildren<Bone>().ToList();
    }

    // Update is called once per frame
    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            // Float32MultiArrayMsg skeletonMsg = new Float32MultiArrayMsg();
            List<float> posList = new List<float>();
            foreach (Bone bone in boneList){
                posList.Add(bone.transform.localEulerAngles.x);
                posList.Add(bone.transform.localEulerAngles.y);
                posList.Add(bone.transform.localEulerAngles.z);
            }
            Debug.Log(boneList.Count);
            Debug.Log(posList.Count);
            Debug.Log(posList[2]);
            // skeletonMsg.data = posList.ToArray();
            // ros.Publish(skeletonTopicName, skeletonMsg);

            timeElapsed = 0;
        }
    }
}