using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using TMPro;

public class RecordingController : MonoBehaviour
{
    [Header("ROS")]
    private ROSConnection ros;

    [SerializeField]
    private string recordingTopicName;
    public bool isRecording = false;
    [Space]
    [SerializeField]
    private string trainingInfoTopicName;
    [SerializeField]
    private string actionName;
    [SerializeField]
    private int sequenceNum = 0;

    [Header("UI")]
    [SerializeField]
    private Button recordingIcon;
    [SerializeField]
    private TMP_Text titleText;
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<BoolMsg>(recordingTopicName);
        ros.RegisterPublisher<StringMsg>(trainingInfoTopicName);
        
        ros.Subscribe<BoolMsg>(recordingTopicName, RecordingTriggerCallback);
        ros.Subscribe<StringMsg>(trainingInfoTopicName, TrainingInfoCallback);
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void RecordingTriggerCallback(BoolMsg _msg){
        isRecording = _msg.data;
        if(isRecording){
            recordingIcon.image.color = Color.green;
        }
        else{
            recordingIcon.image.color = Color.red;
        }
    }

    public void TrainingInfoCallback(StringMsg _msg){
        titleText.text = _msg.data;
    }

    public void PublishRecordingTrigger(){
        BoolMsg trigger = new BoolMsg(!isRecording);
        ros.Publish(recordingTopicName, trigger);

        StringMsg actionInfo = new StringMsg(actionName + "," + sequenceNum.ToString());
        ros.Publish(trainingInfoTopicName, actionInfo);

        sequenceNum += 1;
    }
}
