using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using TMPro;
using System.Collections;

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
    [SerializeField]
    private TMP_Text countDownText;
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

    public void RecordingReady(){
        StartCoroutine(CountdownCoroutine());
    }

    private IEnumerator CountdownCoroutine()
    {
        countDownText.gameObject.SetActive(true);

        for (int i = 3; i >= 1; i--)
        {
            countDownText.text = i.ToString();
            yield return new WaitForSeconds(1f);
        }

        BoolMsg trigger = new BoolMsg(!isRecording);
        ros.Publish(recordingTopicName, trigger);

        StringMsg actionInfo = new StringMsg(actionName + "," + sequenceNum.ToString());
        ros.Publish(trainingInfoTopicName, actionInfo);

        sequenceNum += 1;

        countDownText.gameObject.SetActive(false);
    }
}
