using UnityEngine;

public class CubeButtonController : MonoBehaviour
{
    [SerializeField]
    private RecordingController recordingController;
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void OnTriggerEnter(Collider other)
    {
        Debug.Log("--");

        Debug.Log(other.name);
        recordingController.RecordingReady();

        if(other.name == "Trigger Point")
        {
            
        }
    }
}
