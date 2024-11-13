using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;

// using Unity.Robotics.ROSTCPConnector;
// using RosMessageTypes.Std;
// using RosMessageTypes.Sensor;

public class GazeControl : MonoBehaviour
{
    [Header("Gaze")]
    [SerializeField]
    private Camera mainCamera;  // Assign the main camera in the Inspector.
    [SerializeField]
    private GraphicRaycaster raycaster;  // Assign the GraphicRaycaster for your Canvas
    [SerializeField]
    private EventSystem eventSystem;  // Assign the EventSystem in the scene
    [SerializeField]
    private GameObject interactionMarkerPrefab;

    private GameObject currentMarker;

    [Header("Debug")]
    public Vector2 gazePos = Vector2.zero;


    // [Header("ROS")]
    // private ROSConnection ros;

    [SerializeField]
    private string gazeTopicName = "operator_gaze";

    void Start()
    {
        // ros = ROSConnection.GetOrCreateInstance();
        // ros.RegisterPublisher<Int32MultiArrayMsg>(gazeTopicName);
    }

    void Update()
    {
        // Int32MultiArrayMsg gazeMsg = new Int32MultiArrayMsg();
        // gazeMsg.data = new int[] { -1, -1 };

       // Step 1: Create a pointer event for the raycast
        PointerEventData pointerEventData = new PointerEventData(eventSystem);
        pointerEventData.position = new Vector2(Screen.width / 2, Screen.height / 2);  // Center of the screen

        // Step 2: Raycast using the GraphicRaycaster
        List<RaycastResult> results = new List<RaycastResult>();
        raycaster.Raycast(pointerEventData, results);

        // Step 3: Check if the raycast hit the RawImage
        foreach (RaycastResult result in results)
        {
            RawImage detectedRawImage = result.gameObject.GetComponent<RawImage>();
            if (detectedRawImage != null)
            {
                // Step 4: Convert the screen point to local coordinates of the RawImage
                RectTransform rectTransform = detectedRawImage.GetComponent<RectTransform>();
                Vector2 localPoint;
                RectTransformUtility.ScreenPointToLocalPointInRectangle(rectTransform, pointerEventData.position, mainCamera, out localPoint);

                // Step 5: Convert local coordinates to UV coordinates
                Rect rect = rectTransform.rect;
                float normalizedX = (localPoint.x - rect.x) / rect.width;
                float normalizedY = (localPoint.y - rect.y) / rect.height;

                // Ensure UV coordinates are within bounds (0 to 1)
                if (normalizedX >= 0 && normalizedX <= 1 && normalizedY >= 0 && normalizedY <= 1)
                {
                    // Step 6: Convert UV to pixel coordinates and get the pixel color
                    Texture2D texture = detectedRawImage.texture as Texture2D;
                    if (texture != null)
                    {
                        int pixelX = Mathf.FloorToInt(normalizedX * texture.width);
                        int pixelY = Mathf.FloorToInt(normalizedY * texture.height);

                        Color pixelColor = texture.GetPixel(pixelX, pixelY);
                        // Debug.Log($"Hit Pixel coordinates: ({pixelX}, {pixelY}), Pixel color: {pixelColor}");
                        gazePos = new Vector2(pixelX, pixelY);

                        // gazeMsg.data = new int[] { pixelX, pixelY };
                    }
                }

                Vector3 worldPosition = rectTransform.TransformPoint(localPoint);

                // If there's already a marker, just move it. Otherwise, instantiate a new one.
                if (currentMarker == null)
                {
                    currentMarker = Instantiate(interactionMarkerPrefab, worldPosition, Quaternion.identity);
                }
                else
                {
                    currentMarker.transform.position = worldPosition;
                }

            }
        }

        Ray ray = mainCamera.ScreenPointToRay(pointerEventData.position);
        Debug.DrawRay(ray.origin, ray.direction * 9999f, Color.green, 1f); // 2 seconds visibility

        // ros.Publish(gazeTopicName, gazeMsg);
    }
}
