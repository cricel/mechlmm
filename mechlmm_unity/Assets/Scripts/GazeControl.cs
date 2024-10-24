using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;

public class GazeControl : MonoBehaviour
{
    public Camera mainCamera;  // Assign the main camera in the Inspector.
    public RawImage rawImage;
    public LayerMask layerMask;  // Specify layers to interact with
    public GraphicRaycaster raycaster;  // Assign the GraphicRaycaster for your Canvas
    public EventSystem eventSystem;  // Assign the EventSystem in the scene

    void Update()
    {
       // Step 1: Create a pointer event for the raycast
        PointerEventData pointerEventData = new PointerEventData(eventSystem);
        pointerEventData.position = new Vector2(Screen.width / 2, Screen.height / 2);  // Center of the screen

        // Step 2: Raycast using the GraphicRaycaster
        List<RaycastResult> results = new List<RaycastResult>();
        raycaster.Raycast(pointerEventData, results);

        Debug.Log("1");
        // Step 3: Check if the raycast hit the RawImage
        foreach (RaycastResult result in results)
        {
            RawImage hitRawImage = result.gameObject.GetComponent<RawImage>();
            if (hitRawImage == rawImage)
            {
                Debug.Log("2");
                // Step 4: Convert the screen point to local coordinates of the RawImage
                RectTransform rectTransform = rawImage.GetComponent<RectTransform>();
                Vector2 localPoint;
                RectTransformUtility.ScreenPointToLocalPointInRectangle(rectTransform, pointerEventData.position, mainCamera, out localPoint);

                // Step 5: Convert local coordinates to UV coordinates
                Rect rect = rectTransform.rect;
                float normalizedX = (localPoint.x - rect.x) / rect.width;
                float normalizedY = (localPoint.y - rect.y) / rect.height;

                // Ensure UV coordinates are within bounds (0 to 1)
                if (normalizedX >= 0 && normalizedX <= 1 && normalizedY >= 0 && normalizedY <= 1)
                {
                    Debug.Log("3");
                    // Step 6: Convert UV to pixel coordinates and get the pixel color
                    Texture2D texture = rawImage.texture as Texture2D;
                    if (texture != null)
                    {
                        Debug.Log("4");
                        int pixelX = Mathf.FloorToInt(normalizedX * texture.width);
                        int pixelY = Mathf.FloorToInt(normalizedY * texture.height);

                        Color pixelColor = texture.GetPixel(pixelX, pixelY);
                        Debug.Log($"Hit Pixel coordinates: ({pixelX}, {pixelY}), Pixel color: {pixelColor}");
                    }
                }
            }
        }

        Ray ray = mainCamera.ScreenPointToRay(pointerEventData.position);
        Debug.DrawRay(ray.origin, ray.direction * 9999f, Color.green, 1f); // 2 seconds visibility
    }
}
