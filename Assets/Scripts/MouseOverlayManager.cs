using UnityEngine;
using UnityEngine.EventSystems;

public class MouseOverlayManager : MonoBehaviour
{
    public RectTransform mouseSafeZone;

    private bool isInUI = false;
    private float mouseReleaseCooldown = 0f;

    void Update()
    {
        bool pointerOverUI = RectTransformUtility.RectangleContainsScreenPoint(mouseSafeZone, Input.mousePosition);

        // Update UI state
        if (pointerOverUI)
        {
            if (!isInUI)
            {
                // Just entered UI
                Cursor.lockState = CursorLockMode.None;
                Cursor.visible = true;
                isInUI = true;
            }

            // Block left click from triggering other scripts
            if (Input.GetMouseButtonDown(0))
            {
                mouseReleaseCooldown = 0.1f; // Short delay to prevent click-through
            }
        }
        else
        {
            if (mouseReleaseCooldown > 0)
            {
                mouseReleaseCooldown -= Time.unscaledDeltaTime;
            }
            else if (isInUI)
            {
                // Exited UI panel, resume freecam
                Cursor.lockState = CursorLockMode.Locked;
                Cursor.visible = false;
                isInUI = false;
            }
        }

        // While cooldown is active, cancel all input
        if (mouseReleaseCooldown > 0)
        {
            Input.ResetInputAxes();
        }
    }
}
