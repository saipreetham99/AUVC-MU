using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

public class MouseOverlayManager : MonoBehaviour
{
    public RectTransform mouseSafeZone; // Assign the Panel here

    void Update()
    {
        if (IsPointerOverUI(mouseSafeZone))
        {
            Cursor.lockState = CursorLockMode.None;
            Cursor.visible = true;
        }
        else
        {
            Cursor.lockState = CursorLockMode.Locked;
            Cursor.visible = false;
        }
    }

    bool IsPointerOverUI(RectTransform uiElement)
    {
        Vector2 localMousePos;
        RectTransformUtility.ScreenPointToLocalPointInRectangle(
            uiElement,
            Input.mousePosition,
            null,
            out localMousePos
        );

        return uiElement.rect.Contains(localMousePos);
    }
}

