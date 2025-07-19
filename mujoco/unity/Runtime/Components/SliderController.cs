using UnityEngine;
using UnityEngine.UI;

public class SliderController : MonoBehaviour
{
    public Slider speedSlider;

    public float GetSliderValue()
    {
      if (speedSlider == null)
      {
        Debug.LogWarning("Speed Slider is not assigned!");
        return 0f;
      }

      return speedSlider.value;
    }

}
