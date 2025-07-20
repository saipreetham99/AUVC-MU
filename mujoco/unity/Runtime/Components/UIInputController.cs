using UnityEngine;
using TMPro; // Required for TextMeshPro Input Fields

// Place this script on the 'UI_InputWindow' GameObject in your scene.
public class UIInputController : MonoBehaviour
{
    // --- Public Properties to Access Values ---
    public float WaterHeight { get; private set; }
    public float T1_Min { get; private set; }
    public float T1_Max { get; private set; }
    public float T2_Min { get; private set; }
    public float T2_Max { get; private set; }

    // --- Private references to the UI components ---
    private TMP_InputField waterHeightInput;
    private TMP_InputField t1MinInput;
    private TMP_InputField t1MaxInput;
    private TMP_InputField t2MinInput;
    private TMP_InputField t2MaxInput;

    void Awake()
    {
        // Find all the child InputField components based on their names in the hierarchy.
        waterHeightInput = transform.Find("WaterHeight_InputField")?.GetComponent<TMP_InputField>();
        t1MinInput = transform.Find("T1_Min")?.GetComponent<TMP_InputField>();
        t1MaxInput = transform.Find("T1_Max")?.GetComponent<TMP_InputField>();
        t2MinInput = transform.Find("T2_Min")?.GetComponent<TMP_InputField>();
        t2MaxInput = transform.Find("T2_Max")?.GetComponent<TMP_InputField>();
        
        // Initialize the properties with the default values from the input fields.
        InitializeAllValues();
        
        // Add listeners. These methods will be called ONLY when editing is finished.
        AddListeners();
    }

    // This method hooks up our functions to the 'onEndEdit' event of each input field.
    private void AddListeners()
    {
        // The lambda expression now calls our parsing function and ASSIGNS the returned value
        // to the property. This is the correct way to do it.
        if (waterHeightInput != null) waterHeightInput.onEndEdit.AddListener(value => WaterHeight = ParseValue(value, WaterHeight, "Water Height"));
        if (t1MinInput != null) t1MinInput.onEndEdit.AddListener(value => T1_Min = ParseValue(value, T1_Min, "T1 Min"));
        if (t1MaxInput != null) t1MaxInput.onEndEdit.AddListener(value => T1_Max = ParseValue(value, T1_Max, "T1 Max"));
        if (t2MinInput != null) t2MinInput.onEndEdit.AddListener(value => T2_Min = ParseValue(value, T2_Min, "T2 Min"));
        if (t2MaxInput != null) t2MaxInput.onEndEdit.AddListener(value => T2_Max = ParseValue(value, T2_Max, "T2 Max"));
    }

    // *** CHANGED METHOD ***
    // This method now returns a float instead of taking a ref parameter.
    // If parsing fails, it returns the 'currentValue' that was passed in, so the value doesn't change.
    private float ParseValue(string inputValue, float currentValue, string valueName)
    {
        if (float.TryParse(inputValue, out float result))
        {
            Debug.Log($"{valueName} applied with new value: {result}");
            return result; // Return the new, successfully parsed value
        }
        else
        {
            Debug.LogWarning($"Invalid input for {valueName}. Could not parse '{inputValue}' as a float. Value remains {currentValue}.");
            return currentValue; // Return the old value if parsing fails
        }
    }
    
    // *** CHANGED METHOD ***
    // Helper function to set the initial values on start without generating debug logs.
    private void InitializeAllValues()
    {
        if (waterHeightInput != null && float.TryParse(waterHeightInput.text, out float wh)) WaterHeight = wh;
        if (t1MinInput != null && float.TryParse(t1MinInput.text, out float t1m)) T1_Min = t1m;
        if (t1MaxInput != null && float.TryParse(t1MaxInput.text, out float t1mx)) T1_Max = t1mx;
        if (t2MinInput != null && float.TryParse(t2MinInput.text, out float t2m)) T2_Min = t2m;
        if (t2MaxInput != null && float.TryParse(t2MaxInput.text, out float t2mx)) T2_Max = t2mx;
    }

    // It's good practice to remove listeners when the object is destroyed.
    void OnDestroy()
    {
        if (waterHeightInput != null) waterHeightInput.onEndEdit.RemoveAllListeners();
        if (t1MinInput != null) t1MinInput.onEndEdit.RemoveAllListeners();
        if (t1MaxInput != null) t1MaxInput.onEndEdit.RemoveAllListeners();
        if (t2MinInput != null) t2MinInput.onEndEdit.RemoveAllListeners();
        if (t2MaxInput != null) t2MaxInput.onEndEdit.RemoveAllListeners();
    }
}
