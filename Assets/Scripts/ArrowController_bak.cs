using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public struct ArrowData
{
    public float shaftLength;
    public float shaftRadius;
    public float coneHeight;
    public float coneRadiusMultiplier;
}

[ExecuteAlways]
public class ArrowController : MonoBehaviour
{
    // Internal arrow visuals
    [SerializeField] Transform cylinder;
    [SerializeField] Transform cone;
    [SerializeField] Renderer cylinderRenderer;
    [SerializeField] Renderer coneRenderer;

    [SerializeField] Color positiveColor = Color.green;
    [SerializeField] Color negativeColor = Color.red;

    [SerializeField] ArrowData data;

    static Dictionary<GameObject, ArrowController> registry = new();

    public static ArrowController Get(GameObject obj)
    {
        registry.TryGetValue(obj, out var arrow);
        return arrow;
    }

    public void SetData(ArrowData newData)
    {
        data = newData;
        Apply();
    }

    public void SetLength(float shaftLength)
    {
        data.shaftLength = shaftLength;
        Apply();
    }

    public ArrowData GetData() => data;

    void OnEnable()
    {
        registry[gameObject] = this;
        Apply();
    }

    void OnDisable()
    {
        registry.Remove(gameObject);
    }

    void OnValidate()
    {
        if (!Application.isPlaying) return;
        Apply();
    }

    void Update()
    {
#if UNITY_EDITOR
        if (!Application.isPlaying)
            Apply();
#endif
    }

    void Apply()
    {
        if (!cylinder || !cone) return;

        float dir = Mathf.Sign(data.shaftLength);
        float length = Mathf.Max(Mathf.Abs(data.shaftLength), 0.001f);
        float radius = Mathf.Max(data.shaftRadius, 0.001f);
        float coneHeight = Mathf.Max(data.coneHeight, 0.001f);
        float coneRadius = Mathf.Max(radius * data.coneRadiusMultiplier, 0.001f);

        // Shaft: scale and position
        cylinder.localScale = new Vector3(radius, length / 2f, radius);
        cylinder.localPosition = new Vector3(0, dir * length / 2f, 0);
        cylinder.localRotation = Quaternion.identity;

        // Cone: scale, position, and orientation
        cone.localScale = new Vector3(coneRadius, coneHeight, coneRadius);
        cone.localPosition = new Vector3(0, dir * length, 0);
        cone.localRotation = (dir >= 0) ? Quaternion.identity : Quaternion.Euler(180f, 0, 0);

        // Color
        var color = data.shaftLength >= 0f ? positiveColor : negativeColor;
        if (cylinderRenderer?.sharedMaterial != null)
            cylinderRenderer.sharedMaterial.color = color;
        if (coneRenderer?.sharedMaterial != null)
            coneRenderer.sharedMaterial.color = color;
    }
}
