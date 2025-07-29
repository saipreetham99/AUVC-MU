using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using UnityEngine;
using UnityEngine.Profiling;
using UnityEngine.UI;

/// <summary>
/// A thread-safe singleton that calculates the screen-space bounding box of a target GameObject
/// on the main thread and provides a method for other threads to safely access the latest data.
/// Attach this to a single GameObject in your scene.
/// </summary>
public class ThreadSafeBoundingBoxProvider : MonoBehaviour{
  // --- Singleton Setup ---
  private static ThreadSafeBoundingBoxProvider _instance;
  public static ThreadSafeBoundingBoxProvider Instance{
    get{
      if (_instance == null){
        // The instance is set in Awake(), so if it's null, it means it's not in the scene.
        throw new System.Exception("ThreadSafeBoundingBoxProvider instance not found in the scene. Make sure it's attached to a GameObject.");
      }
      return _instance;
    }
  }

  // --- Public Fields ---
  [Tooltip("The target GameObject whose bounding box will be calculated.")]
  public GameObject target; // Assign your target submarine GameObject to this field in the Unity Inspector

  // --- Thread-Safe Data Storage ---
  private readonly object _bboxDataLock = new object();
  private float[] _latestBboxData = new float[4] { 0f, 0f, 0f, 0f };

  // --- Private Calculation Fields ---
  private Camera _mainCamera;
  private Renderer[] _renderers;

  #region Unity Lifecycle Methods

  void Awake()
  {
    if (_instance == null)
    {
      _instance = this;
      DontDestroyOnLoad(this.gameObject);
    }
    else if (_instance != this)
    {
      // Enforce singleton pattern
      Destroy(gameObject);
    }
  }

  void Start()
  {
    _mainCamera = Camera.main;
    if (_mainCamera == null)
    {
      Debug.LogError("ThreadSafeBoundingBoxProvider: Main Camera not found!");
    }

    // Initial setup of renderers
    if (target != null)
    {
      SetupRenderersForTarget();
    }
  }

  void OnDestroy()
  {
    if (_instance == this)
    {
      _instance = null;
    }
  }

  void Update()
  {
    // This entire method runs on the main thread.
    if (target == null || _mainCamera == null)
    {
      // If no target, ensure the stored data is zeroed out.
      lock (_bboxDataLock)
      {
        System.Array.Clear(_latestBboxData, 0, 4);
      }
      return;
    }

    // If the renderers haven't been fetched yet (e.g., target was assigned after Start)
    if (_renderers == null)
    {
      SetupRenderersForTarget();
    }

    // Calculate the new bounding box. This is safe because we are on the main thread.
    Rect currentBbox = CalculateScreenSpaceBoundingBox();

    // Lock and update the shared data.
    lock (_bboxDataLock)
    {
      _latestBboxData[0] = currentBbox.x;
      _latestBboxData[1] = currentBbox.y;
      _latestBboxData[2] = currentBbox.width;
      _latestBboxData[3] = currentBbox.height;
    }
  }

  #endregion

  #region Public Thread-Safe Accessor

  /// <summary>
  /// Gets the latest calculated bounding box data in a thread-safe manner.
  /// Format: [x, y, width, height]
  /// </summary>
  /// <returns>A float array containing the bounding box data.</returns>
  public float[] GetLatestBoundingBoxData()
  {
    float[] bboxData = new float[4];
    lock (_bboxDataLock)
    {
      // Copy the data to a new array to avoid returning a reference to the internal array.
      System.Buffer.BlockCopy(_latestBboxData, 0, bboxData, 0, 4 * sizeof(float));
    }
    return bboxData;
  }

  #endregion

  #region Private Main-Thread-Only Methods

  /// <summary>
  /// Finds and stores the renderers for the current target.
  /// MUST be called from the main thread.
  /// </summary>
  private void SetupRenderersForTarget()
  {
    _renderers = target.GetComponentsInChildren<Renderer>();
    if (_renderers.Length == 0)
    {
      Debug.LogWarning($"No Renderers found on target GameObject '{target.name}' or its children. Bounding box will be invalid.");
    }
  }

  /// <summary>
  /// Calculates the 2D bounding box of the object as seen by the main camera.
  /// MUST be called from the main thread.
  /// </summary>
  /// <returns>A Rect representing the screen-space bounding box.</returns>
  private Rect CalculateScreenSpaceBoundingBox()
  {
    if (_renderers == null || _renderers.Length == 0)
    {
      return Rect.zero;
    }

    var screenPoints = new List<Vector3>();

    foreach (var rend in _renderers)
    {
      if (rend == null || !rend.isVisible) continue;

      Bounds bounds = rend.bounds;
      Vector3[] corners = new Vector3[8];
      // Calculate screen points for all 8 corners of the renderer's bounds
      corners[0] = _mainCamera.WorldToScreenPoint(new Vector3(bounds.min.x, bounds.min.y, bounds.min.z));
      corners[1] = _mainCamera.WorldToScreenPoint(new Vector3(bounds.min.x, bounds.min.y, bounds.max.z));
      corners[2] = _mainCamera.WorldToScreenPoint(new Vector3(bounds.min.x, bounds.max.y, bounds.min.z));
      corners[3] = _mainCamera.WorldToScreenPoint(new Vector3(bounds.min.x, bounds.max.y, bounds.max.z));
      corners[4] = _mainCamera.WorldToScreenPoint(new Vector3(bounds.max.x, bounds.min.y, bounds.min.z));
      corners[5] = _mainCamera.WorldToScreenPoint(new Vector3(bounds.max.x, bounds.min.y, bounds.max.z));
      corners[6] = _mainCamera.WorldToScreenPoint(new Vector3(bounds.max.x, bounds.max.y, bounds.min.z));
      corners[7] = _mainCamera.WorldToScreenPoint(bounds.max);

      foreach (var corner in corners)
      {
        if (corner.z > 0) // Only consider points in front of the camera
        {
          screenPoints.Add(corner);
        }
      }
    }

    if (screenPoints.Count == 0)
    {
      return Rect.zero;
    }

    float minX = screenPoints.Min(p => p.x);
    float maxX = screenPoints.Max(p => p.x);
    float minY = screenPoints.Min(p => p.y);
    float maxY = screenPoints.Max(p => p.y);

    return new Rect(minX, minY, maxX - minX, maxY - minY);
  }

  #endregion
}
