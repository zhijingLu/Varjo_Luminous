using UnityEngine;

public class BoundingBoxVisualizer : MonoBehaviour
{
    [Tooltip("All objects to visualize bounding boxes for (e.g. tracked/FBX objects)")]
    public GameObject[] trackedObjects;

    [Tooltip("World-space line width")]
    public float lineWidth = 0.004f;

    [Tooltip("Box color")]
    public Color lineColor = Color.green;

    // Each tracked object gets a set of 12 LineRenderers (for 12 box edges)
    private LineRenderer[][] boxLines;

    void Start()
    {
        if (trackedObjects == null) return;
        boxLines = new LineRenderer[trackedObjects.Length][];
        for (int i = 0; i < trackedObjects.Length; i++)
        {
            var obj = trackedObjects[i];
            if (obj == null) continue;
            boxLines[i] = CreateBoxLines(obj.name + "_BoundingBox");
            SetBoxLinesActive(boxLines[i], false); // start disabled
        }
    }

    void Update()
    {
        if (boxLines == null) return;
        for (int i = 0; i < trackedObjects.Length; i++)
        {
            var obj = trackedObjects[i];
            var lines = boxLines[i];
            if (obj != null && IsTracked(obj))
            {
                var rend = obj.GetComponent<Renderer>();
                if (rend != null)
                {
                    UpdateBoxLines(lines, rend.bounds);
                    SetBoxLinesActive(lines, true);
                }
                else
                {
                    SetBoxLinesActive(lines, false);
                }
            }
            else
            {
                SetBoxLinesActive(lines, false);
            }
        }
    }

    void OnDisable()
    {
        // Clean up all created lines
        if (boxLines != null)
        {
            foreach (var arr in boxLines)
            {
                if (arr == null) continue;
                foreach (var lr in arr)
                {
                    if (lr != null)
                        Destroy(lr.gameObject);
                }
            }
        }
        boxLines = null;
    }

    // Utility: create 12 lines for one box, parented under this GameObject
    private LineRenderer[] CreateBoxLines(string prefix)
    {
        var lines = new LineRenderer[12];
        for (int i = 0; i < 12; i++)
        {
            var go = new GameObject(prefix + "_edge" + i);
            go.transform.parent = this.transform;
            var lr = go.AddComponent<LineRenderer>();
            lr.positionCount = 2;
            lr.material = new Material(Shader.Find("Sprites/Default"));
            lr.widthMultiplier = lineWidth;
            lr.startColor = lr.endColor = lineColor;
            lr.useWorldSpace = true;
            lr.loop = false;
            lr.enabled = false;
            lines[i] = lr;
        }
        return lines;
    }

    // Utility: update all lines to match the bounds
    private void UpdateBoxLines(LineRenderer[] lines, Bounds b)
    {
        Vector3 min = b.min;
        Vector3 max = b.max;
        Vector3[] c = new Vector3[8]
        {
            new Vector3(min.x, min.y, min.z),
            new Vector3(max.x, min.y, min.z),
            new Vector3(max.x, max.y, min.z),
            new Vector3(min.x, max.y, min.z),
            new Vector3(min.x, min.y, max.z),
            new Vector3(max.x, min.y, max.z),
            new Vector3(max.x, max.y, max.z),
            new Vector3(min.x, max.y, max.z)
        };

        // The 12 edges: (start,end) pairs
        int[,] edges = new int[12,2]
        {
            {0,1},{1,2},{2,3},{3,0}, // bottom face
            {4,5},{5,6},{6,7},{7,4}, // top face
            {0,4},{1,5},{2,6},{3,7}  // verticals
        };

        for (int i = 0; i < 12; i++)
        {
            lines[i].SetPosition(0, c[edges[i,0]]);
            lines[i].SetPosition(1, c[edges[i,1]]);
        }
    }

    // Utility: show/hide all lines for one box
    private void SetBoxLinesActive(LineRenderer[] lines, bool active)
    {
        foreach (var lr in lines)
            if (lr != null) lr.enabled = active;
    }

    // Your own tracking logic. For now, let's assume it's "object is active in hierarchy"
    private bool IsTracked(GameObject obj)
    {
        // Replace with your own "is tracked" logic as needed
        return obj.activeInHierarchy;
    }
}
