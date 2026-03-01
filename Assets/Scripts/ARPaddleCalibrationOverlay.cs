using UnityEngine;

public class ARPaddleCalibrationOverlay : MonoBehaviour
{
    [Header("Target")]
    [SerializeField] private ARTrackedPaddleMapper mapper;

    [Header("Visibility")]
    [SerializeField] private bool showInEditor = true;
    [SerializeField] private bool onlyShowWhenTracking = true;

    [Header("Step Sizes")]
    [SerializeField] private float positionStepMeters = 0.0025f;
    [SerializeField] private float rotationStepDegrees = 1.0f;

    [Header("Layout")]
    [SerializeField] private Vector2 panelSize = new Vector2(430f, 580f);
    [SerializeField] private int fontSize = 22;

    private Rect panelRect;
    private GUIStyle labelStyle;
    private GUIStyle buttonStyle;

    private void Awake()
    {
        ResolveMapper();
        panelRect = new Rect(20f, 20f, panelSize.x, panelSize.y);
    }

    private void OnEnable()
    {
        ResolveMapper();
    }

    private void ResolveMapper()
    {
        if (mapper != null)
        {
            return;
        }

        ARTrackedPaddleMapper[] mappers = FindObjectsByType<ARTrackedPaddleMapper>(FindObjectsSortMode.None);
        if (mappers.Length > 0)
        {
            mapper = mappers[0];
        }
    }

    private void EnsureStyles()
    {
        if (labelStyle == null)
        {
            labelStyle = new GUIStyle(GUI.skin.label)
            {
                fontSize = fontSize,
                wordWrap = true
            };
        }

        if (buttonStyle == null)
        {
            buttonStyle = new GUIStyle(GUI.skin.button)
            {
                fontSize = fontSize
            };
        }
    }

    private void OnGUI()
    {
        if (!showInEditor && Application.isEditor)
        {
            return;
        }

        ResolveMapper();
        EnsureStyles();

        panelRect = GUILayout.Window(GetInstanceID(), panelRect, DrawPanel, "Paddle Calibration");
    }

    private void DrawPanel(int id)
    {
        if (mapper == null)
        {
            GUILayout.Label("No ARTrackedPaddleMapper found.", labelStyle);
            if (GUILayout.Button("Retry", buttonStyle, GUILayout.Height(52f)))
            {
                ResolveMapper();
            }

            GUI.DragWindow();
            return;
        }

        if (onlyShowWhenTracking && !mapper.gameObject.activeInHierarchy)
        {
            GUILayout.Label("Tracker not active.", labelStyle);
            GUI.DragWindow();
            return;
        }

        GUILayout.Label($"Calibration: {(mapper.CalibrationEnabled ? "ON" : "OFF")}", labelStyle);
        GUILayout.Label($"Pos: {mapper.LocalPositionOffset}", labelStyle);
        GUILayout.Label($"Rot: {mapper.LocalEulerOffset}", labelStyle);

        GUILayout.Space(8f);

        if (GUILayout.Button(mapper.CalibrationEnabled ? "Disable Calibration" : "Enable Calibration", buttonStyle, GUILayout.Height(52f)))
        {
            mapper.ToggleCalibration();
        }

        GUILayout.Space(8f);
        GUILayout.Label("Position", labelStyle);
        DrawAxisButtons(
            "X",
            () => mapper.AdjustPosition(new Vector3(-positionStepMeters, 0f, 0f)),
            () => mapper.AdjustPosition(new Vector3(positionStepMeters, 0f, 0f)));
        DrawAxisButtons(
            "Y",
            () => mapper.AdjustPosition(new Vector3(0f, -positionStepMeters, 0f)),
            () => mapper.AdjustPosition(new Vector3(0f, positionStepMeters, 0f)));
        DrawAxisButtons(
            "Z",
            () => mapper.AdjustPosition(new Vector3(0f, 0f, -positionStepMeters)),
            () => mapper.AdjustPosition(new Vector3(0f, 0f, positionStepMeters)));

        GUILayout.Space(8f);
        GUILayout.Label("Rotation", labelStyle);
        DrawAxisButtons(
            "Pitch X",
            () => mapper.AdjustRotation(new Vector3(-rotationStepDegrees, 0f, 0f)),
            () => mapper.AdjustRotation(new Vector3(rotationStepDegrees, 0f, 0f)));
        DrawAxisButtons(
            "Yaw Y",
            () => mapper.AdjustRotation(new Vector3(0f, -rotationStepDegrees, 0f)),
            () => mapper.AdjustRotation(new Vector3(0f, rotationStepDegrees, 0f)));
        DrawAxisButtons(
            "Roll Z",
            () => mapper.AdjustRotation(new Vector3(0f, 0f, -rotationStepDegrees)),
            () => mapper.AdjustRotation(new Vector3(0f, 0f, rotationStepDegrees)));

        GUILayout.Space(8f);
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Save", buttonStyle, GUILayout.Height(52f)))
        {
            mapper.SaveCurrentOffsets();
        }

        if (GUILayout.Button("Reset", buttonStyle, GUILayout.Height(52f)))
        {
            mapper.ResetOffsets();
        }

        GUILayout.EndHorizontal();

        GUI.DragWindow();
    }

    private void DrawAxisButtons(string label, System.Action minusAction, System.Action plusAction)
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label(label, labelStyle, GUILayout.Width(130f));
        if (GUILayout.Button("-", buttonStyle, GUILayout.Height(52f)))
        {
            minusAction?.Invoke();
        }

        if (GUILayout.Button("+", buttonStyle, GUILayout.Height(52f)))
        {
            plusAction?.Invoke();
        }

        GUILayout.EndHorizontal();
    }
}