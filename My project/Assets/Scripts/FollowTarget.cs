using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class FollowCamera : MonoBehaviour
{
    [Header("Follow settings")]
    [SerializeField] private float followRadius = 2.0f;
    [SerializeField] private float followHeight = 0.5f;
    [SerializeField] private float maxFollowSpeed = 20.0f;

    [Header("Mouse / manual")]
    [SerializeField] private float mouseSensitivity = 0.1f;
    [SerializeField] private float manualTimeout = 1.0f; // sekund po których manual wyłącza się
    [SerializeField] private float rotationSmooth = 0.2f;

    [Header("Targets")]
    [SerializeField] private List<GameObject> targets = new List<GameObject>();
    [SerializeField] private bool autoFindTargets = false;
    [SerializeField] private string targetTag = "Player";

    private const float TERRAIN_CLEARANCE = 0.5f;

    private bool hasFocus = false;
    private bool manual = false;
    private float manualRadius = 0f;

    private Vector2 mouseInput = Vector2.zero;
    private Vector2 mouseInputSmooth = Vector2.zero;
    private float lastTimeMouseMoved = -9999f;

    void Start()
    {
        if (autoFindTargets) FindTargetsByTag();
        if (targets.Count == 0)
        {
            Debug.LogWarning("FollowCamera: brak targetów. Ustaw 'targets' albo włącz autoFindTargets z poprawnym tagiem.");
        }

        // Jeżeli kamera jest dzieckiem jednego z targetów to to powoduje problemy.
        foreach (var t in targets)
        {
            if (t != null && transform.IsChildOf(t.transform))
            {
                Debug.LogWarning("FollowCamera: kamera jest childem targetu. To spowoduje zapętlone transformy. Usuń kamerę z dzieci targetu.");
            }
        }

        // Domyślne ustawienia kursora (można zmienić)
        Cursor.lockState = CursorLockMode.Locked;
        Cursor.visible = false;
    }

    void LateUpdate()
    {
        Vector3 meanPos = MeanTargetPos();

        // Oblicz pozycję docelową kamery
        Vector3 targetCameraPos;
        if (manual)
        {
            targetCameraPos = meanPos + transform.rotation * Vector3.back * manualRadius;
        }
        else
        {
            Vector3 targetToCamera2d = transform.position - meanPos;
            targetToCamera2d.y = 0f;

            targetCameraPos = meanPos;
            targetCameraPos.y += followHeight;
            if (targetToCamera2d.sqrMagnitude < 0.0001f)
            {
                // jeśli kamera na środku, ustaw domyślny offset
                targetCameraPos += Vector3.back * followRadius;
            }
            else
            {
                targetCameraPos += targetToCamera2d.normalized * followRadius;
            }
        }

        // Ruch kamery (gładki gdy nie manual)
        Vector3 toTarget = targetCameraPos - transform.position;
        if (!manual && toTarget.magnitude > 0.001f)
        {
            float speed = (1 - Mathf.Exp(-toTarget.magnitude / 5f)) * maxFollowSpeed;
            transform.position += toTarget.normalized * speed * Time.deltaTime;
        }
        else
        {
            transform.position = Vector3.Lerp(transform.position, targetCameraPos, manual ? 1f : 10f * Time.deltaTime);
        }

        // Wyłącz manual, gdy nie było ruchu myszy
        if (Time.time - lastTimeMouseMoved > manualTimeout)
        {
            manual = false;
        }

        if (hasFocus)
        {
            // Obrót oparty na wejściu myszy
            mouseInputSmooth = Vector2.Lerp(mouseInputSmooth, mouseInput, rotationSmooth);
            mouseInput = Vector2.zero;

            Vector3 eulerAngles = transform.eulerAngles;
            float pitch = eulerAngles.x - mouseInputSmooth.y * mouseSensitivity;
            pitch = pitch > 180 ? pitch - 360 : pitch;
            pitch = Mathf.Clamp(pitch, -89f, 89f);
            float yaw = eulerAngles.y + mouseInputSmooth.x * mouseSensitivity;

            Vector3 desiredEuler = new Vector3(pitch, yaw, 0f);
            Vector3 eulerDelta = desiredEuler - transform.eulerAngles;

            // Obrót wokół środka grupy targetów
            transform.RotateAround(meanPos, Vector3.up, eulerDelta.y);
            transform.RotateAround(meanPos, transform.right, eulerDelta.x);
        }

        // Sprawdź wysokość terenu pod kamerą i podnieś kamerę jeśli jest pod terenem
        float height = -Mathf.Infinity;
        RaycastHit hit;
        if (Physics.Raycast(transform.position + Vector3.up * 1000f, Vector3.down, out hit, 2000f))
        {
            // ustaw minimalną wysokość na podstawie trafionego collidera (terrain lub innego)
            height = Mathf.Max(height, hit.point.y);
        }

        float desiredHeight = (height == -Mathf.Infinity) ? transform.position.y : (height + TERRAIN_CLEARANCE);
        if (desiredHeight > transform.position.y)
        {
            transform.position = new Vector3(transform.position.x, desiredHeight, transform.position.z);
        }

        // Zawsze patrz na środek grupy
        transform.LookAt(meanPos);
    }

    // Input System callbacks - powiąż w ActionMap
    public void OnLook(InputValue value)
    {
        Vector2 mouseDelta = value.Get<Vector2>();
        if (mouseDelta.sqrMagnitude <= 0f) return;

        mouseInput += mouseDelta;
        lastTimeMouseMoved = Time.time;
        if (!manual)
        {
            manualRadius = (transform.position - MeanTargetPos()).magnitude;
        }
        manual = true;
    }

    public void OnClick()
    {
        hasFocus = true;
    }

    public void OnCancel()
    {
        hasFocus = false;
    }

    // Utility
    private Vector3 MeanTargetPos()
    {
        if (targets == null || targets.Count == 0) return Vector3.zero;
        Vector3 mean = Vector3.zero;
        int count = 0;
        foreach (var t in targets)
        {
            if (t == null) continue;
            mean += t.transform.position;
            count++;
        }
        return count == 0 ? Vector3.zero : mean / count;
    }

    public void FindTargetsByTag()
    {
        var found = GameObject.FindGameObjectsWithTag(targetTag);
        targets = new List<GameObject>(found);
        if (targets.Count == 0) Debug.LogWarning($"FollowCamera: nie znaleziono obiektów z tagiem '{targetTag}'.");
    }

    // Publicznie dostępne do debugowania / runtime
    public void SetTargets(List<GameObject> newTargets)
    {
        targets = newTargets ?? new List<GameObject>();
    }
}
