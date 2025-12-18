using System.Collections.Generic;
using UnityEngine;

public class LidarSensor2D : MonoBehaviour
{
    [Header("LiDAR Settings")]
    public int numRays = 360;           // liczba wiązek (co 1 stopień)
    public float maxRange = 10f;        // maksymalny zasięg w metrach
    public float minRange = 0.05f;      // minimalny zasięg
    public float scanFrequency = 10f;   // Hz
    public LayerMask layerMask;         // co LiDAR ma "widzieć"

    [Header("Blind Spot Settings")]
    public bool enableBlindSpot = true;
    public float blindSpotStartAngle = 55f;  // początek martwego pola
    public float blindSpotEndAngle = 140f;   // koniec martwego pola

    [Header("Debug")]
    public bool showDebugRays = true;
    public Color blindSpotColor = Color.yellow;

    private float _timeSinceLastScan = 0f;
    private float _scanPeriod;
    public float[] ranges;
    public float[] intensities;

    void Start()
    {
        _scanPeriod = 1f / scanFrequency;
        ranges = new float[numRays];
        intensities = new float[numRays];
    }

    void Update()
    {
        _timeSinceLastScan += Time.deltaTime;
        if (_timeSinceLastScan >= _scanPeriod)
        {
            PerformScan();
            _timeSinceLastScan = 0f;
        }
    }

    void PerformScan()
    {
        float angleIncrement = 360f / numRays;

        for (int i = 0; i < numRays; i++)
        {
            float angle = transform.eulerAngles.y + i * angleIncrement;
            Vector3 dir = Quaternion.Euler(0, angle, 0) * Vector3.forward;

            // Sprawdź czy kąt znajduje się w martwym polu
            bool isInBlindSpot = enableBlindSpot && IsAngleInBlindSpot(angle);

            if (isInBlindSpot)
            {
                // Martwe pole - ustaw wartości na NaN
                ranges[i] = float.NaN;
                intensities[i] = 0f;
                if (showDebugRays)
                    Debug.DrawRay(transform.position, dir * maxRange, blindSpotColor, _scanPeriod);
            }
            else if (Physics.Raycast(transform.position, dir, out RaycastHit hit, maxRange, layerMask))
            {
                // Normalny pomiar
                ranges[i] = hit.distance;
                intensities[i] = Mathf.Clamp01(1.0f - hit.distance / maxRange);
                if (showDebugRays)
                    Debug.DrawRay(transform.position, dir * hit.distance, Color.green, _scanPeriod);
            }
            else
            {
                // Brak kolizji
                ranges[i] = float.NaN;
                intensities[i] = 0f;
                if (showDebugRays)
                    Debug.DrawRay(transform.position, dir * maxRange, Color.red, _scanPeriod);
            }
        }
    }

    bool IsAngleInBlindSpot(float angle)
    {
        // Normalizuj kąt do zakresu 0-360
        angle = NormalizeAngle(angle);
        
        // Sprawdź czy kąt mieści się w martwym polu
        if (blindSpotStartAngle <= blindSpotEndAngle)
        {
            // Normalny zakres (np. 55° do 140°)
            return angle >= blindSpotStartAngle && angle <= blindSpotEndAngle;
        }
        else
        {
            // Zakres przechodzący przez 0° (np. 330° do 30°)
            return angle >= blindSpotStartAngle || angle <= blindSpotEndAngle;
        }
    }

    float NormalizeAngle(float angle)
    {
        // Normalizuj kąt do zakresu 0-360
        angle %= 360f;
        if (angle < 0f)
            angle += 360f;
        return angle;
    }

    // Metody do dynamicznej zmiany martwego pola
    public void SetBlindSpot(float startAngle, float endAngle)
    {
        blindSpotStartAngle = startAngle;
        blindSpotEndAngle = endAngle;
    }

    public void EnableBlindSpot(bool enable)
    {
        enableBlindSpot = enable;
    }

    // Debug - wyświetl informacje o martwym polu
    void OnDrawGizmosSelected()
    {
        if (!enabled) return;

        Gizmos.color = blindSpotColor;
        int segments = 36;
        float angleStep = (blindSpotEndAngle - blindSpotStartAngle) / segments;

        for (int i = 0; i < segments; i++)
        {
            float startAngle = blindSpotStartAngle + i * angleStep;
            float endAngle = startAngle + angleStep;

            Vector3 startDir = Quaternion.Euler(0, startAngle, 0) * Vector3.forward;
            Vector3 endDir = Quaternion.Euler(0, endAngle, 0) * Vector3.forward;

            Vector3 startPos = transform.position + startDir * minRange;
            Vector3 endPos = transform.position + endDir * minRange;

            Gizmos.DrawLine(startPos, endPos);

            // Rysuj zewnętrzny łuk
            Vector3 startPosOuter = transform.position + startDir * maxRange;
            Vector3 endPosOuter = transform.position + endDir * maxRange;
            Gizmos.DrawLine(startPosOuter, endPosOuter);

            // Rysuj promienie łączące
            Gizmos.DrawLine(startPos, startPosOuter);
        }
    }
}