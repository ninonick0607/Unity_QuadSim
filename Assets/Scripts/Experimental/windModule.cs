using UnityEngine;

namespace Experimental
{
    /// <summary>
    /// A wind/fan volume. Applies velocity-relative drag toward the zone's air
    /// velocity to any non-kinematic Rigidbody inside it. The wind originates from
    /// the back face (-Z) and blows forward (+Z). The force spreads outward and 
    /// develops localized turbulence as it approaches the front face.
    /// </summary>
    [RequireComponent(typeof(BoxCollider))]
    public class WindModule : MonoBehaviour
    {
        [Header("Base Wind Settings")]
        [Tooltip("Fan jet air speed at the source face, m/s. Blow direction is local forward (blue Z axis).")]
        [SerializeField] private float _windSpeed = 10f;

        [Tooltip("Linear drag, N per (m/s) of relative air speed.")]
        [SerializeField] private float _dragCoefficient = 0.5f;

        [Header("Spreading & Cone Settings")]
        [Tooltip("How much the wind spreads outward. 0 = perfectly straight beam; 1 = drastic drop-off at edges over distance.")]
        [Range(0f, 2f)] [SerializeField] private float _spreadFactor = 0f;

        [Header("Turbulence Settings")]
        [Tooltip("Maximum strength of turbulent sideways buffeting force.")]
        [SerializeField] private float _turbulenceStrength = 2f;

        [Tooltip("Speed/frequency of the turbulence fluctuations.")]
        [SerializeField] private float _turbulenceFrequency = 5f;

        [SerializeField] private bool _toggle = false;

        private BoxCollider _boxCollider;

        private void Awake()
        {
            _boxCollider = GetComponent<BoxCollider>();
        }

        private void Reset()   
        {
            var col = GetComponent<BoxCollider>();
            if (col != null) col.isTrigger = true;
        }

        private void OnTriggerStay(Collider other)
        {
            var rb = other.attachedRigidbody;
            if (rb == null || rb.isKinematic) return;

            // 1. Dynamic Length along local Z axis
            float maxRange = _boxCollider.size.z;

            // 2. Locate the Source Face (local back of the box)
            Vector3 sourceFacePosition = transform.position - (transform.forward * (maxRange * 0.5f));

            // 3. Distance Calculation
            Vector3 toTarget = rb.position - sourceFacePosition;
            float distanceAlongFlow = Vector3.Dot(toTarget, transform.forward);
            distanceAlongFlow = Mathf.Clamp(distanceAlongFlow, 0f, maxRange);

            // Normalized distance (0 at source face, 1 at front face)
            float travelRatio = distanceAlongFlow / maxRange;

            // 4. Base Distance Falloff (Max force at face, Min force at edge)
            float distanceFalloff = 1f - travelRatio;

            // 5. Spread (Cone Expansion) Calculation
            // Calculate lateral (side/up) displacement from the central wind vector line
            Vector3 centralProjectedPoint = sourceFacePosition + (transform.forward * distanceAlongFlow);
            float lateralDistance = Vector3.Distance(rb.position, centralProjectedPoint);
            
            // As travelRatio increases, the allowed "core" narrow beam spreads out, dropping edge intensity
            float spreadEdgeLoss = Mathf.Clamp01(1f - (lateralDistance * _spreadFactor * travelRatio));
            
            // Combine forward falloff with lateral spreading reduction
            float finalWindScale = distanceFalloff * spreadEdgeLoss;
            float currentWindSpeed = _windSpeed * finalWindScale;

            // 6. Main Wind Velocity Vector
            Vector3 airVelocity = transform.forward * currentWindSpeed;

            // 7. Turbulence Calculation (Scales UP as travelRatio increases)
            if (_turbulenceStrength > 0f && travelRatio > 0.05f)
            {
                // Generate a continuous pseudo-random 3D direction using Perlin Noise over time and position
                float timeOffset = Time.time * _turbulenceFrequency;
                
                float noiseX = Mathf.PerlinNoise(rb.position.x + timeOffset, rb.position.y);
                float noiseY = Mathf.PerlinNoise(rb.position.y + timeOffset, rb.position.z);
                float noiseZ = Mathf.PerlinNoise(rb.position.z + timeOffset, rb.position.x);

                // Map 0 -> 1 noise to a -1 -> 1 directional offset range
                Vector3 localTurbulenceDir = new Vector3(
                    (noiseX * 2f) - 1f,
                    (noiseY * 2f) - 1f,
                    (noiseZ * 2f) - 1f
                ).normalized;

                // Scale turbulence: stronger the further it travels from the fan source
                float currentTurbulenceIntensity = _turbulenceStrength * travelRatio * spreadEdgeLoss;
                Vector3 turbulenceForce = localTurbulenceDir * currentTurbulenceIntensity;
                
                // Add the turbulent velocity directly into the air vector stream
                airVelocity += turbulenceForce;
            }

            // 8. Calculate relative velocity and apply force sum
            Vector3 relVel = airVelocity - rb.linearVelocity;
            if(_toggle==true)
            {
                rb.AddForce(_dragCoefficient * relVel, ForceMode.Force);
            }
            else
            {
                rb.AddForce(Vector3.zero, ForceMode.Force);
            }
        }

        private void OnDrawGizmosSelected()
        {
            if (_boxCollider == null) _boxCollider = GetComponent<BoxCollider>();
            if (_boxCollider == null) return;

            float length = _boxCollider.size.z;
            Vector3 startPoint = transform.position - (transform.forward * (length * 0.5f));

            // Core beam visualization
            Gizmos.color = Color.cyan;
            Gizmos.DrawRay(startPoint, transform.forward * length);

            // Visualizing the expanding spread boundary bounds conceptually
            Gizmos.color = new Color(0f, 1f, 1f, 0.25f);
            Vector3 endPoint = startPoint + (transform.forward * length);
            Gizmos.DrawWireSphere(endPoint, _spreadFactor * length * 0.5f);
        }
    }
}
