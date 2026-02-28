using UnityEngine;

public class PaddleHitController : MonoBehaviour
{
    [Header("References")]
    public Transform cameraTransform;

    [Header("Mouse 3D Control")]
    public float depthFromCamera = 0.55f;
    public float horizontalRange = 0.45f;
    public float verticalRange = 0.35f;
    public float followSharpness = 24f;
    public float defaultScreenX = 0.75f;
    public float defaultScreenY = 0.36f;
    public bool lockCursorOnPlay;

    [Header("Paddle Pose")]
    public Vector3 baseLocalEuler = new Vector3(15f, -90f, 0f);
    public Vector3 localFaceNormal = Vector3.right;

    [Header("Hit Physics")]
    public float restitution = 0.9f;
    public float paddleVelocityInfluence = 0.35f;
    public float forwardBoost = 1.1f;
    public float maxBallSpeed = 22f;
    public float spinFromTangential = 0.18f;
    public float spinFromOffCenter = 6f;
    public float hitCooldown = 0.03f;
    public bool requireBallTag;
    public string ballTag = "Ball";

    [Header("Fallback Detection")]
    public Rigidbody trackedBall;
    public bool enableProximityFallback = true;
    public float proximityHitDistance = 0.12f;

    private Rigidbody paddleRigidbody;
    private Collider[] paddleColliders;
    private Vector3 previousPosition;
    private Vector3 paddleVelocity;
    private Vector3 paddleAngularVelocity;
    private float lastHitTime;

    private void Awake()
    {
        if (TryGetComponent<Camera>(out _))
        {
            Debug.LogError("PaddleHitController is attached to a Camera. Attach it to the paddle object instead.");
            enabled = false;
            return;
        }

        paddleRigidbody = GetComponent<Rigidbody>();

        if (cameraTransform == null && Camera.main != null)
        {
            cameraTransform = Camera.main.transform;
        }

        if (cameraTransform != null && transform == cameraTransform)
        {
            Debug.LogError("PaddleHitController target transform is the camera. Move this component to the paddle object.");
            enabled = false;
            return;
        }

        if (paddleRigidbody != null)
        {
            paddleRigidbody.isKinematic = true;
            paddleRigidbody.useGravity = false;
            paddleRigidbody.interpolation = RigidbodyInterpolation.Interpolate;
            paddleRigidbody.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;
        }

        if (lockCursorOnPlay)
        {
            Cursor.lockState = CursorLockMode.Locked;
            Cursor.visible = false;
        }

        paddleColliders = GetComponentsInChildren<Collider>();

        previousPosition = transform.position;
    }

    private void FixedUpdate()
    {
        if (cameraTransform == null)
        {
            return;
        }

        Vector3 worldPosition = GetTargetWorldPosition();
        Quaternion worldRotation = GetTargetWorldRotation(worldPosition);

        if (paddleRigidbody != null)
        {
            float lerpFactor = 1f - Mathf.Exp(-followSharpness * Time.fixedDeltaTime);
            Vector3 blendedPosition = Vector3.Lerp(paddleRigidbody.position, worldPosition, lerpFactor);
            Quaternion blendedRotation = Quaternion.Slerp(paddleRigidbody.rotation, worldRotation, lerpFactor);

            paddleRigidbody.MovePosition(blendedPosition);
            paddleRigidbody.MoveRotation(blendedRotation);
        }
        else
        {
            transform.SetPositionAndRotation(worldPosition, worldRotation);
        }

        Vector3 currentPosition = transform.position;
        paddleVelocity = (currentPosition - previousPosition) / Mathf.Max(Time.fixedDeltaTime, 0.0001f);
        Quaternion deltaRotation = worldRotation * Quaternion.Inverse(transform.rotation);
        deltaRotation.ToAngleAxis(out float deltaAngle, out Vector3 deltaAxis);
        if (deltaAngle > 180f)
        {
            deltaAngle -= 360f;
        }
        paddleAngularVelocity = deltaAxis * (deltaAngle * Mathf.Deg2Rad / Mathf.Max(Time.fixedDeltaTime, 0.0001f));
        previousPosition = currentPosition;

        if (enableProximityFallback)
        {
            TryProximityHit();
        }
    }

    private void TryProximityHit()
    {
        Rigidbody candidateBall = trackedBall;

        if (candidateBall == null)
        {
            if (!string.IsNullOrWhiteSpace(ballTag))
            {
                try
                {
                    GameObject ballObject = GameObject.FindWithTag(ballTag);
                    if (ballObject != null)
                    {
                        candidateBall = ballObject.GetComponent<Rigidbody>();
                    }
                }
                catch (UnityException)
                {
                }
            }

            if (candidateBall == null)
            {
                Rigidbody[] rigidbodies = FindObjectsByType<Rigidbody>(FindObjectsSortMode.None);
                for (int index = 0; index < rigidbodies.Length; index++)
                {
                    Rigidbody body = rigidbodies[index];
                    if (body == null || body == paddleRigidbody)
                    {
                        continue;
                    }

                    if (body.gameObject.name.IndexOf("ball", System.StringComparison.OrdinalIgnoreCase) >= 0)
                    {
                        candidateBall = body;
                        break;
                    }
                }
            }
        }

        if (candidateBall == null)
        {
            return;
        }

        Vector3 ballPosition = candidateBall.worldCenterOfMass;
        Vector3 closestPointOnPaddle = GetClosestPointOnPaddle(ballPosition);
        float distance = Vector3.Distance(closestPointOnPaddle, ballPosition);

        if (distance <= proximityHitDistance)
        {
            TryHitBall(candidateBall, candidateBall.gameObject, closestPointOnPaddle);
        }
    }

    private Vector3 GetClosestPointOnPaddle(Vector3 worldPoint)
    {
        if (paddleColliders == null || paddleColliders.Length == 0)
        {
            return transform.position;
        }

        Vector3 bestPoint = transform.position;
        float bestDistance = float.MaxValue;

        for (int index = 0; index < paddleColliders.Length; index++)
        {
            Collider paddleCollider = paddleColliders[index];
            if (paddleCollider == null || !paddleCollider.enabled)
            {
                continue;
            }

            Vector3 candidatePoint = paddleCollider.ClosestPoint(worldPoint);
            float candidateDistance = (candidatePoint - worldPoint).sqrMagnitude;
            if (candidateDistance < bestDistance)
            {
                bestDistance = candidateDistance;
                bestPoint = candidatePoint;
            }
        }

        return bestPoint;
    }

    private Vector3 GetTargetWorldPosition()
    {
        Camera sourceCamera = cameraTransform.GetComponent<Camera>();

        Vector3 localOffset;
        if (sourceCamera != null)
        {
            Vector3 mousePosition = Input.mousePosition;

            if (lockCursorOnPlay)
            {
                mousePosition = new Vector3(Screen.width * defaultScreenX, Screen.height * defaultScreenY, 0f);
            }

            Ray mouseRay = sourceCamera.ScreenPointToRay(mousePosition);
            Plane paddlePlane = new Plane(cameraTransform.forward, cameraTransform.position + cameraTransform.forward * depthFromCamera);

            if (paddlePlane.Raycast(mouseRay, out float enterDistance))
            {
                Vector3 hitPoint = mouseRay.GetPoint(enterDistance);
                localOffset = cameraTransform.InverseTransformPoint(hitPoint);
            }
            else
            {
                localOffset = new Vector3(depthFromCamera * 0.5f, -0.1f, depthFromCamera);
            }
        }
        else
        {
            localOffset = new Vector3(depthFromCamera * 0.5f, -0.1f, depthFromCamera);
        }

        localOffset.z = depthFromCamera;
        localOffset.x = Mathf.Clamp(localOffset.x, -horizontalRange, horizontalRange);
        localOffset.y = Mathf.Clamp(localOffset.y, -verticalRange, verticalRange);

        return cameraTransform.TransformPoint(localOffset);
    }

    private Quaternion GetTargetWorldRotation(Vector3 worldPosition)
    {
        Vector3 toPaddle = worldPosition - cameraTransform.position;
        if (toPaddle.sqrMagnitude < 0.0001f)
        {
            return cameraTransform.rotation * Quaternion.Euler(baseLocalEuler);
        }

        Quaternion lookRotation = Quaternion.LookRotation(toPaddle.normalized, Vector3.up);
        return lookRotation * Quaternion.Euler(baseLocalEuler);
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.contactCount == 0)
        {
            return;
        }

        ContactPoint contact = collision.GetContact(0);
        TryHitBall(collision.rigidbody, collision.gameObject, contact.point);
    }

    private void OnCollisionStay(Collision collision)
    {
        if (collision.contactCount == 0)
        {
            return;
        }

        ContactPoint contact = collision.GetContact(0);
        TryHitBall(collision.rigidbody, collision.gameObject, contact.point);
    }

    private void OnTriggerEnter(Collider other)
    {
        Rigidbody otherRigidbody = other.attachedRigidbody;
        Vector3 contactPoint = other.ClosestPoint(transform.position);
        TryHitBall(otherRigidbody, other.gameObject, contactPoint);
    }

    private void OnTriggerStay(Collider other)
    {
        Rigidbody otherRigidbody = other.attachedRigidbody;
        Vector3 contactPoint = other.ClosestPoint(transform.position);
        TryHitBall(otherRigidbody, other.gameObject, contactPoint);
    }

    private void TryHitBall(Rigidbody ballRigidbody, GameObject ballObject, Vector3 contactPoint)
    {
        if (ballRigidbody == null)
        {
            return;
        }

        if (requireBallTag && !ballObject.CompareTag(ballTag))
        {
            return;
        }

        if (Time.time - lastHitTime < hitCooldown)
        {
            return;
        }

        Vector3 faceNormal = transform.TransformDirection(localFaceNormal).normalized;
        Vector3 toBall = (ballRigidbody.worldCenterOfMass - contactPoint).normalized;
        if (Vector3.Dot(faceNormal, toBall) < 0f)
        {
            faceNormal = -faceNormal;
        }

        Vector3 incomingVelocity = ballRigidbody.velocity;
        Vector3 reflectedVelocity = Vector3.Reflect(incomingVelocity, faceNormal) * restitution;

        Vector3 tangentialVelocity = paddleVelocity - Vector3.Project(paddleVelocity, faceNormal);
        Vector3 outgoingVelocity = reflectedVelocity;
        outgoingVelocity += tangentialVelocity * paddleVelocityInfluence;
        outgoingVelocity += cameraTransform.forward * forwardBoost;

        if (outgoingVelocity.magnitude > maxBallSpeed)
        {
            outgoingVelocity = outgoingVelocity.normalized * maxBallSpeed;
        }

        ballRigidbody.velocity = outgoingVelocity;

        Vector3 contactOffset = contactPoint - ballRigidbody.worldCenterOfMass;
        Vector3 spinFromSwipe = Vector3.Cross(faceNormal, tangentialVelocity) * spinFromTangential;
        Vector3 spinFromOffset = Vector3.Cross(contactOffset, outgoingVelocity) * spinFromOffCenter;
        ballRigidbody.angularVelocity += spinFromSwipe + spinFromOffset + paddleAngularVelocity * 0.05f;

        lastHitTime = Time.time;
    }
}
