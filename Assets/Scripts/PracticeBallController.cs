using UnityEngine;

public class PracticeBallController : MonoBehaviour
{
    [Header("References")]
    public Transform servePoint;

    [Header("Controls")]
    public KeyCode resetKey = KeyCode.R;

    private Rigidbody ballRigidbody;
    private Vector3 initialPosition;
    private Quaternion initialRotation;

    private void Awake()
    {
        ballRigidbody = GetComponent<Rigidbody>();
        initialPosition = transform.position;
        initialRotation = transform.rotation;

        if (servePoint == null && Camera.main != null)
        {
            servePoint = Camera.main.transform;
        }
    }

    private void Update()
    {
        if (Input.GetKeyDown(resetKey))
        {
            ResetBall();
        }
    }

    public void ResetBall()
    {
        if (ballRigidbody != null)
        {
            ballRigidbody.velocity = Vector3.zero;
            ballRigidbody.angularVelocity = Vector3.zero;
        }

        if (servePoint != null)
        {
            transform.position = servePoint.TransformPoint(new Vector3(0.18f, -0.12f, 0.85f));
            transform.rotation = Quaternion.identity;
            return;
        }

        transform.position = initialPosition;
        transform.rotation = initialRotation;
    }
}
