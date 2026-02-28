using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class BallAerodynamics : MonoBehaviour
{
    [Header("Aerodynamics")]
    public float dragCoefficient = 0.015f;
    public float magnusCoefficient = 0.025f;
    public float maxAngularSpeed = 80f;

    private Rigidbody ballRigidbody;

    private void Awake()
    {
        ballRigidbody = GetComponent<Rigidbody>();
        ballRigidbody.maxAngularVelocity = maxAngularSpeed;
        ballRigidbody.interpolation = RigidbodyInterpolation.Interpolate;
        ballRigidbody.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
    }

    private void FixedUpdate()
    {
        Vector3 velocity = ballRigidbody.velocity;
        float speed = velocity.magnitude;
        if (speed < 0.01f)
        {
            return;
        }

        Vector3 dragForce = -velocity * (dragCoefficient * speed);
        Vector3 magnusForce = Vector3.Cross(ballRigidbody.angularVelocity, velocity) * magnusCoefficient;

        ballRigidbody.AddForce(dragForce + magnusForce, ForceMode.Acceleration);
    }
}
