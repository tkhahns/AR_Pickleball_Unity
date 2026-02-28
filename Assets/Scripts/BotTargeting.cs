using UnityEngine;

public class BotTargeting : MonoBehaviour
{
    public Transform ball;
    public Transform aimTarget;
    public Transform[] targets;

    [Header("Optional Auto Aim")]
    public bool autoAimAtBall;
    public float turnSpeed = 5f;

    private void Update()
    {
        if (!autoAimAtBall || ball == null)
        {
            return;
        }

        Vector3 lookDirection = ball.position - transform.position;
        lookDirection.y = 0f;
        if (lookDirection.sqrMagnitude < 0.0001f)
        {
            return;
        }

        Quaternion targetRotation = Quaternion.LookRotation(lookDirection.normalized, Vector3.up);
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, turnSpeed * Time.deltaTime);
    }
}
