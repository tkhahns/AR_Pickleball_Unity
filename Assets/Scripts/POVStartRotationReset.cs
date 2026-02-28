using UnityEngine;

public class POVStartRotationReset : MonoBehaviour
{
    public bool lockRollAndPitchInUpdate;

    private void Start()
    {
        transform.localRotation = Quaternion.identity;
    }

    private void LateUpdate()
    {
        if (!lockRollAndPitchInUpdate)
        {
            return;
        }

        Vector3 euler = transform.localEulerAngles;
        transform.localEulerAngles = new Vector3(0f, euler.y, 0f);
    }
}
