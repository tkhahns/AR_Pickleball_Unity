using System;
using UnityEngine;

public class BotShotProfile : MonoBehaviour
{
    [Serializable]
    public struct ShotConfig
    {
        public float upForce;
        public float hitForce;
    }

    public ShotConfig topSpin;
    public ShotConfig flat;
}
