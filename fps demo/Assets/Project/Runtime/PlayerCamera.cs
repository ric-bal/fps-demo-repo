using System;
using UnityEngine;

public struct CameraInput
{
    public Vector2 look;
}

public class PlayerCamera : MonoBehaviour
{
    private Vector3 _eulerAngles;
    [SerializeField] private float sensitivity = 0.2f;


    public void Initialize(Transform target)
    {
        transform.position = target.position;
        transform.eulerAngles = _eulerAngles = target.eulerAngles;
    }


    public void UpdateRotation(CameraInput input)
    {
        _eulerAngles += new Vector3(-input.look.y, input.look.x) * sensitivity ;

        // stops camera "backflips", remove for free first person camera movement
        _eulerAngles.x = Mathf.Clamp(_eulerAngles.x, -89f, 89f);

        transform.eulerAngles = _eulerAngles; 
    }


    public void UpdatePosition(Transform target)
    {
        transform.position = target.position;
    }
}
