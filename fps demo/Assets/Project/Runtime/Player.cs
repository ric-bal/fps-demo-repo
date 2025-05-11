using UnityEngine;
using UnityEngine.InputSystem;

public class Player : MonoBehaviour
{
    [SerializeField] private PlayerCharacter playerCharacter;
    [SerializeField] private PlayerCamera playerCamera;

    private PlayerInputActions _inputActions;


    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        Cursor.lockState = CursorLockMode.Locked;

        playerCharacter.Initialize();   
        playerCamera.Initialize(playerCharacter.GetCameraTarget());

        _inputActions = new PlayerInputActions();
        _inputActions.Enable();
    }


    void OnDestroy()
    {
        _inputActions.Dispose();
    }


    // Update is called once per frame
    void Update()
    {
        var input = _inputActions.Gameplay;
        var deltaTime = Time.deltaTime;

        // camera
        var cameraInput = new CameraInput { look = input.Look.ReadValue<Vector2>() };    
        playerCamera.UpdateRotation(cameraInput);

        // character
        var characterInput = new CharacterInput
        {
            Rotation    = playerCamera.transform.rotation,
            Move        = input.Move.ReadValue<Vector2>(),
            Jump        = input.Jump.WasPressedThisFrame(),
            JumpSustain = input.Jump.IsPressed(),
            SlamSlide   = input.SlamSlide.IsPressed()
        };
        playerCharacter.UpdateInput(characterInput);
        playerCharacter.UpdateBody(deltaTime);


        #if UNITY_EDITOR
        if (Keyboard.current.tKey.wasPressedThisFrame)
        {
            var ray = new Ray(playerCamera.transform.position, playerCamera.transform.forward);
            if (Physics.Raycast(ray, out var hit))
            {
                Teleport(hit.point + new Vector3(0, 2, 0));
            }
        }
        #endif
    }


    void LateUpdate()
    {
        playerCamera.UpdatePosition(playerCharacter.GetCameraTarget());
    }


    public void Teleport(Vector3 position)
    {
        playerCharacter.SetPosition(position);
    }
}
